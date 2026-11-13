import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

from grbl_msgs.msg import State
from grbl_msgs.action import SendGcodeCmd
from grbl_msgs.srv import Stop as StopGrbl

import time

class CncManager(Node):
    def __init__(self):
        super().__init__('CncManager')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('machine_id', 'cnc_001'),
                ('limits', {'x': 300.0, 'y': 200.0, 'z': 150.0}),
                ('max_jog_speed', 50.0),
                ('deadman_timeout', 1.0),
            ])

        self.id = self.get_parameter('machine_id').get_parameter_value().string_value
        self.limits = self.get_parameter('limits').get_parameter_value().string_value
        self.max_jog_speed = self.get_parameter('max_jog_speed').get_parameter_value().double_value
        self.deadman_timeout = self.get_parameter('deadman_timeout').get_parameter_value().double_value

        self.state = 'DISCONNECTED'
        self.homed = False
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.goal_in_flight = False
        self.last_state_time = None
        self.last_jog_time = None

        callback_group = ReentrantCallbackGroup()

        self.sub_state = self.create_subscription(
            State,
            f'{self.id}/state',
            self.on_grbl_state,
            10,
            callback_group=callback_group)

        self.sub_position = self.create_subscription(
            Pose,
            f'{self.id}/machine_position',
            self.on_position,
            10,
            callback_group=callback_group)

        self.sub_jog = self.create_subscription(
            String,
            'cnc/jog_cmd',
            self.on_jog_cmd,
            10,
            callback_group=callback_group)

        self.pub_joint_states = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_status = self.create_publisher(String, 'cnc/status', 10)

        self.srv_home = self.create_service(Trigger, 'cnc/home', self.on_home, callback_group=callback_group)
        self.srv_unlock = self.create_service(Trigger, 'cnc/unlock', self.on_unlock, callback_group=callback_group)
        self.srv_stop = self.create_service(Trigger, 'cnc/stop', self.on_stop, callback_group=callback_group)

        self.action_client = ActionClient(self, SendGcodeCmd, f'{self.id}/send_gcode_cmd')

        self.timer_poll = self.create_timer(0.2, self.on_poll_timer, callback_group=callback_group)  # 5 Hz
        self.timer_watchdog = self.create_timer(0.1, self.on_watchdog_timer, callback_group=callback_group)  # 10 Hz

        self.get_logger().info(f'CncManager initialized for machine {self.id}')

    def on_grbl_state(self, msg: State):
        self.state = msg.state_name
        self.last_state_time = time.time()
        self.publish_status()

    def on_position(self, msg: Pose):
        self.position['x'] = msg.position.x
        self.position['y'] = msg.position.y
        self.position['z'] = msg.position.z
        self.publish_joint_states()

    def on_jog_cmd(self, msg: String):
        if self.state not in ['IDLE', 'JOG']:
            self.get_logger().warn(f'Cannot jog in state {self.state}')
            return

        self.last_jog_time = time.time()
        command = msg.data

        # Clamp jog command to max speed
        if 'F' in command:
            parts = command.split('F')
            try:
                speed = float(parts[1].split()[0])
                if speed > self.max_jog_speed:
                    command = f"{parts[0]}F{self.max_jog_speed}"
            except (ValueError, IndexError):
                pass

        self.send_to_grbl(command)

    def on_home(self, request, response):
        if self.send_to_grbl('$H'):
            response.success = True
            response.message = 'Homing started'
            self.homed = True
        else:
            response.success = False
            response.message = 'Failed to send home command'
        return response

    def on_unlock(self, request, response):
        if self.send_to_grbl('$X'):
            response.success = True
            response.message = 'Unlock command sent'
        else:
            response.success = False
            response.message = 'Failed to send unlock command'
        return response

    def on_stop(self, request, response):
        if self.send_to_grbl('!'):
            response.success = True
            response.message = 'Stop command sent'
        else:
            response.success = False
            response.message = 'Failed to send stop command'
        return response

    def on_poll_timer(self):
        if self.state == 'IDLE' and not self.goal_in_flight:
            self.send_to_grbl('?')

    def on_watchdog_timer(self):
        current_time = time.time()

        # Check for disconnection (no state update > 1 second)
        if self.last_state_time is not None:
            if current_time - self.last_state_time > 1.0:
                self.state = 'DISCONNECTED'
                self.publish_status()

        # Check for deadman timeout on jog
        if self.last_jog_time is not None:
            if current_time - self.last_jog_time > self.deadman_timeout:
                self.send_to_grbl('!')
                self.last_jog_time = None

    def send_to_grbl(self, command: str) -> bool:
        if self.goal_in_flight:
            return False

        self.goal_in_flight = True
        goal = SendGcodeCmd.Goal()
        goal.command = command

        self.action_client.send_goal_async(goal).add_done_callback(self.on_grbl_result)
        return True

    def on_grbl_result(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected')
                self.goal_in_flight = False
                return

            goal_handle.get_result_async().add_done_callback(self.on_grbl_result_received)
        except Exception as e:
            self.get_logger().error(f'Error sending goal: {e}')
            self.goal_in_flight = False

    def on_grbl_result_received(self, future):
        try:
            result = future.result().result
            if result.success:
                self.get_logger().debug(f'Command successful: {result.status}')
            else:
                self.get_logger().warn(f'Command failed: {result.status}')
        except Exception as e:
            self.get_logger().error(f'Error receiving result: {e}')
        finally:
            self.goal_in_flight = False

    def publish_status(self):
        msg = String()
        msg.data = f'{{machine_id: {self.id}, state: {self.state}, homed: {self.homed}}}'
        self.pub_status.publish(msg)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['x', 'y', 'z']
        msg.position = [self.position['x'], self.position['y'], self.position['z']]
        self.pub_joint_states.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    cnc = CncManager()
    rclpy.spin(cnc)
    cnc.destroy_node()
    rclpy.shutdown()