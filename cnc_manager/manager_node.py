#!/usr/bin/env python3
"""CNC Manager node: the single entry point between the rest of ROS and grbl_ros."""

import json  # used to serialize the /cnc/status message as a JSON string
import threading  # used for a lock that protects shared state between callbacks
from collections import deque  # FIFO queue for G-code lines that wait to be sent

import rclpy  # ROS 2 Python client library
from rclpy.action import ActionClient  # client side of a ROS 2 action
from rclpy.callback_groups import ReentrantCallbackGroup  # lets callbacks run in parallel
from rclpy.executors import MultiThreadedExecutor  # executor that runs callbacks on several threads
from rclpy.node import Node  # base class for every ROS 2 node

from geometry_msgs.msg import Pose  # message type of /<machine_id>/machine_position from grbl_ros
from geometry_msgs.msg import Twist  # message type of /cnc/jog_cmd (velocity command)
from sensor_msgs.msg import JointState  # message type of the position we publish (one joint per axis)
from std_msgs.msg import String  # message type of /cnc/send_line, /cnc/send_file and /cnc/status
from std_srvs.srv import Trigger  # service type of /cnc/home, /cnc/unlock and /cnc/stop

from grbl_msgs.action import SendGcodeCmd  # action: send one G-code line to grbl_ros
from grbl_msgs.action import SendGcodeFile  # action: let grbl_ros stream a whole G-code file
from grbl_msgs.msg import State  # message type of /<machine_id>/state from grbl_ros
from grbl_msgs.srv import Stop  # service type of /<machine_id>/stop from grbl_ros

# Names of the GRBL states, index = numeric value used by grbl_ros (grbl_ros/_logging.py STATE)
GRBL_STATE_NAMES = ['IDLE', 'RUN', 'HOLD', 'JOG', 'ALARM', 'DOOR', 'CHECK', 'HOME', 'SLEEP']  # lookup table

POLL_COMMAND = '?'  # GRBL real-time command that asks for a status report (state + position)


class CncManager(Node):  # our node class, inherits all ROS functionality from Node
    """Manage one GRBL CNC machine through the grbl_ros node."""

    def __init__(self):  # constructor, runs once when the node is created
        super().__init__('cnc_manager')  # register the node with the name "cnc_manager"

        # ---------------- parameters ----------------
        self.declare_parameter('machine_id', 'cnc_001')  # must match machine_id of grbl_ros
        self.declare_parameter('poll_rate_hz', 2.0)  # how often we ask GRBL for state/position when idle
        self.declare_parameter('status_rate_hz', 2.0)  # how often /cnc/status is published
        self.declare_parameter('jog_period_s', 0.1)  # length of one jog step in seconds
        self.declare_parameter('max_jog_feed_mm_min', 1000.0)  # upper limit for jog speed (mm/min)
        self.declare_parameter('state_timeout_s', 3.0)  # no state for this long -> machine counts as offline
        self.declare_parameter('joint_names', ['x_axis', 'y_axis', 'z_axis'])  # joint names for JointState
        self.declare_parameter('use_grbl_stop_service', False)  # grbl_ros Stop service is broken, see proposal

        self.machine_id = self.get_parameter('machine_id').value  # read machine id
        poll_rate = self.get_parameter('poll_rate_hz').value  # read poll rate
        status_rate = self.get_parameter('status_rate_hz').value  # read status rate
        self.jog_period = self.get_parameter('jog_period_s').value  # read jog step length
        self.max_jog_feed = self.get_parameter('max_jog_feed_mm_min').value  # read jog speed limit
        self.state_timeout = self.get_parameter('state_timeout_s').value  # read offline timeout
        self.joint_names = list(self.get_parameter('joint_names').value)  # read joint names
        self.use_grbl_stop = self.get_parameter('use_grbl_stop_service').value  # read stop-service flag

        # ---------------- internal state ----------------
        self.lock = threading.Lock()  # protects the variables below, callbacks run on several threads
        self.state_name = 'UNKNOWN'  # last GRBL state name received (IDLE, RUN, ALARM, ...)
        self.state_code = -1  # last GRBL state number received, -1 = nothing received yet
        self.last_state_time = None  # ROS time of the last state message, None = never
        self.position = [0.0, 0.0, 0.0]  # last machine position x, y, z in metres
        self.queue = deque()  # G-code lines waiting to be sent, one at a time
        self.active_goal = None  # goal handle of the action that is running right now
        self.active_command = None  # text of the command / file that is running right now
        self.active_is_poll = False  # True if the running goal is only a "?" status poll
        self.waiting_for_accept = False  # True between sending a goal and getting accept/reject
        self.last_feedback = ''  # last feedback text from grbl_ros
        self.last_result = ''  # short text describing the last finished command
        self.counter = 0  # seconds since start, used for the "alive" log message

        cb = ReentrantCallbackGroup()  # one group for everything so callbacks never block each other

        # ---------------- connection to grbl_ros (see diagram, right side) ----------------
        grbl = f'/{self.machine_id}'  # prefix of all grbl_ros topics, e.g. "/cnc_001"
        self.create_subscription(State, f'{grbl}/state', self.on_grbl_state, 10, callback_group=cb)  # GRBL state
        self.create_subscription(Pose, f'{grbl}/machine_position', self.on_grbl_position, 10,
                                 callback_group=cb)  # GRBL machine position (metres)
        self.cmd_client = ActionClient(self, SendGcodeCmd, f'{grbl}/send_gcode_cmd',
                                       callback_group=cb)  # client for single G-code lines
        self.file_client = ActionClient(self, SendGcodeFile, f'{grbl}/send_gcode_file',
                                        callback_group=cb)  # client for whole G-code files
        self.stop_client = self.create_client(Stop, f'{grbl}/stop', callback_group=cb)  # grbl_ros stop service

        # ---------------- interface of the manager (see diagram, left side) ----------------
        self.create_subscription(String, '/cnc/send_line', self.on_send_line, 10, callback_group=cb)  # G-code in
        self.create_subscription(String, '/cnc/send_file', self.on_send_file, 10, callback_group=cb)  # file path in
        self.create_subscription(Twist, '/cnc/jog_cmd', self.on_jog_cmd, 10, callback_group=cb)  # jog velocity in
        self.create_service(Trigger, '/cnc/home', self.on_home, callback_group=cb)  # start homing cycle
        self.create_service(Trigger, '/cnc/unlock', self.on_unlock, callback_group=cb)  # clear alarm
        self.create_service(Trigger, '/cnc/stop', self.on_stop, callback_group=cb)  # stop everything
        self.status_pub = self.create_publisher(String, '/cnc/status', 10)  # JSON status out
        self.position_pub = self.create_publisher(JointState, '/machine_position', 10)  # position out

        # ---------------- timers ----------------
        self.create_timer(1.0, self.timer_callback, callback_group=cb)  # 1 Hz "alive" log
        self.create_timer(0.05, self.process_queue, callback_group=cb)  # 20 Hz: send next queued line
        if poll_rate > 0.0:  # polling can be switched off with poll_rate_hz:=0
            self.create_timer(1.0 / poll_rate, self.poll_status, callback_group=cb)  # ask GRBL for status
        self.create_timer(1.0 / status_rate, self.publish_status, callback_group=cb)  # publish /cnc/status

        self.get_logger().info(f'CNC Manager started for machine "{self.machine_id}".')  # startup message

    # =================================================================
    # Callbacks for data coming FROM grbl_ros
    # =================================================================
    def on_grbl_state(self, msg: State):  # called each time grbl_ros publishes its state
        with self.lock:  # protect shared variables
            changed = msg.state_name != self.state_name  # remember if the state changed
            self.state_name = msg.state_name or 'UNKNOWN'  # store the state name
            self.state_code = int(msg.state)  # store the state number
            self.last_state_time = self.get_clock().now()  # remember when we got it
        if changed:  # only log on changes, otherwise the log gets flooded
            self.get_logger().info(f'GRBL state: {msg.state_name}')  # log the new state
            self.publish_status()  # push the new state out right away

    def on_grbl_position(self, msg: Pose):  # called each time grbl_ros publishes the position
        with self.lock:  # protect shared variables
            self.position = [msg.position.x, msg.position.y, msg.position.z]  # store x, y, z (metres)
        js = JointState()  # create the outgoing message
        js.header.stamp = self.get_clock().now().to_msg()  # timestamp = now
        js.name = self.joint_names  # one joint per axis
        js.position = list(self.position)  # joint positions = axis positions in metres
        self.position_pub.publish(js)  # send it to /machine_position

    # =================================================================
    # Callbacks for commands coming FROM the rest of ROS
    # =================================================================
    def on_send_line(self, msg: String):  # one or more G-code lines arrived on /cnc/send_line
        for line in msg.data.splitlines():  # a message may contain several lines
            line = line.split(';')[0].strip()  # remove comments (after ";") and whitespace
            if line:  # ignore empty lines
                self.enqueue(line)  # put it in the queue

    def on_send_file(self, msg: String):  # a file path arrived on /cnc/send_file
        path = msg.data.strip()  # remove whitespace around the path
        if not path:  # an empty path makes no sense
            self.get_logger().warn('Empty file path on /cnc/send_file, ignored.')  # tell the user
            return  # nothing to do
        if not self.is_idle_for_new_job():  # a file needs the machine for itself
            self.get_logger().warn(f'Machine busy, file "{path}" rejected.')  # tell the user
            return  # do not start
        # NOTE: the path is opened by grbl_ros, so it must exist on the computer running grbl_node
        goal = SendGcodeFile.Goal()  # create the action goal
        goal.file_path = path  # put the path into the goal
        self.send_goal(self.file_client, goal, f'FILE {path}', is_poll=False)  # start streaming

    def on_jog_cmd(self, msg: Twist):  # a jog velocity arrived on /cnc/jog_cmd (m/s like cmd_vel)
        vx, vy, vz = msg.linear.x, msg.linear.y, msg.linear.z  # velocities per axis in m/s
        speed = (vx ** 2 + vy ** 2 + vz ** 2) ** 0.5  # total speed in m/s
        if speed == 0.0:  # zero velocity = stop jogging
            return  # the last small jog step just finishes (real jog-cancel needs grbl_ros change)
        with self.lock:  # protect shared variables
            busy = self.active_goal is not None or self.waiting_for_accept  # something running?
            busy_with_poll = self.active_is_poll  # is it only a status poll?
            queued = len(self.queue)  # number of waiting lines
        if (busy and not busy_with_poll) or queued > 0:  # never mix jogging with a running job
            return  # drop this jog command, the next one will come soon
        if self.state_name not in ('IDLE', 'JOG'):  # GRBL only accepts jog in Idle or Jog state
            self.get_logger().warn(f'Jog ignored, machine is {self.state_name}.', throttle_duration_sec=2.0)  # info
            return  # do not send
        feed = min(speed * 1000.0 * 60.0, self.max_jog_feed)  # m/s -> mm/min, limited
        scale = feed / (speed * 1000.0 * 60.0)  # factor if we had to limit the speed
        dt = self.jog_period  # length of one jog step in seconds
        dx = vx * 1000.0 * dt * scale  # distance in mm on X for this step
        dy = vy * 1000.0 * dt * scale  # distance in mm on Y for this step
        dz = vz * 1000.0 * dt * scale  # distance in mm on Z for this step
        cmd = f'$J=G91 G21 X{dx:.3f} Y{dy:.3f} Z{dz:.3f} F{feed:.1f}'  # GRBL jog: relative, millimetres
        self.enqueue(cmd)  # send it through the normal queue

    def on_home(self, request, response):  # /cnc/home service was called
        self.enqueue('$H')  # $H = GRBL homing cycle
        response.success = True  # the request was accepted
        response.message = 'Homing ($H) queued.'  # text for the caller
        return response  # send answer back

    def on_unlock(self, request, response):  # /cnc/unlock service was called
        self.enqueue('$X')  # $X = GRBL kill alarm lock
        response.success = True  # the request was accepted
        response.message = 'Unlock ($X) queued.'  # text for the caller
        return response  # send answer back

    def on_stop(self, request, response):  # /cnc/stop service was called
        with self.lock:  # protect shared variables
            dropped = len(self.queue)  # how many lines we throw away
            self.queue.clear()  # forget every line that was not sent yet
            goal = self.active_goal  # the goal that runs right now (if any)
        notes = [f'{dropped} queued line(s) dropped']  # collect what we did for the answer
        if goal is not None:  # something is running on grbl_ros
            goal.cancel_goal_async()  # ask grbl_ros to cancel (it rejects until the proposal is done)
            notes.append('cancel requested (grbl_ros does not support cancel yet)')  # tell the caller
        if self.use_grbl_stop and self.stop_client.service_is_ready():  # only if enabled and available
            req = Stop.Request()  # create the service request
            req.machine_id = self.machine_id  # which machine
            req.command = 's'  # 's' = disable steppers in grbl_ros
            self.stop_client.call_async(req)  # fire and forget
            notes.append('grbl_ros stop service called')  # tell the caller
        else:  # stop service not used
            notes.append('NO hardware stop sent - use the E-stop! (see grbl_ros proposal)')  # warn the caller
        response.success = self.use_grbl_stop  # only "true" if the machine itself was told to stop
        response.message = '; '.join(notes)  # join all notes into one text
        self.get_logger().warn(f'STOP: {response.message}')  # log it clearly
        return response  # send answer back

    # =================================================================
    # Queue and action handling
    # =================================================================
    def enqueue(self, line: str):  # add one G-code line to the queue
        with self.lock:  # protect shared variables
            self.queue.append(line)  # append at the end (FIFO)
        self.get_logger().debug(f'Queued: {line}')  # debug output only

    def is_idle_for_new_job(self) -> bool:  # True if nothing is running or waiting
        with self.lock:  # protect shared variables
            busy = (self.active_goal is not None and not self.active_is_poll) or self.waiting_for_accept  # busy?
            return not busy and len(self.queue) == 0  # idle = not busy and queue empty

    def process_queue(self):  # timer: send the next queued line if nothing else is running
        with self.lock:  # protect shared variables
            if self.active_goal is not None or self.waiting_for_accept:  # one goal at a time on the serial port
                return  # wait for the running one to finish
            if not self.queue:  # nothing to send
                return  # done
            line = self.queue.popleft()  # take the oldest line
        goal = SendGcodeCmd.Goal()  # create the action goal
        goal.command = line  # put the G-code into the goal
        self.send_goal(self.cmd_client, goal, line, is_poll=False)  # send it to grbl_ros

    def poll_status(self):  # timer: ask GRBL for state + position while nothing else runs
        with self.lock:  # protect shared variables
            if self.active_goal is not None or self.waiting_for_accept or self.queue:  # busy?
                return  # while busy, grbl_ros polls by itself (gcodeCallback sends "?")
        goal = SendGcodeCmd.Goal()  # create the action goal
        goal.command = POLL_COMMAND  # "?" = status report request
        self.send_goal(self.cmd_client, goal, POLL_COMMAND, is_poll=True)  # send it to grbl_ros

    def send_goal(self, client, goal, description, is_poll):  # common code to start any action goal
        if not client.server_is_ready():  # grbl_node not running?
            if not is_poll:  # do not spam the log for polls
                self.get_logger().error(f'grbl_ros action server not available, "{description}" dropped.')  # error
            return  # cannot send
        with self.lock:  # protect shared variables
            self.waiting_for_accept = True  # block other goals until accepted/rejected
            self.active_command = description  # remember what we send
            self.active_is_poll = is_poll  # remember if it is just a poll
        if not is_poll:  # only log real commands
            self.get_logger().info(f'Sending: {description}')  # log what we send
        future = client.send_goal_async(goal, feedback_callback=self.on_feedback)  # send the goal
        future.add_done_callback(self.on_goal_response)  # called when grbl_ros accepts or rejects

    def on_goal_response(self, future):  # grbl_ros answered whether it accepts the goal
        goal_handle = future.result()  # the handle of our goal
        if goal_handle is None or not goal_handle.accepted:  # rejected or failed
            with self.lock:  # protect shared variables
                self.waiting_for_accept = False  # free the queue again
                self.last_result = f'REJECTED: {self.active_command}'  # remember the failure
                self.active_command = None  # nothing running
            self.get_logger().warn(self.last_result)  # log it
            return  # done
        with self.lock:  # protect shared variables
            self.active_goal = goal_handle  # this goal is now running
            self.waiting_for_accept = False  # accept phase is over
        result_future = goal_handle.get_result_async()  # ask for the final result
        result_future.add_done_callback(self.on_result)  # called when the goal is finished

    def on_feedback(self, feedback_msg):  # grbl_ros sends progress while a goal runs
        with self.lock:  # protect shared variables
            self.last_feedback = feedback_msg.feedback.status  # store the progress text

    def on_result(self, future):  # the running goal finished
        wrapped = future.result()  # contains .status and .result
        success = wrapped is not None and wrapped.result.success  # True if grbl_ros said ok
        with self.lock:  # protect shared variables
            command = self.active_command  # what was running
            was_poll = self.active_is_poll  # was it only a poll?
            self.active_goal = None  # nothing running any more
            self.active_command = None  # nothing running any more
            self.active_is_poll = False  # reset the poll flag
            self.last_feedback = ''  # clear old progress text
            if not was_poll:  # polls do not change the last result
                self.last_result = f'{"OK" if success else "FAILED"}: {command}'  # remember result
        if was_poll:  # grbl_ros reports "?" as failure because GRBL answers without "ok"
            return  # that is expected, state/position were published anyway
        if success:  # command worked
            self.get_logger().info(f'Done: {command}')  # log success
        else:  # command failed
            self.get_logger().error(f'Failed: {command} (check grbl_ros log for the GRBL error)')  # log failure

    # =================================================================
    # Status output
    # =================================================================
    def publish_status(self):  # build and publish /cnc/status as JSON
        now = self.get_clock().now()  # current ROS time
        with self.lock:  # protect shared variables
            if self.last_state_time is None:  # never got a state
                age = None  # unknown age
            else:  # got a state at least once
                age = (now - self.last_state_time).nanoseconds / 1e9  # age in seconds
            online = age is not None and age < self.state_timeout  # recent state = online
            status = {  # dictionary that becomes the JSON
                'machine_id': self.machine_id,  # which machine
                'online': online,  # do we receive data from grbl_ros?
                'grbl_node_available': self.cmd_client.server_is_ready(),  # is grbl_node running?
                'state': self.state_name if online else 'OFFLINE',  # GRBL state or OFFLINE
                'state_code': self.state_code,  # GRBL state number (see GRBL_STATE_NAMES)
                'state_age_s': age,  # how old the state is
                'position_m': dict(zip(('x', 'y', 'z'), self.position)),  # last position in metres
                'busy': (self.active_goal is not None and not self.active_is_poll)
                or self.waiting_for_accept,  # True while a real command/file runs
                'active_command': None if self.active_is_poll else self.active_command,  # what runs now
                'feedback': self.last_feedback,  # progress text from grbl_ros
                'queue_length': len(self.queue),  # number of waiting lines
                'last_result': self.last_result,  # result of the last finished command
            }  # end of dictionary
        self.status_pub.publish(String(data=json.dumps(status)))  # publish as JSON text

    def timer_callback(self):  # 1 Hz timer, prints an "alive" message every minute
        self.counter += 1  # one more second
        if self.counter % 60 == 0:  # every 60 seconds
            self.get_logger().info(f'Node is running since {self.counter // 60} minutes')  # alive log


def main():  # entry point used by "ros2 run cnc_manager manager_node"
    rclpy.init()  # start ROS 2
    node = CncManager()  # create our node
    executor = MultiThreadedExecutor()  # several threads so action results and timers run in parallel
    executor.add_node(node)  # let the executor handle our node
    try:  # catch Ctrl+C
        executor.spin()  # run until shutdown
    except KeyboardInterrupt:  # user pressed Ctrl+C
        pass  # normal exit
    finally:  # always clean up
        executor.shutdown()  # stop executor threads
        node.destroy_node()  # remove the node
        rclpy.try_shutdown()  # stop ROS 2 if not already stopped


if __name__ == '__main__':  # file started directly with python3
    main()  # run main
