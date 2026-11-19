# CNC Manager – Quick Usage

## Build
```bash
cd ~/ros2_packages/grbl_ros_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Run (2 terminals)
```bash
ros2 run grbl_ros grbl_node --ros-args --params-file src/grbl_ros_humble/config/cnc001.yaml
ros2 run cnc_manager manager_node
```

## Use
```bash
ros2 service call /cnc/unlock std_srvs/srv/Trigger                     # clear alarm ($X)
ros2 service call /cnc/home   std_srvs/srv/Trigger                     # home ($H)
ros2 topic pub --once /cnc/send_line std_msgs/msg/String "data: 'G0 X10 Y10'"
ros2 topic pub --once /cnc/send_file std_msgs/msg/String "data: '/abs/path/file.gcode'"
ros2 topic pub -r 10 /cnc/jog_cmd geometry_msgs/msg/Twist "{linear: {x: 0.005}}"  # jog +X 5 mm/s
ros2 topic echo /cnc/status          # state, position, busy, queue (JSON)
ros2 topic echo /machine_position    # x/y/z in metres (JointState)
```

## Parameters (`--ros-args -p name:=value`)
`machine_id` (cnc_001) · `poll_rate_hz` (2.0, 0 = off) · `max_jog_feed_mm_min` (1000) · `jog_period_s` (0.1)

## Limitations
- `/cnc/stop` only clears the queue and **does not stop the machine**. Use the E-stop. See `GRBL_ROS_PROPOSAL.md`.
- The file for `send_file` must exist on the computer running `grbl_node`.
