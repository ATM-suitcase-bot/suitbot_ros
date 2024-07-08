# SuitBot

Things to put in `.bashrc`:

## Step 1: Start Lidar and stream its data over ROS
```sh
alias atm_start_lidar='roslaunch livox_ros_driver livox_lidar.launch xfer_format:=2 publish_freq:=10.0'
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## Step 2: Choose one of the following commands:
```sh
# atm_auto_test: Autonomous control based on path (left, middle, right) selected based on user audio input

# atm_teleop_test: Manual control using joystick, with user audio disabled

# atm_debug_odometry: Debug odometry by user-specified moving time, linear velocity, and angular velocity. 
#      Example usage: atm_debug_odometry debug_time:=3.0 debug_linear:=0.3 debug_angular:=0.2  
#      In this example, 3.0 means move for 3s, 0.3 is in m/s, 0.2 is in rad/s. Positive angular velocity will cause the robot to turn right
alias atm_auto_test='roslaunch suitbot_ros job_test.launch use_rviz:=false manual_control:=false use_audio:=true'
alias atm_teleop_test='roslaunch suitbot_ros job_test.launch use_rviz:=false manual_control:=true use_audio:=false'
alias atm_debug_odometry='roslaunch suitbot_ros job_test.launch use_rviz:=false manual_control:=false debug_odometry:=true debug_time:=3.0 use_audio:=false'
```

## Step 3: After all the nodes are up, record bag file if needed. Topics: Encoder feedback velocity, velocity command sent to the robot, and time-sync Lidar data
```sh
alias atm_record_bag='rosbag record /suitbot/mobility/velocity /suitbot/ctrl/velocity /livox/lidar /suitbot/handle/force'
```