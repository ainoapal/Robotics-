# Robotics-project1

## Build files
- catkin_make 

## Run the subscriber:
- rosrun project1 odometry

## Run the topics in bag X (from src folder, having put the bags in a folder called "bags"):
- rosbag play bags/bagX.bag
### See content of a topic:
- rostopic echo <topic>

## Launch project
- roslaunch project1 project1.launch

## Calibration
- rosrun project1 calibration <r> <N>
- <r>, <N> are respectively the wheel radius and the encoder resolution

## Reset odometry
- rosrun project1 reset_odometry <x> <y> <theta>
- <x>, <y>, <theta> are the three float parameters to be passed to set the new pose

## Dynamic reconfigure
- rosrun dynamic_reconfigure dynparam set odometry int_method [0,1]

## Useful tools
- rqt_graph
- rosrun plotjuggler plotjuggler
- rosrun rqt_tf_tree rqt_tf_tree
- rviz