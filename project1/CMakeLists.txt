cmake_minimum_required(VERSION 2.8.3)
project(project1)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  dynamic_reconfigure
  tf2
  tf2_ros
)

add_message_files(
  FILES
  WheelSpeed.msg
)

add_service_files(
  FILES
  Reset.srv
  Calibrate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Build odometry and wheel_speed

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometry src/odometry.cpp)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(wheel_speed src/wheel_speed.cpp)
add_dependencies(wheel_speed ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(wheel_speed ${catkin_LIBRARIES})

add_executable(reset_odometry src/reset_odometry.cpp)
add_dependencies(reset_odometry ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(reset_odometry ${catkin_LIBRARIES})

add_executable(calibration src/calibration.cpp)
add_dependencies(calibration ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(calibration ${catkin_LIBRARIES})

add_executable(tf2_broad src/broadcaster_tf2.cpp)
add_dependencies(tf2_broad ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tf2_broad ${catkin_LIBRARIES})