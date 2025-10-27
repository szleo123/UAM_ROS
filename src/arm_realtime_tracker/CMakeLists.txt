cmake_minimum_required(VERSION 3.8)
project(arm_realtime_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)

add_executable(pose_tracker_node src/pose_tracker_node.cpp)
add_executable(pose_servo_tracker_node src/pose_servo_tracker_node.cpp)

ament_target_dependencies(pose_tracker_node
  rclcpp rclcpp_action rclcpp_components
  geometry_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
  moveit_ros_planning_interface
)
ament_target_dependencies(pose_servo_tracker_node
  rclcpp
  geometry_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs
)

install(TARGETS pose_tracker_node 
  pose_servo_tracker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
