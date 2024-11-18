cmake_minimum_required(VERSION 3.5)
project(go2_mapping)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)
find_package(ament_index_python REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

include_directories(
  include
  ${unitree_sdk2_INCLUDE_DIRS}
)

add_executable(dds_to_ros2_bridge src/dds_to_ros2_bridge.cpp)
ament_target_dependencies(dds_to_ros2_bridge
  rclcpp
  sensor_msgs
  unitree_sdk2
)

install(TARGETS
  dds_to_ros2_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
