cmake_minimum_required(VERSION 3.8)
project(movement)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include include/common include/nlohmann)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
  interfaces
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

add_executable(back_and_forth src/back_and_forth.cpp src/common/ros2_sport_client.cpp)
ament_target_dependencies(back_and_forth ${DEPENDENCY_LIST})

add_executable(move_to_location src/move_to_location.cpp src/common/ros2_sport_client.cpp)
ament_target_dependencies(move_to_location ${DEPENDENCY_LIST})

add_executable(path_planning src/path_planning.cpp)
ament_target_dependencies(path_planning ${DEPENDENCY_LIST})

add_executable(keyboard_control src/keyboard_control.cpp src/common/ros2_sport_client.cpp)
ament_target_dependencies(keyboard_control ${DEPENDENCY_LIST})

install(TARGETS
  back_and_forth
  move_to_location
  path_planning
  keyboard_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
