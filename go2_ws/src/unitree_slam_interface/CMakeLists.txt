cmake_minimum_required(VERSION 3.8)
project(unitree_slam_interface)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message generation
set(msg_files
  "msg/QtCommand.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# Install standalone Python scripts
install(PROGRAMS
  src/slam_mapping_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package (if necessary)
# ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR unitree_slam_interface)
# Install standalone Python scripts



# Testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
