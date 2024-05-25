cmake_minimum_required(VERSION 3.5)
project(plan_msgs)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_generation REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages
add_message_files(
  FILES
  RobotState.msg
  RobotStateStamped.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

# Export dependencies for ROS 2
ament_export_dependencies(ament_cmake geometry_msgs message_runtime nav_msgs sensor_msgs std_msgs)

# Add necessary dependencies for compilation
ament_package()
