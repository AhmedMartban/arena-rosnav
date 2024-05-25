cmake_minimum_required(VERSION 3.5)
project(plan_msgs)

# Find dependencies in ROS 2
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add message files
set(msg_files
  "msg/RobotState.msg"
  "msg/RobotStateStamped.msg"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
)

# Install package
ament_package()
