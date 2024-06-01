cmake_minimum_required(VERSION 3.5)
project(observations)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Observation.msg"
)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  sensor_msgs geometry_msgs std_msgs
)

# Include generated messages
include_directories(${CMAKE_CURRENT_BINARY_DIR}/ ${PROJECT_NAME})

# Define dependencies
ament_export_dependencies(rosidl_default_runtime)

# Define executables
add_executable(observation_packer 
  observation_packer/observation_packer.cpp 
  common/zVector2d.cpp 
  common/f_math.cpp
)

# Add dependencies
ament_target_dependencies(observation_packer 
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs
)

# Add dependency on message generation
add_dependencies(observation_packer ${PROJECT_NAME}_generate_interfaces)


# Install targets
install(TARGETS
  observation_packer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
