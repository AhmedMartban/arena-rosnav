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

# Add message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Observation.msg"
)

# Include generated messages
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

ament_package()

###########
## Build ##
###########

# adding observation_packer
add_executable(observation_packer src/observation_packer.cpp src/common/zVector2d.cpp src/common/f_math.cpp)
ament_target_dependencies(observation_packer rclcpp std_msgs sensor_msgs geometry_msgs)
add_dependencies(observation_packer ${PROJECT_NAME}_generate_interfaces)

install(TARGETS
  observation_packer
  DESTINATION lib/${PROJECT_NAME}
)
