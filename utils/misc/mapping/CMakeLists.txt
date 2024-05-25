cmake_minimum_required(VERSION 3.5)
project(mapping)

# Add support for C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

## Find ament packages and other dependencies
find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(laser_geometry REQUIRED)

## Include directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(mapping src/mapping.cpp src/raycast.cpp)

## Link libraries to the library target
ament_target_dependencies(mapping
  rclcpp
  message_filters
  rclpy
  std_msgs
  pcl_conversions
  sensor_msgs
  visualization_msgs
  tf2
  pcl_ros
  Eigen3
  PCL
  laser_geometry
)

## Declare a C++ executable
add_executable(mapping_node src/mapping_node.cpp)

## Specify libraries to link the executable target against
target_link_libraries(mapping_node
  mapping
)

## Install targets
install(TARGETS
  mapping
  mapping_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Install include files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Export dependencies
ament_export_dependencies(
  ament_cmake
  message_filters
  rclpy
  std_msgs
  pcl_conversions
  sensor_msgs
  visualization_msgs
  tf2
  pcl_ros
  Eigen3
  PCL
  rclcpp
  laser_geometry
)

## Export include directories
include_directories(include)

## Export libraries
ament_export_libraries(mapping)

ament_package()
