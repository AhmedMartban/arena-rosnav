cmake_minimum_required(VERSION 3.5)
project(rviz_plugins)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)

if(NOT Qt5Widgets_FOUND)
  message(FATAL_ERROR "Qt5Widgets not found")
endif()

# Specify additional locations of header files
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${rviz_default_plugins_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Define sources and headers
set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
  src/task_tool.cpp
)

# Create the library
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

# Link against dependencies using ament_target_dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_default_plugins
  Qt5Widgets
  std_srvs
  geometry_msgs
)

# Install targets and other files
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rviz_common rviz_default_plugins Qt5Widgets std_srvs geometry_msgs)

ament_package()
