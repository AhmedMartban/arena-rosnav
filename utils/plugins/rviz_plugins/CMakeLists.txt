cmake_minimum_required(VERSION 3.5)
project(rviz_plugins)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(Qt5Widgets REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(NOT Qt5Widgets_FOUND)
  message(FATAL_ERROR "Qt5Widgets not found")
endif()

# Specify additional locations of header files
include_directories(include)
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Define sources and headers
set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
  src/task_tool.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/moc_goal_tool.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/moc_task_tool.cpp
)

# Create the library
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

# Link against dependencies
target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${rviz_common_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
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
ament_export_dependencies(rclcpp rviz_common Qt5Widgets)

ament_package()
