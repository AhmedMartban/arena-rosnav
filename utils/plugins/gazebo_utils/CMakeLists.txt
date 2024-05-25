cmake_minimum_required(VERSION 3.5)
project(rviz_plugins)

# Add support for C++14 or later
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Specify additional locations of header files
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${rviz_default_plugins_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Define the library
set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
  src/task_tool.cpp
  ${MOC_FILES}
)

# Wrap Qt meta-object compiler
qt5_wrap_cpp(MOC_FILES
  src/goal_tool.h
  src/task_tool.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${MOC_FILES})

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rviz_common"
  "rviz_default_plugins"
  "Qt5Core"
  "Qt5Widgets"
)

target_link_libraries(${PROJECT_NAME} ${Qt5_LIBRARIES})

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin description
install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media
)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)

ament_package()
