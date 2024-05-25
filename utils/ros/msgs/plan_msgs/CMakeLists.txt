cmake_minimum_required(VERSION 3.5)
project(plan_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Generate messages
add_message_files(
  FILES
  RobotState.msg
  RobotStateStamped.msg
)

# Generate services
add_service_files(
  FILES
  Subgoal.srv
  MakeGlobalPlan.srv
)

# Generate actions (if any)
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  std_msgs
  nav_msgs 
  sensor_msgs 
)

# Declare the package
ament_package()

# Specify additional locations of header files
include_directories(
  ${ament_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Mark executables and/or libraries for installation
install(DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}
)

# Add cmake target dependencies of the executable/library
ament_export_dependencies(rosidl_default_runtime)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
