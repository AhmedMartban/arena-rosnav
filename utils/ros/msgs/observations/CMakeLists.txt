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
find_package(message_generation REQUIRED)

# Uncomment the following lines if you have external dependencies
# find_package(SDL2 REQUIRED)
# find_package(OpenGL REQUIRED)
# find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  Observation.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

ament_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ./
  ${SDL2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
)

# adding observation_packer
add_executable(observation_packer src/observation_packer.cpp src/common/zVector2d.cpp src/common/f_math.cpp)
ament_target_dependencies(observation_packer rclcpp std_msgs sensor_msgs geometry_msgs)
add_dependencies(observation_packer observations_generate_messages_cpp)

install(TARGETS
  observation_packer
  DESTINATION lib/${PROJECT_NAME}
)

# Uncomment the following lines if you have scripts to install
# install(PROGRAMS
#   scripts/agent_play.py
#   DESTINATION lib/${PROJECT_NAME}
# )
