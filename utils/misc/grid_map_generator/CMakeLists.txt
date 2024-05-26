cmake_minimum_required(VERSION 3.5)
project(grid_map_generator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(grid_map_generator_node src/grid_map_generator_node.cpp)
ament_target_dependencies(grid_map_generator_node
  rclcpp
  std_msgs
)

install(TARGETS
  grid_map_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
