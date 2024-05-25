cmake_minimum_required(VERSION 3.5)
project(task_generator)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()



find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${rclpy_LIBRARIES})
set(LIBS ${ament_cmake_LIBRARIES} ${rclpy_LIBRARIES})

generate_dynamic_reconfigure_options(cfg/TaskGenerator.cfg)


catkin_install_python(PROGRAMS scripts/main.py scripts/scenario_helper.py
  scripts/server.py scripts/filewatcher.py scripts/delay_restart.py
  DESTINATION bin)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclpy)
include_directories(${INCLUDE_DIRS})

ament_package()
