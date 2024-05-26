cmake_minimum_required(VERSION 3.5)
project(eval)

## Compile as C++14, supported in ROS 2 and newer
# add_compile_options(-std=c++14)

## Find ament macros and libraries
## if COMPONENTS list like find_package(ament_cmake REQUIRED COMPONENTS xyz)
## is used, also find other ament packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See https://docs.ros.org/en/foxy/How-To-Guides/Ament-Python-Setups.html
# ament_python_install_package(${PROJECT_NAME})

################################################
## Declare ROS messages, services, and actions ##
################################################

## To declare and build messages, services, or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "rosidl_default_generators"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "rosidl_default_runtime"
## * In this file (CMakeLists.txt):
##   * add "rosidl_default_generators" and every package in MSG_DEP_SET to
##     find_package(ament_cmake REQUIRED COMPONENTS ...)
##   * add "rosidl_default_runtime" and every package in MSG_DEP_SET to
##     ament_package()
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the rosidl_generate_interfaces entry below
##   * add every package in MSG_DEP_SET to rosidl_generate_interfaces(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# rosidl_generate_interfaces(${PROJECT_NAME}
#   FILES
#   Message1.msg
#   Service1.srv
#   Action1.action
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(ament_cmake REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## ament specific configuration ##
###################################
## The ament_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: ament_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

ament_package(
#  INCLUDE_DIRS include
#  LIBRARIES eval
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${ament_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/eval.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${ament_EXPORTED_TARGETS})

## Declare a C++ executable
## With ament all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/eval_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "ros2 run someones_pkg node" instead of "ros2 run someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${ament_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${ament_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use ament DESTINATION variables
# See https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  src/run_evals.py
  DESTINATION lib/${PROJECT_NAME}
)

## Mark executables for installation
## See https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html#installing-executables
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

## Mark libraries for installation
## See https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html#installing-libraries
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION share/${PROJECT_NAME}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# ament_add_gtest(${PROJECT_NAME}-test test/test_eval.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# ament_add_nosetests(test)
