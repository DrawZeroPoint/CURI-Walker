cmake_minimum_required(VERSION 2.8.3)
project(walker_movement)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  walker_moveit_config
  walker_webots_hardware_interface
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  actionlib_msgs
  rviz_visual_tools
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  tf2_kdl
  tf2_msgs
  tf2_py
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
 FILES
 GetJointState.srv
 GetEePose.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  MoveToEePose.action
  MoveToJointPose.action
  Grasp.action
  DualArmEeMove.action
  DualArmJointMove.action
  DualMirroredJointMove.action
  DualArmMirroredEeMove.action
  FollowEePoseTrajectory.action
  DualFollowEePoseTrajectory.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
    geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    walker_moveit_config
    walker_webots_hardware_interface
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    geometry_msgs
    rviz_visual_tools
    tf2
    tf2_ros
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(move_helper src/move_helper.cpp src/control_utils.cpp)
add_dependencies(move_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_helper ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(hand_helper src/hand_helper.cpp)
add_dependencies(hand_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hand_helper ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(dual_control src/dual_control.cpp src/control_utils.cpp)
add_dependencies(dual_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dual_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
