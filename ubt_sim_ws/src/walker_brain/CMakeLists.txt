cmake_minimum_required(VERSION 3.5.1)
project(walker_brain)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  behaviortree_cpp_v3
  geometry_msgs
  dynamic_reconfigure
  roscpp
  sensor_msgs
  std_msgs
)
find_package(GTest)

add_message_files(
  FILES
  BehaviorTree.msg
  NodeParameter.msg
  NodeStatus.msg
  StatusChange.msg
  StatusChangeLog.msg
  TreeNode.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(grasp_cup_node src/grasp_cup_node.cpp)
add_dependencies(grasp_cup_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(grasp_cup_node ${catkin_LIBRARIES} )

