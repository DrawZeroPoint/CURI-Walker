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

add_service_files(
  FILES
  Dummy.srv
  EstimateTargetPose.srv
  MoveToPose2D.srv
)

add_action_files(
  DIRECTORY action
  FILES DummyAction.action
)

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
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/lib/rosout_logger.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(grasp_cup_node src/grasp_cup_node.cpp)
add_dependencies(grasp_cup_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(grasp_cup_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(switch_light_node src/switch_light_node.cpp)
add_dependencies(switch_light_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(switch_light_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(open_fridge_node src/open_fridge_node.cpp)
add_dependencies(open_fridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_fridge_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(dummy_test_node src/dummy_test_node.cpp)
add_dependencies(dummy_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dummy_test_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


add_executable(dummy_server test/dummy_server.cpp)
add_dependencies(dummy_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dummy_server ${catkin_LIBRARIES})
