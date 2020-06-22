#!/usr/bin/env python


import sys
import argparse
import rospy
import actionlib
import walker_movement.msg
import time

rospy.init_node('grap_cup_py')

moveEeClient = actionlib.SimpleActionClient('/walker/move_helper_left_arm/move_to_ee_pose', walker_movement.msg.MoveToEePoseAction)
moveJointsClient = actionlib.SimpleActionClient('/walker/move_helper_left_arm/move_to_joint_pose', walker_movement.msg.MoveToJointPoseAction)
graspClient = actionlib.SimpleActionClient('/walker/hand_helper_left/grasp', walker_movement.msg.GraspAction)
rospy.loginfo("Waiting for action servers...")
moveEeClient.wait_for_server()
moveJointsClient.wait_for_server()
rospy.loginfo("Action servers connected")

rospy.loginfo("Moving to intermediate pose")
moveJointsClient.send_goal(walker_movement.msg.MoveToJointPoseGoal([-0.78046666105187, -0.9453471405387165, -1.9496421516412918, -1.1763506230509633, 1.6622610797676982, 0.25049953264133823, -0.45839707051223666]))
moveJointsClient.wait_for_result()
print(moveJointsClient.get_result())
rospy.loginfo("Moved")

#
# - Translation: [0.386, 0.440, -0.232]
# - Rotation: in Quaternion [-0.012, -0.010, 0.006, 1.000]
#             in RPY (radian) [-0.023, -0.020, 0.012]
#             in RPY (degree) [-1.334, -1.140, 0.665]
#
rospy.loginfo("Moving over can 1")
goal = walker_movement.msg.MoveToEePoseGoal()
goal.pose.header.frame_id = "base_link"
goal.pose.pose.position.x = 0.3855
goal.pose.pose.position.y = 0.4395
goal.pose.pose.position.z = -0.132
goal.pose.pose.orientation.x = -0.012
goal.pose.pose.orientation.y = -0.010
goal.pose.pose.orientation.z = 0.006
goal.pose.pose.orientation.w = 1
goal.end_effector_link = "left_tcp"
moveEeClient.send_goal(goal)
moveEeClient.wait_for_result()
print(moveEeClient.get_result())
rospy.loginfo("Moved")

time.sleep(1)

rospy.loginfo("Moving down")
goal = walker_movement.msg.MoveToEePoseGoal()
goal.pose.header.frame_id = "left_tcp"
goal.pose.pose.position.x = 0
goal.pose.pose.position.y = 0
goal.pose.pose.position.z = -0.1
goal.pose.pose.orientation.x = 0
goal.pose.pose.orientation.y = 0
goal.pose.pose.orientation.z = 0
goal.pose.pose.orientation.w = 1
goal.end_effector_link = "left_tcp"
moveEeClient.send_goal(goal)
moveEeClient.wait_for_result()
print(moveEeClient.get_result())
rospy.loginfo("Moved")

time.sleep(1)

rospy.loginfo("Grasping...")
graspClient.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_CAN))
graspClient.wait_for_result()
print(graspClient.get_result())
rospy.loginfo("Grasped")

rospy.loginfo("Moving up")
goal = walker_movement.msg.MoveToEePoseGoal()
goal.pose.header.frame_id = "left_tcp"
goal.pose.pose.position.z = 0.1
goal.end_effector_link = "left_tcp"
moveEeClient.send_goal(goal)
moveEeClient.wait_for_result()
print(moveEeClient.get_result())
rospy.loginfo("Moved")

time.sleep(10)

goal.pose.pose.position.z = -0.1
moveEeClient.send_goal(goal)
moveEeClient.wait_for_result()
print(moveEeClient.get_result())

time.sleep(1)

rospy.loginfo("Opening hand...")
graspClient.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_OPEN))
graspClient.wait_for_result()
print(graspClient.get_result())
rospy.loginfo("Opened")


time.sleep(1)

rospy.loginfo("Moving to intermediate pose")
moveJointsClient.send_goal(walker_movement.msg.MoveToJointPoseGoal([-0.78046666105187, -0.9453471405387165, -1.9496421516412918, -1.1763506230509633, 1.6622610797676982, 0.25049953264133823, -0.45839707051223666]))
moveJointsClient.wait_for_result()
print(moveJointsClient.get_result())
rospy.loginfo("Moved")

rospy.loginfo("Moving to home pose")
moveJointsClient.send_goal(walker_movement.msg.MoveToJointPoseGoal([0, 0, 0, 0, 0, 0, 0]))
moveJointsClient.wait_for_result()
print(moveJointsClient.get_result())
rospy.loginfo("Finished")
