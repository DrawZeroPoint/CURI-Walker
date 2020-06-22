#!/usr/bin/env python


import sys
import argparse
import rospy
import actionlib
import walker_movement.msg
import time

poseOverCan1 = [0.3855, 0.4395, -0.132]
posesOverCans = []
for i in range(5):
    posesOverCans.append([poseOverCan1[0],poseOverCan1[1]-i*0.25,poseOverCan1[2]])

def moveToIntermediate():
    rospy.loginfo("Moving to intermediate pose")
    moveJointsClient.send_goal(walker_movement.msg.MoveToJointPoseGoal([-0.3383338087563895, -0.6975777197356626, -1.7908677458471025, -1.3, 1.2121745970333424, 0.05322237355283372, -0.44186775786234195]))
    #moveJointsClient.send_goal(walker_movement.msg.MoveToJointPoseGoal([-0.78046666105187, -0.9453471405387165, -1.9496421516412918, -1.1763506230509633, 1.6622610797676982, 0.25049953264133823, -0.45839707051223666]))
    moveJointsClient.wait_for_result()
    #print(moveJointsClient.get_result())
    rospy.loginfo("Moved")

def moveOverCan(can_number):
    rospy.loginfo("Moving over can "+str(can_number))
    goal = walker_movement.msg.MoveToEePoseGoal()
    goal.pose.header.frame_id = "base_link"
    goal.pose.pose.position.x = posesOverCans[can_number-1][0]
    goal.pose.pose.position.y = posesOverCans[can_number-1][1]
    goal.pose.pose.position.z = posesOverCans[can_number-1][2]
    goal.pose.pose.orientation.x = -0.012
    goal.pose.pose.orientation.y = -0.010
    goal.pose.pose.orientation.z = 0.006
    goal.pose.pose.orientation.w = 1
    goal.end_effector_link = "left_tcp"
    moveEeClient.send_goal(goal)
    moveEeClient.wait_for_result()
    #print(moveEeClient.get_result())
    rospy.loginfo("Moved")

def moveVertical(dz):
    rospy.loginfo("Moving vertically")
    goal = walker_movement.msg.MoveToEePoseGoal()
    goal.pose.header.frame_id = "left_tcp"
    goal.pose.pose.position.x = 0
    goal.pose.pose.position.y = 0
    goal.pose.pose.position.z = dz
    goal.pose.pose.orientation.x = 0
    goal.pose.pose.orientation.y = 0
    goal.pose.pose.orientation.z = 0
    goal.pose.pose.orientation.w = 1
    goal.end_effector_link = "left_tcp"
    moveEeClient.send_goal(goal)
    moveEeClient.wait_for_result()
    #print(moveEeClient.get_result())
    rospy.loginfo("Moved")

def graspCan():
    rospy.loginfo("Grasping...")
    graspClient.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_CAN))
    graspClient.wait_for_result()
    #print(graspClient.get_result())
    rospy.loginfo("Grasped")

def releaseCan():
    rospy.loginfo("Opening hand...")
    graspClient.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_OPEN))
    graspClient.wait_for_result()
    #print(graspClient.get_result())
    rospy.loginfo("Opened")

def moveToHome():
    rospy.loginfo("Moving to home pose")
    moveJointsClient.send_goal(walker_movement.msg.MoveToJointPoseGoal([0, 0, 0, 0, 0, 0, 0]))
    moveJointsClient.wait_for_result()
    #print(moveJointsClient.get_result())
    rospy.loginfo("Moved")



rospy.init_node('grap_cup_py')
requested_can = rospy.get_param("~can_number")

moveEeClient = actionlib.SimpleActionClient('/walker/move_helper_left_arm/move_to_ee_pose', walker_movement.msg.MoveToEePoseAction)
moveJointsClient = actionlib.SimpleActionClient('/walker/move_helper_left_arm/move_to_joint_pose', walker_movement.msg.MoveToJointPoseAction)
graspClient = actionlib.SimpleActionClient('/walker/hand_helper_left/grasp', walker_movement.msg.GraspAction)
rospy.loginfo("Waiting for action servers...")
moveEeClient.wait_for_server()
moveJointsClient.wait_for_server()
rospy.loginfo("Action servers connected")


moveToIntermediate()
moveOverCan(requested_can)
moveVertical(-0.1)
rospy.sleep(0.5)


graspCan()
rospy.sleep(0.25)

moveVertical(0.1)
for i in range(5):
    rospy.sleep(1)
    rospy.loginfo(str(i)+" seconds passed")
moveVertical(-0.1)
rospy.sleep(0.5)

releaseCan()
time.sleep(0.25)

moveToIntermediate()
moveToHome()

rospy.loginfo("Finished")
