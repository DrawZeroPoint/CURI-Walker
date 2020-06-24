#!/usr/bin/env python


import sys
import argparse
import rospy
import actionlib
import walker_movement.msg
import time
from walker_srvs.srv import leg_motion_MetaFuncCtrl
from geometry_msgs.msg import Twist, Vector3

# graspPositionLeft = [-0.170, -0.310, 0.390]
# graspOrientationLeft = [-0.295, -0.139, 0.560, 0.761]
graspPositionLeft = [0.410, 0.174, -0.263]
graspOrientationLeft = [0.258, 0.190, -0.514, 0.796]

# graspPositionRight = [-0.167, 0.297, 0.392]
# graspOrientationRight = [0.294, -0.136, -0.560, 0.763]
graspPositionRight = [0.410, -0.174, -0.263]
graspOrientationRight = [-0.258, 0.190, 0.514, 0.796]



def moveToEePose(isLeft, position_xyz, orientation_xyzw):
    rospy.loginfo("Moving to ee pose")
    if isLeft:
        c = moveEeLeftClient
        eel = "left_tcp"
    else:
        c = moveEeRightClient
        eel = "right_tcp"
    goal = walker_movement.msg.MoveToEePoseGoal()
    goal.pose.header.frame_id = "base_link"
    goal.pose.pose.position.x = position_xyz[0]
    goal.pose.pose.position.y = position_xyz[1]
    goal.pose.pose.position.z = position_xyz[2]
    goal.pose.pose.orientation.x = orientation_xyzw[0]
    goal.pose.pose.orientation.y = orientation_xyzw[1]
    goal.pose.pose.orientation.z = orientation_xyzw[2]
    goal.pose.pose.orientation.w = orientation_xyzw[3]
    goal.end_effector_link = eel
    c.send_goal(goal)
    c.wait_for_result()
    rospy.loginfo("Moved")


# def moveToJoint(isLeft, jointIfItWereLeft):
#     if isLeft:
#         c = moveJointsLeftClient
#         goal = jointIfItWereLeft
#     else:
#         c = moveJointsRightClient
#         t = [-1,1,-1,1,-1,-1,1]
#         g = [t[i]*jointIfItWereLeft[i] for i in range(len(jointIfItWereLeft))]
#         goal = g
#     c.send_goal(walker_movement.msg.MoveToJointPoseGoal(goal))
#     c.wait_for_result()
#     #print(c.get_result())



def graspHandle(isLeft):
    rospy.loginfo("Grasping...")
    if isLeft:
        c = graspLeftClient
    else:
        c = graspRightClient
    c.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_CART_HANDLE_CORNER))
    c.wait_for_result()
    #print(c.get_result())
    rospy.loginfo("Grasped")

def releaseGrasp(isLeft):
    rospy.loginfo("Opening hand...")
    if isLeft:
        c = graspLeftClient
    else:
        c = graspRightClient
    c.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_OPEN))
    c.wait_for_result()
    #print(c.get_result())
    rospy.loginfo("Opened")

def moveToHome(isLeft):
    rospy.loginfo("Moving to home pose")
    if isLeft:
        c = moveJointsLeftClient
    else:
        c = moveJointsRightClient
    c.send_goal(walker_movement.msg.MoveToJointPoseGoal([0, 0, 0, 0, 0, 0, 0]))
    c.wait_for_result()
    #print(c.get_result())
    rospy.loginfo("Moved")



rospy.init_node('push_cart_py')

moveEeLeftClient = actionlib.SimpleActionClient('/walker/move_helper_left_arm/move_to_ee_pose', walker_movement.msg.MoveToEePoseAction)
moveJointsLeftClient = actionlib.SimpleActionClient('/walker/move_helper_left_arm/move_to_joint_pose', walker_movement.msg.MoveToJointPoseAction)
graspLeftClient = actionlib.SimpleActionClient('/walker/hand_helper_left/grasp', walker_movement.msg.GraspAction)
moveEeRightClient = actionlib.SimpleActionClient('/walker/move_helper_right_arm/move_to_ee_pose', walker_movement.msg.MoveToEePoseAction)
moveJointsRightClient = actionlib.SimpleActionClient('/walker/move_helper_right_arm/move_to_joint_pose', walker_movement.msg.MoveToJointPoseAction)
graspRightClient = actionlib.SimpleActionClient('/walker/hand_helper_right/grasp', walker_movement.msg.GraspAction)


rospy.loginfo("Waiting for action servers...")
moveEeLeftClient.wait_for_server()
moveJointsLeftClient.wait_for_server()
graspLeftClient.wait_for_server()
moveEeRightClient.wait_for_server()
moveJointsRightClient.wait_for_server()
graspRightClient.wait_for_server()
rospy.loginfo("Action servers connected")


rospy.wait_for_service('/Leg/TaskScheduler')
legCommandService = rospy.ServiceProxy('/Leg/TaskScheduler', leg_motion_MetaFuncCtrl)
rospy.loginfo("Leg service connected")
nav_publisher = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=1)

#ensure hands are open
releaseGrasp(True)
releaseGrasp(False)

intermediatePosR = [graspPositionRight[0],graspPositionRight[1]-0.1,graspPositionRight[2]+0.1]
moveToEePose(False,intermediatePosR,graspOrientationRight)
moveToEePose(False,graspPositionRight,graspOrientationRight)
intermediatePosL = [graspPositionLeft[0],graspPositionLeft[1]+0.1,graspPositionLeft[2]+0.1]
moveToEePose(True,intermediatePosL,graspOrientationLeft)
moveToEePose(True,graspPositionLeft,graspOrientationLeft)
rospy.sleep(0.25)

graspHandle(True)
graspHandle(False)
rospy.sleep(0.25)


legCommandService("dynamic","","start")
rospy.sleep(0.5)
nav_publisher.publish(Twist(linear=Vector3(1,0,0),angular=Vector3(0,0,0)))
rospy.sleep(7)
nav_publisher.publish(Twist(linear=Vector3(0,0,0),angular=Vector3(0,0,0)))
legCommandService("dynamic","","stop")
rospy.sleep(0.5)

releaseGrasp(True)
releaseGrasp(False)
time.sleep(0.25)

moveToEePose(False,intermediatePosR,graspOrientationRight)
moveToEePose(True,intermediatePosL,graspOrientationLeft)
moveToHome(True)
moveToHome(False)

rospy.loginfo("Finished")
