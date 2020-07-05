#!/usr/bin/env python
"""Lift the box"""

import rospy
from walker_movement_utils import WalkerMovementUtils

import moveit_commander
import geometry_msgs.msg
import sys


intermediate_left  = [[0.072,  0.32, -0.278],[-0.280, 0.650, -0.280, 0.649]]
intermediate2_left  = [[0.250,  0.319, -0.263], [-0.281, 0.651, -0.279, 0.647]]
grapPose_left  = [[0.250, 0.186, -0.273], [-0.450, 0.551, -0.444, 0.545]]

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('lift_box_py')
scene = moveit_commander.PlanningSceneInterface(ns="/walker")
wmu = WalkerMovementUtils(disable_gait=True)

wmu.moveToJointDualArmSimmetrical([0,0,0,0,0,0,0])
wmu.releaseGrasp(True)
wmu.releaseGrasp(False)

objId = wmu.addCollisionBox(wmu.buildPoseStamped((0.2,0,-1),(0,0,0,1), "base_link"),(0.1,2,1.2))
if objId is None:
    exit(1)
rospy.loginfo("Moving to pose 1")
# wmu.moveToEeDualArmMirrored(intermediate_left[0],intermediate_left[1],do_cartesian=False)
# wmu.removeCollision(objId)
#
# rospy.loginfo("Moving to pose 2")

wmu.moveToEeDualArmMirrored(intermediate2_left[0],intermediate2_left[1],do_cartesian=False)

rospy.loginfo("Moving to grasp pose")

wmu.moveToEeDualArmMirrored(grapPose_left[0],grapPose_left[1],do_cartesian=False)


wmu.graspCan(True)
wmu.graspCan(False)

leftPose = wmu.getEePose(True)
rightPose = wmu.getEePose(False)

leftPoseL  = [[leftPose.pose.position.x,leftPose.pose.position.y,leftPose.pose.position.z+0.1],[leftPose.pose.orientation.x,leftPose.pose.orientation.y,leftPose.pose.orientation.z,leftPose.pose.orientation.w]]
rightPoseL = [[rightPose.pose.position.x,rightPose.pose.position.y,rightPose.pose.position.z+0.1],[rightPose.pose.orientation.x,rightPose.pose.orientation.y,rightPose.pose.orientation.z,rightPose.pose.orientation.w]]

rospy.loginfo("Moving up")
wmu.moveToEeDualArmMirrored(leftPoseL[0],leftPoseL[1],do_cartesian=True)
