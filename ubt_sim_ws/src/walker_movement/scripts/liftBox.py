#!/usr/bin/env python
"""Lift the box"""

import rospy
from walker_movement_utils import WalkerMovementUtils
import time


#intermediate_left = [1.0278720178804883, -1.3548031962129417, 0.7903397757571958, -2.1011504197978526, -0.27940800022942835, -0.08638610618302137, -0.08877344509505528]
#intermediate2_left = [1.5590584678108292, -0.8524430544075843, 1.041603057611335, -1.5654486502790619, -0.7991347163547041, -0.35222487010045445, 0.2374708386083253]
#graspPose_left = [0.9852922323727588, -0.7316110152627334, 0.3189245030450703, -1.5297904841656615, -0.7319055829572017, -0.3665281814108243, 0.32694086780620724]

intermediate_left  = [[0.072,  0.312, -0.278],[-0.280, 0.650, -0.280, 0.649]]
intermediate_right = [[0.072, -0.312, -0.278],[ 0.280, 0.650,  0.280, 0.649]]

intermediate2_left  = [[0.250,  0.319, -0.263], [-0.281, 0.651, -0.279, 0.647]]
intermediate2_right = [[0.250, -0.319, -0.263], [ 0.281, 0.651,  0.279, 0.647]]

grapPose_left  = [[0.242, 0.186, -0.273], [-0.450, 0.551, -0.444, 0.545]]
#grapPose_right = [[0.242, -0.186, -0.273],[ 0.450, 0.551,  0.444, 0.545]]
grapPose_right = [[0.266, -0.171, -0.242],[0.405, 0.586, 0.396, 0.579]]

rospy.init_node('lift_box_py')

wmu = WalkerMovementUtils(disable_gait=True)

wmu.moveToJointDualArmSimmetrical([0,0,0,0,0,0,0])
wmu.releaseGrasp(True)
wmu.releaseGrasp(False)

rospy.loginfo("Moving to pose 1")

wmu.moveToEeDualArm(intermediate_left[0],intermediate_left[1],intermediate_right[0],intermediate_right[1],do_cartesian=True)

rospy.loginfo("Moving to pose 2")

wmu.moveToEeDualArm(intermediate2_left[0],intermediate2_left[1],intermediate2_right[0],intermediate2_right[1],do_cartesian=False)

rospy.loginfo("Moving to grasp pose")

wmu.moveToEeDualArm(grapPose_left[0],grapPose_left[1],grapPose_right[0],grapPose_right[1],do_cartesian=False)


wmu.graspCan(True)
wmu.graspCan(False)

leftPose = wmu.getEePose(True)
rightPose = wmu.getEePose(False)

leftPoseL  = [[leftPose.pose.position.x,leftPose.pose.position.y,leftPose.pose.position.z+0.05],[leftPose.pose.orientation.x,leftPose.pose.orientation.y,leftPose.pose.orientation.z,leftPose.pose.orientation.w]]
rightPoseL = [[rightPose.pose.position.x,rightPose.pose.position.y,rightPose.pose.position.z+0.05],[rightPose.pose.orientation.x,rightPose.pose.orientation.y,rightPose.pose.orientation.z,rightPose.pose.orientation.w]]

rospy.loginfo("Moving up")
wmu.moveToEeDualArm(leftPoseL[0],leftPoseL[1],rightPoseL[0],rightPoseL[1],do_cartesian=True)
