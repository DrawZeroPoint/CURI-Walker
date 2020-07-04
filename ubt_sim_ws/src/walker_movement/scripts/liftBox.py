#!/usr/bin/env python
"""Lift the box"""

import rospy
from walker_movement_utils import WalkerMovementUtils

import moveit_commander
import geometry_msgs.msg
import sys

def addCollision():
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.20
    box_pose.pose.position.z = -1
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 2, 1.2))

    start = rospy.get_time()
    while (rospy.get_time() - start < 10) and not rospy.is_shutdown():
        if box_name in scene.get_known_object_names():
            rospy.loginfo("Added CollisionObject")
            return True #added
        rospy.sleep(0.1)

    rospy.loginfo("Failed to add CollisionObject")
    return False #failed


def removeCollision():
    scene.remove_world_object("box")


intermediate_left  = [[0.072,  0.32, -0.278],[-0.280, 0.650, -0.280, 0.649]]
intermediate2_left  = [[0.250,  0.319, -0.263], [-0.281, 0.651, -0.279, 0.647]]
grapPose_left  = [[0.242, 0.186, -0.273], [-0.450, 0.551, -0.444, 0.545]]

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('lift_box_py')
scene = moveit_commander.PlanningSceneInterface(ns="/walker")
wmu = WalkerMovementUtils(disable_gait=True)

wmu.moveToJointDualArmSimmetrical([0,0,0,0,0,0,0])
wmu.releaseGrasp(True)
wmu.releaseGrasp(False)

addCollision()
rospy.loginfo("Moving to pose 1")
wmu.moveToEeDualArmMirrored(intermediate_left[0],intermediate_left[1],do_cartesian=False)
removeCollision()

rospy.loginfo("Moving to pose 2")

wmu.moveToEeDualArmMirrored(intermediate2_left[0],intermediate2_left[1],do_cartesian=False)

rospy.loginfo("Moving to grasp pose")

wmu.moveToEeDualArmMirrored(grapPose_left[0],grapPose_left[1],do_cartesian=False)


wmu.graspCan(True)
wmu.graspCan(False)

leftPose = wmu.getEePose(True)
rightPose = wmu.getEePose(False)

leftPoseL  = [[leftPose.pose.position.x,leftPose.pose.position.y,leftPose.pose.position.z+0.05],[leftPose.pose.orientation.x,leftPose.pose.orientation.y,leftPose.pose.orientation.z,leftPose.pose.orientation.w]]
rightPoseL = [[rightPose.pose.position.x,rightPose.pose.position.y,rightPose.pose.position.z+0.05],[rightPose.pose.orientation.x,rightPose.pose.orientation.y,rightPose.pose.orientation.z,rightPose.pose.orientation.w]]

rospy.loginfo("Moving up")
wmu.moveToEeDualArmMirrored(leftPoseL[0],leftPoseL[1],do_cartesian=True)
