#!/usr/bin/env python


import sys
import argparse
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

def check_joint_pose_match(pose1, pose2, tolerance):
    if type(pose1) is geometry_msgs.msg.PoseStamped:
        return check_joint_pose_match(pose1.pose, pose2.pose, tolerance)
    if type(pose1) is geometry_msgs.msg.Pose:
        return check_joint_pose_match(pose_to_list(pose1), pose_to_list(pose2), tolerance)

    if type(pose1) is list:
        for index in range(len(pose1)):
            if abs(pose2[index] - pose1[index]) > tolerance:
                return False
    return True

def main(move_group_name, joint_goal):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('got_to_joint_pose', anonymous=True)

    #robot = moveit_commander.RobotCommander()
    #scene = moveit_commander.PlanningSceneInterface()

    move_group = moveit_commander.MoveGroupCommander(move_group_name)

    #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    if len(joint_goal) != len(move_group.get_joints()):
        rospy.logerror("Specified joint pose does not have the right number of joints (should be "+str(len(move_group.get_joints()))+" but is "+str(len(joint_goal))+")")
        return

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    rospy.loginfo("Moving....")
    move_group.go(joint_goal, wait=True)
    rospy.loginfo("Done.")

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    rospy.loginfo("Stopped.")

    current_joints = move_group.get_current_joint_values()
    if not check_joint_pose_match(joint_goal, current_joints, 0.01):
        rospy.logerr("End pose out of goal tolerance.")
    else:
        rospy.loginfo("Moved successfully.")


ap = argparse.ArgumentParser()
ap.add_argument('-n','--name', type=str, help='The moveit move_group name', required=True)
ap.add_argument('-j','--joints', nargs='+', help='Specify the joint pose (e.g. -j 1 2 3 4 5 6 )', required=True)
args = vars(ap.parse_args())

joint_float_list = [float(js) for js in args["joints"]]

main(args["name"],joint_float_list)
