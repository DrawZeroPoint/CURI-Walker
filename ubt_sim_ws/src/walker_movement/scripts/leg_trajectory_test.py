#!/usr/bin/env python3

import rospy
import actionlib
import walker_movement.msg
from geometry_msgs.msg import PoseStamped

def buildPoseStamped(position_xyz, orientation_xyzw, frame_id):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = position_xyz[0]
    pose.pose.position.y = position_xyz[1]
    pose.pose.position.z = position_xyz[2]
    pose.pose.orientation.x = orientation_xyzw[0]
    pose.pose.orientation.y = orientation_xyzw[1]
    pose.pose.orientation.z = orientation_xyzw[2]
    pose.pose.orientation.w = orientation_xyzw[3]
    return pose

rospy.init_node('leg_trajectory_test', anonymous=True)

followEeTrajClient = actionlib.SimpleActionClient('/walker/move_helper_left_leg/follow_ee_pose_trajectory', walker_movement.msg.FollowEePoseTrajectoryAction)
moveEeDualClient = actionlib.SimpleActionClient('/walker/dual_leg_control/move_to_ee_pose', walker_movement.msg.DualArmEeMoveAction)
moveJointDualClient = actionlib.SimpleActionClient('/walker/dual_leg_control/move_to_joint_pose', walker_movement.msg.DualArmJointMoveAction)

rospy.loginfo("Connecting to action servers")
followEeTrajClient.wait_for_server()
moveEeDualClient.wait_for_server()
moveJointDualClient.wait_for_server()
rospy.loginfo("Action clients connected")


goal = walker_movement.msg.DualArmJointMoveGoal()
goal.left_pose = [0,0,0,0,0,0]
goal.right_pose = [0,0,0,0,0,0]
goal.mirror = False
moveJointDualClient.send_goal(goal)
moveJointDualClient.wait_for_result()
if not moveJointDualClient.get_result().succeded:
    rospy.logerr("Failed to move to home pose")
    exit(1)


goal = walker_movement.msg.DualArmEeMoveGoal()
goal.left_pose = buildPoseStamped([0,0.0,0.10],[0,0,0,1],"left_foot_sole")
goal.left_end_effector_link = "left_foot_sole"
goal.right_pose = buildPoseStamped([0,0.0,0.10],[0,0,0,1],"right_foot_sole")
goal.right_end_effector_link = "right_foot_sole"
goal.do_cartesian = False
moveEeDualClient.send_goal(goal)
moveEeDualClient.wait_for_result()
if not moveEeDualClient.get_result().succeded:
    rospy.logerr("Failed to move down")
    exit(1)

rospy.sleep(0.5)

for i in range(12): #The whole movement plan fails because of collisions (moveit doesn't know both legs are moving)
    goal = walker_movement.msg.DualArmEeMoveGoal()
    goal.left_pose = buildPoseStamped([0,0.01,0.0],[0,0,0,1],"left_foot_sole")
    goal.left_end_effector_link = "left_foot_sole"
    goal.right_pose = buildPoseStamped([0,0.01,0.0],[0,0,0,1],"right_foot_sole")
    goal.right_end_effector_link = "right_foot_sole"
    goal.do_cartesian = False
    moveEeDualClient.send_goal(goal)
    moveEeDualClient.wait_for_result()
    if not moveEeDualClient.get_result().succeded:
        rospy.logerr("Failed to move sideways")
        exit(1)
    rospy.loginfo("Moved sideways "+str(i)+"cm")


goal = walker_movement.msg.FollowEePoseTrajectoryGoal()
goal.poses = [  buildPoseStamped([0.000, 0.000, 0.025],[0,0,0,1],"left_foot_sole"),
                buildPoseStamped([0.000, 0.000, 0.050],[0,0,0,1],"left_foot_sole"),

                buildPoseStamped([0.025, 0.000, 0.050],[0,0,0,1],"left_foot_sole"),
                buildPoseStamped([0.050, 0.000, 0.050],[0,0,0,1],"left_foot_sole"),

                buildPoseStamped([0.050, 0.000, 0.050],[0,0,0,1],"left_foot_sole"),
                buildPoseStamped([0.050, 0.000, 0.025],[0,0,0,1],"left_foot_sole"),
                buildPoseStamped([0.050, 0.000, 0.000],[0,0,0,1],"left_foot_sole")]
goal.times_from_start = [rospy.Duration.from_sec(i*0.1) for i in range(len(goal.poses))]

followEeTrajClient.send_goal(goal)
rospy.loginfo("Sent action request")
followEeTrajClient.wait_for_result()
if not moveEeDualClient.get_result().succeded:
    rospy.logerr("Failed to steps")
    exit(1)
