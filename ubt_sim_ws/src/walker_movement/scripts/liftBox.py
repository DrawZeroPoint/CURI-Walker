#!/usr/bin/env python
"""Lift the box"""

import rospy
from walker_movement_utils import WalkerMovementUtils

import moveit_commander
import geometry_msgs.msg
import sys
import actionlib
import walker_movement.msg
from geometry_msgs.msg import PoseStamped
from scipy.io import loadmat
import numpy as np
import rospkg

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

intermediate_left  = [[0.072,  0.32, -0.278],[-0.280, 0.650, -0.280, 0.649]]
intermediate2_left  = [[0.250,  0.319, -0.263], [-0.281, 0.651, -0.279, 0.647]]
grapPose_left  = [[0.255, 0.186, -0.273], [-0.450, 0.551, -0.444, 0.545]]

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('lift_box_py')
scene = moveit_commander.PlanningSceneInterface(ns="/walker")
wmu = WalkerMovementUtils(disable_gait=True)

followEeTrajClient = actionlib.SimpleActionClient('/walker/move_helper_left_leg/follow_ee_pose_trajectory', walker_movement.msg.FollowEePoseTrajectoryAction)
moveEeDualClient = actionlib.SimpleActionClient('/walker/dual_leg_control/move_to_ee_pose', walker_movement.msg.DualArmEeMoveAction)
moveJointDualClient = actionlib.SimpleActionClient('/walker/dual_leg_control/move_to_joint_pose', walker_movement.msg.DualArmJointMoveAction)
dualFollowEeTrajClient = actionlib.SimpleActionClient('/walker/dual_leg_control/follow_ee_pose_trajectory', walker_movement.msg.DualFollowEePoseTrajectoryAction)

# wmu.moveToJointDualArmSimmetrical([0,0,0,0,0,0,0])
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
lift = 0.1
leftPoseL  = [[leftPose.pose.position.x,leftPose.pose.position.y,leftPose.pose.position.z+lift],[leftPose.pose.orientation.x,leftPose.pose.orientation.y,leftPose.pose.orientation.z,leftPose.pose.orientation.w]]
rightPoseL = [[rightPose.pose.position.x,rightPose.pose.position.y,rightPose.pose.position.z+lift],[rightPose.pose.orientation.x,rightPose.pose.orientation.y,rightPose.pose.orientation.z,rightPose.pose.orientation.w]]

rospy.loginfo("Moving up")
wmu.moveToEeDualArmMirrored(leftPoseL[0],leftPoseL[1],do_cartesian=True)

rospy.loginfo("Moved Up! Sleeping for 3 simulated seconds")
rospy.sleep(3)
rospy.loginfo("Starting backwards movement")



rospy.loginfo("Connecting to action servers")
followEeTrajClient.wait_for_server()
moveEeDualClient.wait_for_server()
moveJointDualClient.wait_for_server()
dualFollowEeTrajClient.wait_for_server()

rospy.loginfo("Action clients connected")



inputFile = rospkg.RosPack().get_path('walker_walk')+"/data/backwards_24steps_5cm_sloooow.mat"
x = loadmat(inputFile)["footinfos"]
rospy.loginfo("Loaded Matlab data from "+inputFile)
y = x[0]

full_time= []
full_foot_left_x = []
full_foot_left_y = []
full_foot_left_z = []
full_foot_right_x = []
full_foot_right_y = []
full_foot_right_z = []

time_dilat = 0
trig = True
for elem in y:
    time_vector =elem["timevec"][0][0][0][1:]

    decimate_val = 100
    time_vector = time_vector#[-1]
    for aux in time_vector:
        full_time.append(aux)
    time_dilat=time_dilat+1
    foot_left = elem["footleft"][0][0]
    foot_right = elem["footright"][0][0]

    foot_left_x     = foot_left[0][1:] #[-1]
    foot_left_x_vel = foot_left[1][1:] #[-1]
    foot_left_y     = foot_left[2][1:] #[-1]
    foot_left_y_vel = foot_left[3][1:] #[-1]
    foot_left_z     = foot_left[4][1:] #[-1]
    foot_left_z_vel = foot_left[5][1:] #[-1]

    dist_value = 0.10
    if trig:
        ivo_aux = np.arange(0,dist_value,(dist_value/len(foot_left_x)))
        trig = False
    else:
        ivo_aux = np.full(len(foot_left_x),dist_value)
    for aux in foot_left_x:
        full_foot_left_x.append(aux)
    for aux,altaux in zip(foot_left_y,ivo_aux):
        full_foot_left_y.append(aux+altaux)
        # print(aux+altaux)
    for aux in foot_left_z:
        full_foot_left_z.append(aux)
    foot_right_x     = foot_right[0][1:] #[-1]
    foot_right_x_vel = foot_right[1][1:] #[-1]
    foot_right_y     = foot_right[2][1:] #[-1]
    foot_right_y_vel = foot_right[3][1:] #[-1]
    foot_right_z     = foot_right[4][1:] #[-1]
    foot_right_z_vel = foot_right[5][1:] #[-1]

    for aux in foot_right_x:
        full_foot_right_x.append(aux)
    for aux,altaux in zip(foot_right_y,ivo_aux):
        full_foot_right_y.append(aux+altaux)
    for aux in foot_right_z:
        full_foot_right_z.append(aux)


def movement_function():
    goal = walker_movement.msg.DualFollowEePoseTrajectoryGoal()

    startTime = time_vector[0]
    duration = []
    prev_elem = full_time[0]
    # for time_elem in full_time[1:]:
        # if prev_elem >= time_elem:
        #     print(prev_elem,time_elem)
        # else:
        #     prev_elem = time_elem
        # duration.append(rospy.Duration.from_sec(time_elem - startTime))
        # duration.append(rospy.Duration.from_sec(time_elem))
    for time_elem in full_time:
        duration.append(rospy.Duration.from_sec(time_elem))

    poses_left = []
    poses_right = []
    for x,y,z in zip(full_foot_left_x,full_foot_left_y,full_foot_left_z):
        poses_left.append(buildPoseStamped([x,y,z],[0,0,0.707,0.707],"center_of_mass")) #[0,0,0.793,0.609]

    for x,y,z in zip(full_foot_right_x,full_foot_right_y,full_foot_right_z):
        poses_right.append(buildPoseStamped([x,y,z],[0,0,0.707,0.707],"center_of_mass")) #[0,0,0.609,0.793]
        pass

    goal.times_from_start_left = duration
    goal.times_from_start_right  = duration
    goal.poses_left = poses_left #[buildPoseStamped([foot_left_x,foot_left_y,foot_left_z],[0,0,0.707,0.707],"center_of_mass")]
    goal.poses_right = poses_right #[buildPoseStamped([foot_right_x,foot_right_y,foot_right_z],[0,0,0.707,0.707],"center_of_mass")]
    # print(goal)
    # exit(1)
    rospy.loginfo("times_from_start_left[0]="+str(goal.times_from_start_left[0]))
    rospy.loginfo("times_from_start_right[0]="+str(goal.times_from_start_right[0]))

    rospy.loginfo("Before goal")
    dualFollowEeTrajClient.send_goal(goal)
    rospy.loginfo("Sent Goal")
    dualFollowEeTrajClient.wait_for_result()
    if not dualFollowEeTrajClient.get_result().succeded:
        rospy.logerr("Failed to steps")
        print(dualFollowEeTrajClient.get_result())
        exit(1)
    rospy.loginfo("Movement Completed")


movement_function()