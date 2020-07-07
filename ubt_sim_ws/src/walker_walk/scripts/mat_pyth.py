#!/usr/bin/env python

from scipy.io import loadmat
import rospy
import actionlib
import walker_movement.msg
from geometry_msgs.msg import PoseStamped
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

rospy.init_node('matlab_to_python_leg_node', anonymous=True)


followEeTrajClient = actionlib.SimpleActionClient('/walker/move_helper_left_leg/follow_ee_pose_trajectory', walker_movement.msg.FollowEePoseTrajectoryAction)
moveEeDualClient = actionlib.SimpleActionClient('/walker/dual_leg_control/move_to_ee_pose', walker_movement.msg.DualArmEeMoveAction)
moveJointDualClient = actionlib.SimpleActionClient('/walker/dual_leg_control/move_to_joint_pose', walker_movement.msg.DualArmJointMoveAction)
dualFollowEeTrajClient = actionlib.SimpleActionClient('/walker/dual_leg_control/follow_ee_pose_trajectory', walker_movement.msg.DualFollowEePoseTrajectoryAction)

rospy.loginfo("Connecting to action servers")
followEeTrajClient.wait_for_server()
moveEeDualClient.wait_for_server()
moveJointDualClient.wait_for_server()
dualFollowEeTrajClient.wait_for_server()

rospy.loginfo("Action clients connected")



inputFile = rospkg.RosPack().get_path('walker_walk')+"/data/backwards_lowvel_lowheight.mat"
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

    for aux in foot_left_x:
        full_foot_left_x.append(aux)
    for aux in foot_left_y:
        full_foot_left_y.append(aux)
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
    for aux in foot_right_y:
        full_foot_right_y.append(aux)
    for aux in foot_right_z:
        full_foot_right_z.append(aux)





def rand_functio():
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
        poses_left.append(buildPoseStamped([x,y,z],[0,0,0.793,0.609],"center_of_mass"))

    for x,y,z in zip(full_foot_right_x,full_foot_right_y,full_foot_right_z):
        poses_right.append(buildPoseStamped([x,y,z],[0,0,0.609,0.793],"center_of_mass"))
        pass

    goal.times_from_start_left = duration
    goal.times_from_start_right  = duration
    goal.poses_left = poses_left #[buildPoseStamped([foot_left_x,foot_left_y,foot_left_z],[0,0,0.707,0.707],"center_of_mass")]
    goal.poses_right = poses_right #[buildPoseStamped([foot_right_x,foot_right_y,foot_right_z],[0,0,0.707,0.707],"center_of_mass")]
    print(goal)
    # exit(1)
    rospy.loginfo("times_from_start_left[0]="+str(goal.times_from_start_left[0]))
    rospy.loginfo("times_from_start_right[0]="+str(goal.times_from_start_right[0]))

    rospy.loginfo("Before goal")
    dualFollowEeTrajClient.send_goal(goal)
    dualFollowEeTrajClient.wait_for_result()
    if not dualFollowEeTrajClient.get_result().succeded:
        rospy.logerr("Failed to steps :(")
        print(dualFollowEeTrajClient.get_result())
        exit(1)
    rospy.loginfo("Step Completed")


rand_functio()
