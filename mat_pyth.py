from scipy.io import loadmat
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


x = loadmat('outie.mat')["footinfos"]
y = x[0]
for elem in y:
    time_vector =elem["timevec"][0][0][0]

    decimate_val = 100
    time_vector = time_vector[-1]

    foot_left = elem["footleft"][0][0]
    foot_right = elem["footright"][0][0]

    foot_left_x     = foot_left[0][-1] 
    foot_left_x_vel = foot_left[1][-1]
    foot_left_y     = foot_left[2][-1]
    foot_left_y_vel = foot_left[3][-1]
    foot_left_z     = foot_left[4][-1]
    foot_left_z_vel = foot_left[5][-1]

    foot_right_x     = foot_right[0][-1]
    foot_right_x_vel = foot_right[1][-1]
    foot_right_y     = foot_right[2][-1]
    foot_right_y_vel = foot_right[3][-1]
    foot_right_z     = foot_right[4][-1]
    foot_right_z_vel = foot_right[5][-1]

    goal = walker_movement.msg.FollowEePoseTrajectoryGoal()

    duration = []
    # for time_elem in time_vector:
    #     duration.append(rospy.Duration.from_sec(time_elem))
    duration.append(rospy.Duration.from_sec(time_vector))

    poses_left = []
    poses_right = []
    # for x,y,z in zip(foot_left_x,foot_left_y,foot_left_z):
    #     poses_left.append(buildPoseStamped([x,y,z],[0,0,0,1],"center_of_mass"))
    
    # for x,y,z in zip(foot_right_x,foot_right_y,foot_right_z):
    #     poses_right.append(buildPoseStamped([x,y,z],[0,0,0,1],"center_of_mass"))
    #     pass

    goal.times_from_start = duration
    # goal.times_from_start  = duration
    # goal.poses = [buildPoseStamped([foot_left_x,foot_left_y,foot_left_z],[0,0,0,1],"center_of_mass")]
    goal.poses = [buildPoseStamped([foot_right_x,foot_right_y,foot_right_z],[0,0,0,1],"center_of_mass")]
    print(goal)
    followEeTrajClient.send_goal(goal)
    followEeTrajClient.wait_for_result()

    if not followEeTrajClient.get_result().succeded:
        rospy.logerr("Failed to steps :(")
        print(followEeTrajClient.get_result())
        exit(1)
    exit(1)

