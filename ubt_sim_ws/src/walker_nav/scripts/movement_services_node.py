import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import walker_nav.srv

def pose_to_goal_abs(pose):
    pose_to_goal(pose,"map")

def pose_to_goal_rel(pose):
    print(pose)
    pose_to_goal(pose,"base_link")

def pose_to_goal(pose,frame):

    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose.pose

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return return_str("Success")
        pass


if __name__ == '__main__':
    rospy.init_node('rolling_panda_base_base_services')
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    # move_to_abs_pos recives a pose and converts it to a goal according to the map frame
    s = rospy.Service('/move_base/move_to_abs_pos', walker_nav.srv.MoveToAbsPos, pose_to_goal_abs)
    rospy.loginfo("Started move_to_abs_pos.")

    # move_to_rel_pos recives a pose and converts it to a goal according to the base_link / the robot itself
    s = rospy.Service('/move_base/move_to_rel_pos', walker_nav.srv.MoveToRelPos, pose_to_goal_rel)
    rospy.loginfo("Started move_to_rel_pos.")
    rospy.spin()
