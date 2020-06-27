#! /usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import walker_nav.srv
import std_msgs.msg
import tf

def return_str(msg=None):
    service_return = std_msgs.msg.String()
    service_return.data="{}".format(msg)
    return(service_return)

def pose_to_goal_abs(pose):
    pose_to_goal(pose,"map")
    resp = walker_nav.srv.MoveToAbsPosResponse()
    resp.response.data = "Success"
    return resp

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

def pos_saver(data):
    rate = rospy.Rate(10.0)
    name = data.name.data
    rospy.loginfo("Recieved:{}.".format(name))
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Listner lookup")
            trans,rot = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            #pose_name = str(raw_input("Name for this position:"))
            #full_line = "{},{} : {}\n".format(trans,rot,pose_name)
            #print(full_line)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x , goal.target_pose.pose.position.y , goal.target_pose.pose.position.z = trans
            goal.target_pose.pose.orientation.x , goal.target_pose.pose.orientation.y , goal.target_pose.pose.orientation.z , goal.target_pose.pose.orientation.w= rot
            print(goal)

            return return_str(str(goal))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

if __name__ == '__main__':
    rospy.init_node('rolling_panda_base_base_services')
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo("Listener lookup")
    listener = tf.TransformListener()

    # move_to_abs_pos recives a pose and converts it to a goal according to the map frame
    s = rospy.Service('/move_base/move_to_abs_pos', walker_nav.srv.MoveToAbsPos, pose_to_goal_abs)
    rospy.loginfo("Started move_to_abs_pos.")

    # move_to_rel_pos recives a pose and converts it to a goal according to the base_link / the robot itself
    s = rospy.Service('/move_base/move_to_rel_pos', walker_nav.srv.MoveToRelPos, pose_to_goal_rel)
    rospy.loginfo("Started move_to_rel_pos.")
    s = rospy.Service('/base/save_pos', walker_nav.srv.SavePos,pos_saver)
    rospy.loginfo("Started save_pos.")
    rospy.spin()
