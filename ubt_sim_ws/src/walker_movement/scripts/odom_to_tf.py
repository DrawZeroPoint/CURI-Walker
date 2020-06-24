#!/usr/bin/env python
"""Receives the odom messages from the gait module and republishes the data as a tf frame."""

import rospy
import tf
from nav_msgs.msg import Odometry

firstCall = True
def odomCallback(msg):
    """Receive the odometry message and publish the tf."""
    global firstCall
    if firstCall:
        rospy.loginfo("Publishing tf for walker_odometry")
    firstCall = False
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    br = tf.TransformBroadcaster()
    br.sendTransform((position.x, position.y, position.z),
                     (orientation.x,orientation.y,orientation.z,orientation.z),
                     msg.header.stamp,
                     "walker_odometry",
                     "world")




rospy.init_node('odom_to_tf_py')

rospy.Subscriber("/Leg/walking_odom", Odometry, odomCallback)
rospy.spin()
