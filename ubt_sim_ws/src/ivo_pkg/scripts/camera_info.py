import rospy
from sensor_msgs.msg import CameraInfo, Image
import numpy as np
# for head and chest camera

def rgb_callback(msg):
    # rospy.loginfo("rgb callback")
    msg.header.frame_id = "head_camera"
    republisher_rgb.publish(msg)

def depth_callback(msg):
    # rospy.loginfo("depth callback")
    msg.header.frame_id = "head_camera"
    republisher_depth.publish(msg)

camera_info_msg = CameraInfo()
camera_info_msg.width = 640
camera_info_msg.height = 480
camera_info_msg.K = list((521.1711084,0,320,0,547.7089685,240,0,0,1))
camera_info_msg.R = list((0,0,0,0,0,0,0,0,0))
camera_info_msg.P = list((521.1711084,0,320,0,0,547.7089685,240,0,0,0,1,0))

rospy.init_node("camera_info_publisher", anonymous=True)
publisher = rospy.Publisher("/walker/camera/camera_info", CameraInfo, queue_size=10)
republisher_rgb = rospy.Publisher("/walker/camera/headRGBFramed", Image, queue_size=10)
republisher_depth = rospy.Publisher("/walker/camera/headDepthFramed", Image, queue_size=10)
rate = rospy.Rate(30)
rospy.loginfo("Init")
rospy.loginfo(camera_info_msg)

sub_rgb = rospy.Subscriber("/walker/camera/headRGB",Image,rgb_callback,queue_size=10)
sub_depth = rospy.Subscriber("/walker/camera/headDepth",Image,depth_callback,queue_size=10)


while not rospy.is_shutdown():
    # rospy.loginfo("yey")
    camera_info_msg.header.stamp=rospy.Time.now()
    publisher.publish(camera_info_msg)
    rate.sleep()