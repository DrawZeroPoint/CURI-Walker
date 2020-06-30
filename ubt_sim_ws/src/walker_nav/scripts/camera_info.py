#! /usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo, Image , Imu
import numpy as np
# for head and chest camera
def imu_repub_callback(msg):
    msg.header.frame_id = "head_camera"
    republisher_imu.publish(msg)


def head_rgb_callback(msg):
    # rospy.loginfo("rgb callback")

    camera_info_msg = CameraInfo()
    camera_info_msg.width = 640
    camera_info_msg.height = 480
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = list((0,0,0,0,0))
    camera_info_msg.K = list((521.1711084,0,320,0,547.7089685,240,0,0,1))
    camera_info_msg.R = list((0,0,0,0,0,0,0,0,0))
    camera_info_msg.P = list((521.1711084,0,320,0,0,547.7089685,240,0,0,0,1,0))

    msg.header.frame_id = "head_camera"
    camera_info_msg.header.stamp=msg.header.stamp
    publisherRGB.publish(camera_info_msg)
    republisher_rgb.publish(msg)

def head_depth_callback(msg):
    # rospy.loginfo("depth callback")

    camera_info_msg = CameraInfo()
    camera_info_msg.width = 640
    camera_info_msg.height = 480
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = list((0,0,0,0,0))
    camera_info_msg.K = list((521.1711084,0,320,0,547.7089685,240,0,0,1))
    camera_info_msg.R = list((0,0,0,0,0,0,0,0,0))
    camera_info_msg.P = list((521.1711084,0,320,0,0,547.7089685,240,0,0,0,1,0))

    msg.header.frame_id = "head_camera"
    camera_info_msg.header.stamp=msg.header.stamp
    publisherDepth.publish(camera_info_msg)
    republisher_depth.publish(msg)

def bottom_rgb_callback(msg):
    # rospy.loginfo("rgb callback")

    camera_info_msg = CameraInfo()
    camera_info_msg.width = 640
    camera_info_msg.height = 480
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = list((0,0,0,0,0))
    camera_info_msg.K = list((521.1711084,0,320,0,547.7089685,240,0,0,1))
    camera_info_msg.R = list((0,0,0,0,0,0,0,0,0))
    camera_info_msg.P = list((521.1711084,0,320,0,0,547.7089685,240,0,0,0,1,0))

    msg.header.frame_id = "bottom_camera"
    camera_info_msg.header.stamp=msg.header.stamp
    publisherBottomRGB.publish(camera_info_msg)
    republisher_Bottom_rgb.publish(msg)

def bottom_depth_callback(msg):
    # rospy.loginfo("depth callback")

    camera_info_msg = CameraInfo()
    camera_info_msg.width = 640
    camera_info_msg.height = 480
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = list((0,0,0,0,0))
    camera_info_msg.K = list((521.1711084,0,320,0,547.7089685,240,0,0,1))
    camera_info_msg.R = list((0,0,0,0,0,0,0,0,0))
    camera_info_msg.P = list((521.1711084,0,320,0,0,547.7089685,240,0,0,0,1,0))

    msg.header.frame_id = "bottom_camera"
    camera_info_msg.header.stamp=msg.header.stamp
    publisherBottomDepth.publish(camera_info_msg)
    republisher_Bottom_depth.publish(msg)


def left_fish_rgb_callback(msg):
    # rospy.loginfo("rgb callback")

    camera_info_msg = CameraInfo()
    camera_info_msg.width = 640
    camera_info_msg.height = 480
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = list((0,0,0,0,0))
    camera_info_msg.K = list((288.1292942,0,320,0,330.3316609,240,0,0,1))
    camera_info_msg.R = list((0,0,0,0,0,0,0,0,0))
    camera_info_msg.P = list((288.1292942,0,320,0,0,330.3316609,240,0,0,0,1,0))

    msg.header.frame_id = "left_fisheye"
    camera_info_msg.header.stamp=msg.header.stamp
    publisher_fish_left.publish(camera_info_msg)
    republisher_fish_left.publish(msg)

def right_fish_rgb_callback(msg):
    # rospy.loginfo("rgb callback")

    camera_info_msg = CameraInfo()
    camera_info_msg.width = 640
    camera_info_msg.height = 480
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = list((0,0,0,0,0))
    camera_info_msg.K = list((288.1292942,0,320,0,330.3316609,240,0,0,1))
    camera_info_msg.R = list((0,0,0,0,0,0,0,0,0))
    camera_info_msg.P = list((288.1292942,0,320,0,0,330.3316609,240,0,0,0,1,0))

    msg.header.frame_id = "right_fisheye"
    camera_info_msg.header.stamp=msg.header.stamp
    publisher_fish_right.publish(camera_info_msg)
    republisher_fish_right.publish(msg)

rospy.init_node("camera_info_publisher", anonymous=True)

sub_imu = rospy.Subscriber("/sensor/head_imu",Imu,imu_repub_callback,queue_size=10)
republisher_imu = rospy.Publisher("/sensor/head_imu_framed",Imu,queue_size=10)

publisherRGB = rospy.Publisher("/walker/camera/rgb_framed/camera_info", CameraInfo, queue_size=10)
publisherDepth = rospy.Publisher("/walker/camera/depth_framed/camera_info", CameraInfo, queue_size=10)

republisher_rgb = rospy.Publisher("/walker/camera/rgb_framed/headRGB", Image, queue_size=10)
republisher_depth = rospy.Publisher("/walker/camera/depth_framed/headDepth", Image, queue_size=10)

publisherBottomRGB = rospy.Publisher("/walker/camera/bottom_rgbs_framed/camera_info", CameraInfo, queue_size=10)
publisherBottomDepth = rospy.Publisher("/walker/camera/bottom_depth_framed/camera_info", CameraInfo, queue_size=10)

republisher_Bottom_rgb = rospy.Publisher("/walker/camera/bottom_rgb_framed/bottomRGB", Image, queue_size=10)
republisher_Bottom_depth = rospy.Publisher("/walker/camera/bottom_depth_framed/bottomDepth", Image, queue_size=10)
rate = rospy.Rate(60)
rospy.loginfo("CameraInfo Init")

sub_rgb = rospy.Subscriber("/walker/camera/headRGB",Image,head_rgb_callback,queue_size=10)
sub_depth = rospy.Subscriber("/walker/camera/headDepth",Image,head_depth_callback,queue_size=10)

sub_rgb = rospy.Subscriber("/walker/camera/bottomRGB",Image,bottom_rgb_callback,queue_size=10)
sub_depth = rospy.Subscriber("/walker/camera/bottomDepth",Image,bottom_depth_callback,queue_size=10)


publisher_fish_left = rospy.Publisher("/walker/camera/fisheyes/left/camera_info", CameraInfo, queue_size=10)
publisher_fish_right = rospy.Publisher("/walker/camera/fisheyes/right/camera_info", CameraInfo, queue_size=10)
republisher_fish_left= rospy.Publisher("/walker/camera/fisheyes/left/image", Image, queue_size=10)
republisher_fish_right = rospy.Publisher("/walker/camera/fisheyes/right/image", Image, queue_size=10)
sub_rgb = rospy.Subscriber("/walker/camera/doubleLeftRGB",Image,left_fish_rgb_callback,queue_size=10)
sub_rgb = rospy.Subscriber("/walker/camera/doubleRightRGB",Image,right_fish_rgb_callback,queue_size=10)


while not rospy.is_shutdown():
    # rospy.loginfo("yey")
    rate.sleep()