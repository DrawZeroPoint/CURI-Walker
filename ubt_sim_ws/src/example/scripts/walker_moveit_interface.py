#!/usr/bin/env python

"""
Before running this:
1. Launch the MoveIt demo
2. Start webots and the simulation
"""

from __future__ import print_function

import rospy

import numpy as np
import geometry_msgs.msg as GeometryMsg
import std_msgs.msg as StdMsg
import sensor_msgs.msg as SensorMsg
import ubt_core_msgs.msg as UBTMsg
import control_msgs.msg as ControlMsg
import trajectory_msgs.msg as TrajectoryMsg

# make sure setting > export PYTHONPATH="${PYTHONPATH}:~/rotools" in bashrc
from rotools.robot.serial import model
from rotools.robot.serial import predefined_models
from rotools.interface import moveit
from rotools.policy.rmp_flow import planner
from rotools.utility import common
from rotools.utility import transform


# Initialize the RoTools for the robot
interface = moveit.WalkerMoveGroupInterface('walker_left_arm', 'left_arm_home')
pub_left_arm = rospy.Publisher('/walker/leftLimb/controller', UBTMsg.JointCommand, queue_size=1)

# Webots joint states for the left limb
wb_js_left_limb = SensorMsg.JointState()


def wb_js_left_limb_cb(data):
    """Get joint state of the left limb from Webots API

    """
    global wb_js_left_limb
    wb_js_left_limb = data

    # The API gives no name to these joints, hence we add them according to
    # MoveIt configuration
    wb_js_left_limb.name = interface.get_active_joint_names()


def task_cb(data):
    """The task cb give the moveit interface some time to start.

    """
    # Initialize the joint states of the arms
    print('current left limb state', wb_js_left_limb)
    interface.set_start_state(wb_js_left_limb)

    prepare_js_deg = np.array([139., -90., 24., -102, -19, -19, -19])
    prepare_js_rad = prepare_js_deg / 180. * np.pi
    activate_js_deg = np.array([132., -66., 26., -70, -25, -13, -8])
    activate_js_rad = activate_js_deg / 180. * np.pi
    if data.data == 0:
        # reset to home pose
        interface.go_home()
    elif data.data == 1:
        # go to prepare pose
        interface.go_to_joint_state(prepare_js_rad)
    elif data.data == 2:
        interface.go_to_joint_state(activate_js_rad)


def moveit_js_cb(data):
    """Convert moveit planned joint states to ubt control msg

    """
    joint_names = data.name
    joint_pos = data.position
    name_to_pub = []
    pos_to_pub = []
    for j, jn in enumerate(joint_names):
        if 'left_limb_j' in jn:
            name_to_pub.append(jn)
            pos_to_pub.append(joint_pos[j])

    control_msg = UBTMsg.JointCommand()
    control_msg.mode = 5
    control_msg.names = name_to_pub
    control_msg.command = pos_to_pub
    pub_left_arm.publish(control_msg)


def walker_moveit_interface():
    rospy.init_node('walker_moveit_interface', anonymous=True)

    sub_curr_js_left_limb = rospy.Subscriber('/walker/leftLimb/joint_states',
                                             SensorMsg.JointState, wb_js_left_limb_cb)
    sub_task_trigger = rospy.Subscriber('task', StdMsg.Int8, task_cb)
    sub_moveit_js = rospy.Subscriber('joint_states', SensorMsg.JointState, moveit_js_cb)

    # on call for executing task
    # pose_goal = GeometryMsg.Pose()
    # pose_goal.position.x = 0.3
    # pose_goal.position.y = -0.3
    # pose_goal.position.z = 0.1
    # pose_goal.orientation.x = 1
    # pose_goal.orientation.y = 0
    # pose_goal.orientation.z = 0
    # pose_goal.orientation.w = 0
    # interface.go_to_pose_goal(pose_goal)

    print("MoveIt interface ready.\n")
    rospy.spin()


if __name__ == "__main__":
    try:
        walker_moveit_interface()
    except rospy.ROSInterruptException:
        pass
