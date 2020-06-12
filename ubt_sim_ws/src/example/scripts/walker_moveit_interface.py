#!/usr/bin/env python

"""
Before running this:
1. roslaunch walker_moveit_config demo.launch
2. Start webots and the simulation
3. roslaunch leg_motion walker2_leg.launch account_file:=/home/dzp/CURI-Walker/user_account.json
4. set task 1: rosservice call /walker/sence "scene_name: 'SwitchLight' nav: false vision: false"

Start: roslaunch example task_1.launch

Trigger task 1: rostopic pub --once /task std_msgs/Int8 "data: 1"  1 for executing task 1, 0 for home
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

# Initialize the MoveIt interface for the robot via RoTools
left_arm_interface = moveit.WalkerMoveGroupInterface('walker_left_arm', 'left_arm_home')
right_arm_interface = moveit.WalkerMoveGroupInterface('walker_right_arm', 'right_arm_home')
head_interface = moveit.WalkerMoveGroupInterface('walker_head', 'head_home')

# Initialize walker calculation model
walker_left_arm_model = model.RobotModel.from_poe_parameters(predefined_models.walker_left_arm_poe())

# Initialize publisher to the controllers in webots_api
pub_left_arm = rospy.Publisher('/walker/leftLimb/controller', UBTMsg.JointCommand, queue_size=1)
pub_head = rospy.Publisher('walker/head/controller', UBTMsg.JointCommand, queue_size=1)

# Joint states derived from Walker's Webots simulator
wb_js_left_limb = SensorMsg.JointState()
wb_js_right_limb = SensorMsg.JointState()
wb_js_head = SensorMsg.JointState()


def wb_js_left_limb_cb(data):
    """Get instant joint state of the left limb from Webots API.

    :param data: sensor_msgs.JointState
    """
    global wb_js_left_limb
    wb_js_left_limb = data

    # The API gives no name to these joints, hence we add them according to
    # the MoveIt configuration
    wb_js_left_limb.name = left_arm_interface.get_active_joint_names()


def wb_js_head_cb(data):
    """Get instant joint state of the head from Webots API.

    :param data: sensor_msgs.JointState
    """
    global wb_js_head
    wb_js_head = data

    # The API gives no name to these joints, hence we add them according to
    # the MoveIt configuration
    wb_js_head.name = head_interface.get_active_joint_names()


def task_1_cb(data):
    """The task cb is triggered by an external topic,
    giving the moveit interface some time to start.

    :param data: std_msgs.Int trigger signal
    """
    # Initialize the start joint states in MoveIt with those from Webots
    # print('current left limb state', wb_js_left_limb)
    left_arm_interface.set_start_state(wb_js_left_limb)

    # the joint states in obtained with pre-planning
    prepare_js_deg = np.array([23., -65., -115., -109, -7, -47, -28])
    prepare_js_rad = prepare_js_deg / 180. * np.pi
    activate_js_deg = np.array([80., -56., -63., -69, 11, -42, -12])
    activate_js_rad = activate_js_deg / 180. * np.pi
    prepare_pose = walker_left_arm_model.fk_to_base(prepare_js_rad)
    activate_pose = walker_left_arm_model.fk_to_base(activate_js_rad)
    # print('prepare pose \n', np.array2string(prepare_pose, separator=', '), '\n')
    # print('activate pose \n', np.array2string(activate_pose, separator=', '), '\n')
    prepare_ros_pose = common.to_ros_pose(prepare_pose)
    activate_ros_pose = common.to_ros_pose(activate_pose)

    if data.data == 0:
        # reset to home pose
        left_arm_interface.go_home()
    else:
        # switch the light on
        left_arm_interface.go_through_poses([prepare_ros_pose, activate_ros_pose])


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

    sub_curr_js = rospy.Subscriber('/walker/leftLimb/joint_states',
                                   SensorMsg.JointState, wb_js_left_limb_cb)
    sub_task_1 = rospy.Subscriber('task_1', StdMsg.Int8, task_1_cb)
    sub_moveit_js = rospy.Subscriber('joint_states', SensorMsg.JointState, moveit_js_cb)

    print("Task 1 interface ready.\n")
    rospy.spin()


if __name__ == "__main__":
    try:
        walker_moveit_interface()
    except rospy.ROSInterruptException:
        pass
