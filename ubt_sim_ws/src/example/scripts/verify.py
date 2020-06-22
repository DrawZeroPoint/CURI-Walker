#!/usr/bin/env python
from __future__ import print_function

import rospy

import numpy as np
import ubt_core_msgs.msg as UBTMsg
import control_msgs.msg as ControlMsg
import trajectory_msgs.msg as TrajectoryMsg


pub_left_arm = rospy.Publisher('/walker/leftLimb/controller', UBTMsg.JointCommand, queue_size=10)


def verify():
    rospy.init_node('verify', anonymous=True)
    print("Ready to verify.")

    left_arm_cmd = UBTMsg.JointCommand()
    left_arm_cmd.mode = 5
    joint_states_deg = np.array([77., -41., -61., -54, 40, -20, 1])
    joint_states_rad = joint_states_deg / 180. * np.pi
    left_arm_cmd.command = list(joint_states_rad)
    left_arm_cmd.names.extend(['LShoulderPitch', 'LShoulderRoll', 'LShoulderYaw', 'LElbowRoll',
                               'LElbowYaw', 'LWristPitch', 'LWristRoll'])
    print(left_arm_cmd)
    while True:
        pub_left_arm.publish(left_arm_cmd)
        rospy.sleep(rospy.Duration.from_sec(0.001))


if __name__ == "__main__":
    try:
        verify()
    except rospy.ROSInterruptException:
        print('error')
        pass
