#!/usr/bin/env python
from __future__ import print_function

import rospy

import numpy as np
import ubt_core_msgs.msg as UBTMsg
import sensor_msgs.msg as SensorMsg
import control_msgs.msg as ControlMsg
import trajectory_msgs.msg as TrajectoryMsg

from dynamic_reconfigure.server import Server
from example.cfg import ConvertConfig


class GaitConverter(object):

    def __init__(self):
        super(GaitConverter, self).__init__()

        self.cfg_srv = Server(ConvertConfig, self.callback)
        self.lipm_suber = rospy.Subscriber('/control/joint_states', SensorMsg.JointState, self.handle)
        self.leg_puber = rospy.Publisher('/Leg/DesiredJoint', SensorMsg.JointState, queue_size=1)

        self.l_hip_y = None
        self.l_hip_r = None
        self.l_hip_p = None
        self.l_knee = None
        self.l_ank_p = None
        self.l_ank_r = None

        self.r_hip_y = None
        self.r_hip_r = None
        self.r_hip_p = None
        self.r_knee = None
        self.r_ank_p = None
        self.r_ank_r = None

        self.hip_y = 1.
        self.hip_r = 1.
        self.hip_p = 1.
        self.knee = 1.
        self.ank_p = 1.
        self.ank_r = 1.

    def callback(self, config, level):
        self.hip_y = config['hip_y']
        self.hip_r = config['hip_r']
        self.hip_p = config['hip_p']
        self.knee = config['knee']
        self.ank_p = config['ank_p']
        self.ank_r = config['ank_r']
        print(self.hip_y)
        return config

    def handle(self, msg):
        msg_to_send = SensorMsg.JointState()

        """
         0        1        2        3       4          5          6        7        8
        [R_HIP_P, R_HIP_R, R_HIP_Y, R_KNEE, R_ANKLE_R, R_ANKLE_P, L_HIP_P, L_HIP_R, L_HIP_Y,
         9       10         11         
         L_KNEE, L_ANKLE_R, L_ANKLE_P, WAIST_Y, WAIST_P, WAIST_R, NECK_Y, NECK_R, NECK_P,
         R_SHOULDER_P, R_SHOULDER_R, R_SHOULDER_Y, R_ELBOW_P, R_ELBOW_Y, R_WRIST_R, R_WRIST_Y,
         R_UTHUMB, R_LTHUMB, R_UINDEX, R_LINDEX, R_ULITTLE, R_LLITTLE, L_SHOULDER_P, L_SHOULDER_R,
         L_SHOULDER_Y, L_ELBOW_P, L_ELBOW_Y, L_WRIST_R, L_WRIST_Y, L_UTHUMB, L_LTHUMB, L_UINDEX,
         L_LINDEX, L_ULITTLE, L_LLITTLE]
        """
        if not self.l_hip_y:
            self.l_hip_y = msg.position[8]

        if not self.l_hip_r:
            self.l_hip_r = msg.position[7]

        if not self.l_hip_p:
            self.l_hip_p = msg.position[6]

        if not self.l_hip_y:
            self.l_hip_y = msg.position[8]

        if not self.l_hip_y:
            self.l_hip_y = msg.position[8]

        if not self.l_hip_y:
            self.l_hip_y = msg.position[8]

        if not self.l_hip_y:
            self.l_hip_y = msg.position[8]

        if not self.l_hip_y:
            self.l_hip_y = msg.position[8]

        if not self.l_hip_y:
            self.l_hip_y = msg.position[8]

        if not self.l_hip_y:
            self.l_hip_y = msg.position[8]

        if not self.l_hip_y:
            self.l_hip_y = msg.position[8]

        msg_to_send.position.append(msg.position[8] * self.hip_y)  # L_HIP_Y

        msg_to_send.position.append(msg.position[7] * self.hip_r)  # L_HIP_R
        msg_to_send.position.append(msg.position[6] * self.hip_p)  # L_HIP_P
        msg_to_send.position.append(msg.position[9] * self.knee)  # KNEE
        msg_to_send.position.append(msg.position[11] * self.ank_p)  # ANK_P
        msg_to_send.position.append(msg.position[10] * self.ank_r)  # ANK_R

        msg_to_send.position.append(msg.position[2] * self.hip_y)
        msg_to_send.position.append(msg.position[1] * self.hip_r)
        msg_to_send.position.append(msg.position[0] * self.hip_p)
        msg_to_send.position.append(msg.position[3] * self.knee)
        msg_to_send.position.append(msg.position[5] * self.ank_p)
        msg_to_send.position.append(msg.position[4] * self.ank_r)

        # msg_to_send.position.extend([0, 0, 0, 0, 0, 0.5, 0.5, 0, 0, 0, 0, 0])

        self.leg_puber.publish(msg_to_send)
        # rospy.sleep(0.001)


if __name__ == "__main__":
    try:
        rospy.init_node('converter')
        mtp = GaitConverter()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
