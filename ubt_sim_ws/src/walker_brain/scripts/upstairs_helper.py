#!/usr/bin/env python

from __future__ import print_function

import rospy
import math

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose2D
from walker_brain.srv import MoveToPose2D, MoveToPose2DResponse, Dummy, DummyResponse, MoveLeg, MoveLegResponse
from walker_srvs.srv import leg_motion_MetaFuncCtrl, leg_motion_MetaFuncCtrlRequest


class MoveToPoseServer(object):

    def __init__(self):
        super(MoveToPoseServer, self).__init__()

        self._leg_motion_server = rospy.Service('execute_leg_motion', MoveLeg, self.leg_motion_handle)
        self._stabilize_server = rospy.Service('stabilize_base', Dummy, self.stabilize_handle)

        self._leg_puber = rospy.Publisher('/Leg/DesiredJoint', JointState, queue_size=1)

    @staticmethod
    def stabilize_handle(req):
        resp = DummyResponse()
        rospy.logwarn("Brain: Stabilizing the base for 2 sec.")
        rospy.sleep(2)
        resp.result_status = resp.SUCCEEDED
        return resp

    def leg_motion_handle(self, req):
        resp = MoveLegResponse()
        msg = JointState()
        if len(req.joint_states) != 12:
            rospy.logerr("Brain: Leg motion joints number is wrong")
            resp.result_status = resp.FAILED
            return resp

        msg.position.extend(req.joint_states)
        self._leg_puber.publish(msg)
        resp.result_status = resp.SUCCEEDED
        return resp


if __name__ == "__main__":
    try:
        rospy.init_node('upstairs_helper')
        rospy.logwarn("Upstairs service ready.")
        mtp = MoveToPoseServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
