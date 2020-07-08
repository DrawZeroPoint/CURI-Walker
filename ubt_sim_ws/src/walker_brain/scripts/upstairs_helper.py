#!/usr/bin/env python

from __future__ import print_function

import rospy
import math
import numpy as np
# Brings in the SimpleActionClient
import actionlib

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose2D
from walker_brain.srv import MoveToPose2D, MoveToPose2DResponse, Dummy, DummyResponse, MoveLeg, MoveLegResponse
from walker_movement.srv import GetJointState
from walker_movement.msg import DualArmJointMoveAction, DualArmJointMoveGoal


class UpstairsHelperServer(object):

    def __init__(self):
        super(UpstairsHelperServer, self).__init__()

        self._get_left_js = rospy.ServiceProxy('/walker/move_helper_left_leg/get_joint_state', GetJointState)
        self._get_right_js = rospy.ServiceProxy('/walker/move_helper_right_leg/get_joint_state', GetJointState)
        self._set_js_client = actionlib.SimpleActionClient('/walker/dual_leg_control/move_to_joint_pose', DualArmJointMoveAction)

        self._get_left_js.wait_for_service()
        self._get_right_js.wait_for_service()
        self._set_js_client.wait_for_server()

        self._leg_motion_server = rospy.Service('execute_leg_motion', MoveLeg, self.leg_motion_handle)
        self._stabilize_server = rospy.Service('stabilize_base', Dummy, self.stabilize_handle)
        self._balance_server = rospy.Service('execute_balance_base', Dummy, self.balance_handle)

        self._leg_puber = rospy.Publisher('/Leg/DesiredJoint', JointState, queue_size=1)

        self._last_js = np.zeros(12)

    def balance_handle(self, req):
        resp = DummyResponse()
        left_js_resp = self._get_left_js.call()
        right_js_resp = self._get_right_js.call()

        left_poses = list(left_js_resp.joint_poses)
        right_poses = list(right_js_resp.joint_poses)

        # hip_yaw
        left_poses[0] = 0
        right_poses[0] = 0

        # hip_roll
        left_poses[1] = 0
        right_poses[1] = 0

        # ankle_roll
        left_poses[-1] = 0
        right_poses[-1] = 0
        goal = DualArmJointMoveGoal()
        goal.left_pose = left_poses
        goal.right_pose = right_poses
        goal.mirror = False
        self._set_js_client.send_goal(goal)
        self._set_js_client.wait_for_result()
        res = self._set_js_client.get_result()
        if res.succeded:
            resp.result_status = resp.SUCCEEDED
        else:
            resp.result_status = resp.FAILED
        return resp

    @staticmethod
    def stabilize_handle(req):
        resp = DummyResponse()
        rospy.logwarn("Brain: Stabilizing the base for 1.5 sec.")
        rospy.sleep(1.5)
        resp.result_status = resp.SUCCEEDED
        return resp

    def leg_motion_handle(self, req):
        resp = MoveLegResponse()
        if len(req.joint_states) != 12:
            rospy.logerr("Brain: Leg motion joints number is wrong")
            resp.result_status = resp.FAILED
            return resp

        div = 50.
        offset = np.array(req.joint_states) - self._last_js
        delta = offset / div
        for i in range(50):
            msg = JointState()
            js = self._last_js + delta * (i + 1)
            msg.position.extend(js.tolist())
            self._leg_puber.publish(msg)
            resp.result_status = resp.SUCCEEDED
            rospy.sleep(0.001)

        self._last_js = req.joint_states
        return resp


if __name__ == "__main__":
    try:
        rospy.init_node('upstairs_helper')
        rospy.logwarn("Upstairs service ready.")
        mtp = UpstairsHelperServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
