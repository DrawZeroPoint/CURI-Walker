#!/usr/bin/env python

from __future__ import print_function

import rospy
import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose2D
from walker_brain.srv import MoveToPose2D, MoveToPose2DResponse, Dummy, DummyResponse
from walker_srvs.srv import leg_motion_MetaFuncCtrl, leg_motion_MetaFuncCtrlRequest


class MoveToPoseServer(object):

    def __init__(self):
        super(MoveToPoseServer, self).__init__()

        self._server = rospy.Service('execute_move_base', MoveToPose2D, self.handle)
        # self._adjust_yaw_server = rospy.Service('execute_rotate_base', MoveToPose2D, self.adjust_yaw_handle)
        self._walk_cmd_server = rospy.Service('execute_walk_cmd', Dummy, self.cmd_handle)
        self._stabilize_server = rospy.Service('stabilize_base', Dummy, self.stabilize_handle)

        self._remote = rospy.ServiceProxy('/Leg/TaskScheduler', leg_motion_MetaFuncCtrl)
        self._ls = rospy.Subscriber('/Leg/leg_status', String, self.status_cb)
        self._vel_puber = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=1)

        self._x_primary_vel = 0.25
        self._y_primary_vel = 0.04
        self._r_primary_vel = 0.25
        self._x_min_step = 0.005
        self._y_min_step = 0.005
        self._r_min_step = 0.005

        self._status = ""

    def status_cb(self, msg):
        self._status = msg.data

    def on_start(self):
        clear_vel = Twist()
        self._vel_puber.publish(clear_vel)
        while True:
            self.call("start")
            rospy.sleep(0.35)
            if self._status == 'dynamic':
                break
            else:
                rospy.logwarn("Brain: Waiting for state change..")

    def on_stop(self):
        vel = Twist()
        self._vel_puber.publish(vel)
        self.call("stop")
        rospy.sleep(0.7)

    def move_forward_backward(self, offset, abs_vel, is_negative):
        assert abs_vel > 0
        vel = Twist()
        if not is_negative:
            info = 'Move forward'
            vel.linear.x = abs_vel
        else:
            info = 'Move backward'
            vel.linear.x = -abs_vel
        self._vel_puber.publish(vel)
        step_num = math.floor(offset / abs_vel)
        rospy.loginfo("Brain: {} for {} steps ({} m/step)".format(info, int(step_num), abs_vel))
        # The compensate time is obtained by trial and error
        if is_negative:
            rospy.sleep(step_num * 0.7 + 0.2)
        else:
            rospy.sleep(step_num * 0.7 + 0.08)
        residual = offset - step_num * abs_vel
        return residual

    def move_left_right(self, offset, abs_vel, is_negative):
        assert abs_vel > 0
        vel = Twist()
        if not is_negative:
            info = 'Move left'
            vel.linear.y = abs_vel
        else:
            info = 'Move right'
            vel.linear.y = -abs_vel
        self._vel_puber.publish(vel)
        step_num = math.floor(offset / abs_vel)
        rospy.loginfo("Brain: {} for {} steps ({:.3f} m/step)".format(info, int(step_num), abs_vel))
        rospy.sleep(step_num * 2 * 0.7)
        residual = offset - step_num * abs_vel
        return residual

    def turn_left_right(self, offset, abs_vel, is_negative):
        assert abs_vel > 0
        vel = Twist()
        if not is_negative:
            info = 'Turn left'
            vel.angular.z = abs_vel
        else:
            info = 'Turn right'
            vel.angular.z = -abs_vel
        rospy.logwarn(vel)
        self._vel_puber.publish(vel)
        step_num = math.floor(offset / abs_vel)
        rospy.loginfo("Brain: {} for {} steps ({:.3f} rad/step)".format(info, int(step_num), abs_vel))
        rospy.sleep(step_num * 2 * 0.7)
        residual = offset - step_num * abs_vel
        return residual

    def cmd_handle(self, req):
        resp = DummyResponse()
        if req.header.frame_id == 'start':
            self.on_start()
            resp.result_status = resp.SUCCEEDED
        elif req.header.frame_id == 'stop':
            self.on_stop()
            resp.result_status = resp.SUCCEEDED
        else:
            rospy.logerr("Brain: Unknown walk cmd {}".format(req.header.frame_id))
            resp.result_status = resp.FAILED
        return resp

    @staticmethod
    def stabilize_handle(req):
        resp = DummyResponse()
        rospy.logwarn("Brain: Stabilizing the base for 1 sec.")
        rospy.sleep(2)
        resp.result_status = resp.SUCCEEDED
        return resp

    def adjust_yaw_handle(self, req):
        resp = MoveToPose2DResponse()
        self.on_start()

        rospy.loginfo("Brain: Received rotation adjusting request: {}".format(req.nav_pose))
        if req.nav_pose.theta != 0:
            residual = abs(req.nav_pose.theta)
            abs_vel = min(residual, self._r_primary_vel)
            if abs_vel >= self._r_min_step:
                is_negative = False
                if req.nav_pose.theta < 0:
                    is_negative = True
                while residual >= self._r_min_step and abs(abs_vel) >= self._r_min_step:
                    residual = self.turn_left_right(residual, abs_vel, is_negative)
                    if self._r_min_step <= residual <= self._r_primary_vel:
                        abs_vel = residual
                    else:
                        abs_vel /= 2.0

        self.on_stop()
        resp.result_status = resp.SUCCEEDED
        return resp

    def handle(self, req):
        resp = MoveToPose2DResponse()
        self.on_start()

        rospy.loginfo("Brain: Walk helper received call: {}".format(req.nav_pose))

        if req.nav_pose.y != 0:
            residual = abs(req.nav_pose.y)
            abs_vel = min(residual, self._y_primary_vel)
            if abs_vel >= self._y_min_step:
                is_negative = False
                if req.nav_pose.y < 0:
                    is_negative = True
                while residual >= self._y_min_step and abs(abs_vel) >= self._y_min_step:
                    residual = self.move_left_right(residual, abs_vel, is_negative)
                    if self._y_min_step <= residual <= self._y_primary_vel:
                        abs_vel = residual
                    else:
                        abs_vel /= 2.0

        if req.nav_pose.x != 0:
            residual = abs(req.nav_pose.x)
            abs_vel = min(residual, self._x_primary_vel)
            if abs_vel >= self._x_min_step:
                is_negative = False
                if req.nav_pose.x < 0:
                    is_negative = True
                while residual >= self._x_min_step and abs(abs_vel) >= self._x_min_step:
                    residual = self.move_forward_backward(residual, abs_vel, is_negative)
                    if self._x_min_step <= residual <= self._x_primary_vel:
                        abs_vel = residual
                    else:
                        abs_vel /= 2.0

        if req.nav_pose.theta != 0:
            residual = abs(req.nav_pose.theta)
            abs_vel = min(residual, self._r_primary_vel)
            if abs_vel >= self._r_min_step:
                is_negative = False
                if req.nav_pose.theta < 0:
                    is_negative = True
                while residual >= self._r_min_step and abs(abs_vel) >= self._r_min_step:
                    residual = self.turn_left_right(residual, abs_vel, is_negative)
                    if self._r_min_step <= residual <= self._r_primary_vel:
                        abs_vel = residual
                    else:
                        abs_vel /= 2.0

        self.on_stop()
        resp.result_status = resp.SUCCEEDED
        return resp

    def call(self, cmd):
        try:
            req = leg_motion_MetaFuncCtrlRequest()
            req.cmd = cmd
            req.func_name = req.FUNC_DYNAMIC
            resp = self._remote.call(req)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


if __name__ == "__main__":
    try:
        rospy.init_node('walk_helper')
        rospy.wait_for_service('/Leg/TaskScheduler')
        mtp = MoveToPoseServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
