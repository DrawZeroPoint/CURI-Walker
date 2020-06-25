#!/usr/bin/env python

from __future__ import print_function

import rospy
import math

from geometry_msgs.msg import Twist, Pose2D
from walker_brain.srv import MoveToPose2D, MoveToPose2DResponse
from walker_srvs.srv import leg_motion_MetaFuncCtrl, leg_motion_MetaFuncCtrlRequest


class MoveToPoseServer(object):

    def __init__(self):
        super(MoveToPoseServer, self).__init__()

        self._server = rospy.Service('move_to_pose2d', MoveToPose2D, self.handle)
        self._remote = rospy.ServiceProxy('/Leg/TaskScheduler', leg_motion_MetaFuncCtrl)
        self._vel_puber = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=1)

        self._x_primary_vel = 0.2
        self._y_primary_vel = 0.04
        self._x_min_step = 0.005
        self._y_min_step = 0.005

    def on_stop(self):
        vel = Twist()
        self._vel_puber.publish(vel)
        self.call("stop")
        rospy.sleep(1)

    def move_forward_backward(self, offset, abs_vel):
        if offset == 0:
            return

        vel = Twist()
        if offset > 0:
            info = 'forward'
            vel.linear.x = abs_vel
        else:
            info = 'backward'
            vel.linear.x = -abs_vel
        self._vel_puber.publish(vel)

        step_num_x = math.floor(abs(offset) / abs_vel)
        rospy.loginfo("Brain: {} for {} steps ({} m/step)".format(info, step_num_x, abs_vel))
        rospy.sleep(step_num_x * 0.7)

        residual = abs(offset) - step_num_x * abs_vel
        return residual

    def move_left_right(self, offset, abs_vel):
        if offset == 0:
            return

        vel = Twist()
        if offset > 0:
            info = 'Move left'
            vel.linear.y = abs_vel
        else:
            info = 'Move right'
            vel.linear.y = -abs_vel
        self._vel_puber.publish(vel)
        step_num_y = math.floor(abs(offset) / abs_vel)
        rospy.loginfo("Brain: {} for {} steps ({} m/step)".format(info, step_num_y, abs_vel))
        # For Walker slides, each step takes twice the time, add 2 for stabling
        rospy.sleep(step_num_y * 2 * 0.7)

        residual = abs(offset) - step_num_y * abs_vel
        return residual

    def handle(self, req):
        resp = MoveToPose2DResponse()
        self.call("start")

        y_vel_init_ = self._y_primary_vel
        residual = req.nav_pose.y
        while abs(residual) > self._y_min_step and y_vel_init_ >= self._y_min_step:
            residual = self.move_left_right(residual, y_vel_init_)
            y_vel_init_ /= 2.0

        x_vel_init_ = self._x_primary_vel
        residual = req.nav_pose.x
        while abs(residual) > self._x_min_step and x_vel_init_ >= self._x_min_step:
            residual = self.move_forward_backward(residual, x_vel_init_)
            x_vel_init_ /= 2.0

        self.on_stop()
        resp.result_status = resp.SUCCEEDED
        return resp

    def call(self, cmd):
        rospy.wait_for_service('/Leg/TaskScheduler')
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
        mtp = MoveToPoseServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
