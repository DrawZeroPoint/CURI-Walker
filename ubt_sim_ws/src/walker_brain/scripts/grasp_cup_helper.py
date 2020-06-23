#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist, Pose2D
from walker_brain.srv import EstimateTargetPose, EstimateTargetPoseResponse, MoveToPose2D, MoveToPose2DResponse
from walker_srvs.srv import leg_motion_MetaFuncCtrl, leg_motion_MetaFuncCtrlRequest


class EstimateServer(object):

    def __init__(self):
        super(EstimateServer, self).__init__()

        self._tgt_id = rospy.get_param('~target_id') - 1  # cup number start from 1
        self._server = rospy.Service('estimate_target_pose', EstimateTargetPose, self.handle)

        self._x_offset = rospy.get_param('~x_offset')
        self._y_offset = rospy.get_param('~y_offset')
        self._z_offset = rospy.get_param('~z_offset')

    def handle(self, req):
        resp = EstimateTargetPoseResponse()

        pose_num = len(req.obj_poses.poses)
        if self._tgt_id > pose_num - 1:
            rospy.logwarn("Brain: Not enough detection, known # {}".format(pose_num))
            resp.result_status = resp.FAILED
            return resp
        else:
            grasp_num = self._tgt_id + 1
            tgt_pose = req.obj_poses.poses[self._tgt_id]
            rospy.loginfo("Brain: Got pose of target {} \n {}\n".format(grasp_num, tgt_pose))
            resp.tgt_nav_pose = Pose2D()
            resp.tgt_nav_pose.x = tgt_pose.position.x - self._x_offset
            resp.tgt_nav_pose.y = tgt_pose.position.y + self._y_offset
            resp.tgt_nav_pose.theta = 0

            resp.tgt_grasp_pose.position.x = self._x_offset
            resp.tgt_grasp_pose.position.y = -self._y_offset
            resp.tgt_grasp_pose.position.z = tgt_pose.position.z
            resp.tgt_grasp_pose.orientation = tgt_pose.orientation

            resp.tgt_pre_grasp_pose = resp.tgt_grasp_pose
            resp.tgt_pre_grasp_pose.position.z += self._z_offset

            resp.result_status = resp.SUCCEEDED

            return resp


class MoveToPoseServer(object):

    def __init__(self):
        super(MoveToPoseServer, self).__init__()

        self._server = rospy.Service('move_to_pose2d', MoveToPose2D, self.handle)
        self._remote = rospy.ServiceProxy('/Leg/TaskScheduler', leg_motion_MetaFuncCtrl)
        self._vel_puber = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=1)

        self._x_primary_vel = 0.1
        self._y_primary_vel = 0.04
        self._x_fine_vel = 0.005
        self._y_fine_vel = 0.005

    def on_stop(self):
        vel = Twist()
        self._vel_puber.publish(vel)
        self.call("stop")

    def move_along_x(self, x_offset):
        if x_offset == 0:
            return

        vel = Twist()
        if x_offset > 0:
            vel.linear.x = self._x_primary_vel
        else:
            vel.linear.x = -self._x_primary_vel
        self._vel_puber.publish(vel)
        step_num_x = int(abs(x_offset) / self._x_primary_vel)
        rospy.loginfo("Brain: Steps along x for {} steps in primary speed".format(step_num_x))
        rospy.sleep(step_num_x * 0.7)

        residual = abs(x_offset) - step_num_x * self._x_primary_vel
        if residual <= 0.005:
            self.on_stop()
            return

        if x_offset > 0:
            vel.linear.x = self._x_fine_vel
        else:
            vel.linear.x = -self._x_fine_vel
        self._vel_puber.publish(vel)
        step_num_x = int(abs(residual) / self._x_fine_vel)
        rospy.loginfo("Brain: Steps along x for {} steps in fine speed".format(step_num_x))
        rospy.sleep(step_num_x * 0.7)
        self.on_stop()

    def move_along_y(self, y_offset):
        if y_offset == 0:
            return

        vel = Twist()
        if y_offset > 0:
            vel.linear.y = self._y_primary_vel
        else:
            vel.linear.y = -self._y_primary_vel
        self._vel_puber.publish(vel)
        step_num_y = int(abs(y_offset) / self._y_primary_vel) * 2
        rospy.loginfo("Brain: Steps along y for {} steps in primary speed".format(step_num_y))
        rospy.sleep(step_num_y * 0.7)

        residual = abs(y_offset) - step_num_y * self._y_primary_vel
        if residual <= 0.005:
            self.on_stop()
            return

        if y_offset > 0:
            vel.linear.y = self._y_fine_vel
        else:
            vel.linear.y = -self._y_fine_vel
        self._vel_puber.publish(vel)
        step_num_y = int(abs(residual) / self._y_fine_vel) * 2
        rospy.loginfo("Brain: Steps along y for {} steps in fine speed".format(step_num_y))
        rospy.sleep(step_num_y * 0.7)
        self.on_stop()

    def handle(self, req):
        resp = MoveToPose2DResponse()
        self.call("start")
        self.move_along_x(req.nav_pose.x)
        self.move_along_y(req.nav_pose.y)
        self.call("stop")
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
        rospy.init_node('grasp_cup_helper')
        es = EstimateServer()
        mtp = MoveToPoseServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
