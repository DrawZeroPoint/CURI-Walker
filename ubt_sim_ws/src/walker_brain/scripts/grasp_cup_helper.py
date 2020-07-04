#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist, Pose2D, PoseArray
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

        self._fixed_pose = rospy.get_param('~fixed_pose')

    @staticmethod
    def sort_poses(pose_array):
        """Sort the given Pose list according to poses' y coordinates,
        return pose list with y descending

        :param pose_array: List[Pose] equal to PoseArray.poses
        """
        pose_array.sort(key=lambda p: p.position.y, reverse=True)
        return pose_array

    def fixed_handle(self):
        resp = EstimateTargetPoseResponse()
        if self._tgt_id < 0 or self._tgt_id > 4:
            rospy.logerr("Brain: Invalid target id: {}".format(self._tgt_id))
            resp.result_status = resp.FAILED
            return resp

        if self._tgt_id == 0:
            resp.tgt_hover_pose = [-5, -71, -95, -86, -1, 3, 5]
            resp.tgt_pre_grasp_pose = [11, -38, -77, -55, 25, 19, 20]
            resp.tgt_grasp_pose = [16, -36, -74, -55, 28, 17, 16]
        elif self._tgt_id == 1:
            resp.tgt_hover_pose = [49, -52, -41, -101, 15, 7, -11]
            resp.tgt_pre_grasp_pose = [38, -38, -30, -80, 25, 21, -3]
            resp.tgt_grasp_pose = [39, -35, -30, -79, 25, 20, -4]
        elif self._tgt_id == 2:
            resp.tgt_hover_pose = [-72, -48, 28, -108, -2, -1, -25]
            resp.tgt_pre_grasp_pose = [-52, -25, 12, -76, -23, -23, -15]
            resp.tgt_grasp_pose = [-52, -22, 14, -75, -19, -22, -14]
        elif self._tgt_id == 3:
            resp.tgt_hover_pose = [-18, -67, 58, -97, -8, -21, -2]
            resp.tgt_pre_grasp_pose = [-14, -40, 57, -69, -28, -24, 10]
            resp.tgt_grasp_pose = [-25, -31, 53, -66, -34, -18, 2]
        else:
            resp.tgt_hover_pose = [-5, -74, 85, -76, -2, -1, -21]
            resp.tgt_pre_grasp_pose = [-21, -42, 68, -38, -22, -23, -3]
            resp.tgt_grasp_pose = [-30, -36, 67, -26, -20, -23, 0]

        resp.result_status = resp.SUCCEEDED
        return resp

    def handle(self, req):
        if self._fixed_pose:
            return self.fixed_handle()

        resp = EstimateTargetPoseResponse()

        pose_num = len(req.obj_poses.poses)
        if self._tgt_id > pose_num - 1:
            rospy.logwarn("Brain: Not enough detection, known # {}".format(pose_num))
            resp.result_status = resp.FAILED
            return resp
        else:
            grasp_num = self._tgt_id + 1
            rospy.loginfo("Brain: Unsorted poses {}\n".format(req.obj_poses.poses))
            sorted_poses = self.sort_poses(req.obj_poses.poses)
            rospy.logdebug("Brain: Sorted poses {}\n".format(sorted_poses))

            tgt_pose = sorted_poses[self._tgt_id]
            rospy.loginfo("Brain: Got pose of target {} \n {}\n".format(grasp_num, tgt_pose))
            resp.tgt_nav_pose = Pose2D()
            resp.tgt_nav_pose.x = tgt_pose.position.x - self._x_offset
            resp.tgt_nav_pose.y = tgt_pose.position.y - self._y_offset
            resp.tgt_nav_pose.theta = 0

            resp.tgt_grasp_pose.position.x = self._x_offset
            resp.tgt_grasp_pose.position.y = self._y_offset
            resp.tgt_grasp_pose.position.z = -0.18
            resp.tgt_grasp_pose.orientation.x = 0
            resp.tgt_grasp_pose.orientation.y = 0
            resp.tgt_grasp_pose.orientation.z = -0.38268343
            resp.tgt_grasp_pose.orientation.w = 0.92387953
            rospy.loginfo("Brain: Planned grasp pose {}\n".format(resp.tgt_grasp_pose))

            # Note that here we cannot use direct match: resp.tgt_pre_grasp_pose = resp.tgt_grasp_pose
            resp.tgt_pre_grasp_pose.position.x = self._x_offset
            resp.tgt_pre_grasp_pose.position.y = self._y_offset
            resp.tgt_pre_grasp_pose.position.z = 0
            resp.tgt_pre_grasp_pose.orientation.x = 0
            resp.tgt_pre_grasp_pose.orientation.y = 0
            resp.tgt_pre_grasp_pose.orientation.z = 0
            resp.tgt_pre_grasp_pose.orientation.w = 1
            rospy.loginfo("Brain: Planned pre grasp pose {}\n".format(resp.tgt_pre_grasp_pose))

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
        self._x_min_step = 0.005
        self._y_min_step = 0.005

    def on_stop(self):
        vel = Twist()
        self._vel_puber.publish(vel)
        self.call("stop")
        rospy.sleep(1)

    def move_along_x(self, x_offset, x_vel):
        if x_offset == 0:
            return

        vel = Twist()
        pn = '+'
        if x_offset > 0:
            vel.linear.x = x_vel
        else:
            pn = '-'
            vel.linear.x = -x_vel
        self._vel_puber.publish(vel)
        step_num_x = int(abs(x_offset) / x_vel)
        rospy.loginfo("Brain: Steps along {}x for {} steps ({} m/step)".format(pn, step_num_x, x_vel))
        rospy.sleep(step_num_x * 0.7)

        residual = abs(x_offset) - step_num_x * x_vel
        return residual

    def move_along_y(self, y_offset, y_vel):
        if y_offset == 0:
            return

        vel = Twist()
        pn = '+'
        if y_offset > 0:
            vel.linear.y = y_vel
        else:
            pn = '-'
            vel.linear.y = -y_vel
        self._vel_puber.publish(vel)
        step_num_y = int(abs(y_offset) / y_vel)
        rospy.loginfo("Brain: Steps along {}y for {} steps ({} m/step)".format(pn, step_num_y, y_vel))
        # For Walker slides, each step takes twice the time, add 2 for stabling
        rospy.sleep((step_num_y * 2 + 2) * 0.7)

        residual = abs(y_offset) - step_num_y * y_vel
        return residual

    def handle(self, req):
        resp = MoveToPose2DResponse()
        self.call("start")

        y_vel_init_ = self._y_primary_vel
        residual = req.nav_pose.y
        while abs(residual) > self._y_min_step and y_vel_init_ >= self._y_min_step:
            residual = self.move_along_y(residual, y_vel_init_)
            y_vel_init_ /= 2.0

        x_vel_init_ = self._x_primary_vel
        residual = req.nav_pose.x
        while abs(residual) > self._x_min_step and x_vel_init_ >= self._x_min_step:
            residual = self.move_along_x(residual, x_vel_init_)
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
        rospy.init_node('grasp_cup_helper')
        es = EstimateServer()
        mtp = MoveToPoseServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
