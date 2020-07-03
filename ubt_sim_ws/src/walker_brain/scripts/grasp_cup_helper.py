#!/usr/bin/env python

from __future__ import print_function

import math
import rospy

from sensor_msgs.msg import Range, JointState
from geometry_msgs.msg import Twist, Pose2D, PoseArray, WrenchStamped
from walker_brain.srv import EstimateTargetPose, EstimateTargetPoseResponse, \
    EstimateContactForce, EstimateContactForceResponse


class EstimateServer(object):

    def __init__(self):
        super(EstimateServer, self).__init__()

        self.l_arm_force_suber = rospy.Subscriber('/sensor/ft/lwrist', WrenchStamped, self.l_arm_force_cb)
        self.r_arm_force_suber = rospy.Subscriber('/sensor/ft/rwrist', WrenchStamped, self.r_arm_force_cb)

        self.lb_us_suber = rospy.Subscriber('/walker/ultrasound/leftBack', Range, self.lb_us_cb)
        self.mb_us_suber = rospy.Subscriber('/walker/ultrasound/middleBack', Range, self.mb_us_cb)
        self.rb_us_suber = rospy.Subscriber('/walker/ultrasound/rightBack', Range, self.rb_us_cb)
        self.lb_dis = 0
        self.mb_dis = 0
        self.rb_dis = 0

        self._tgt_id = rospy.get_param('~target_id') - 1  # cup number start from 1
        self._server = rospy.Service('estimate_target_pose', EstimateTargetPose, self.handle)
        self._adjust_server = rospy.Service('estimate_adjust_pose', EstimateTargetPose, self.range_handle)
        self._force_server = rospy.Service('estimate_contact_force', EstimateContactForce, self.force_handle)

        self._x_offset = rospy.get_param('~x_offset')
        self._y_offset = rospy.get_param('~y_offset')

        self._fixed_pose = rospy.get_param('~fixed_pose')

        self._best_distance = 1.21

        self._l_arm_force = None
        self._r_arm_force = None

    def l_arm_force_cb(self, msg):
        self._l_arm_force = msg

    def r_arm_force_cb(self, msg):
        self._r_arm_force = msg

    def force_handle(self, req):
        resp = EstimateContactForceResponse()
        if req.header.frame_id == "left_wrist":
            f = self._l_arm_force
        elif req.header.frame_id == "right_wrist":
            f = self._r_arm_force
        else:
            rospy.logerr("Brain: Unknown force sensor type %s", req.header.frame_id)
            resp.result_status = resp.FAILED
            return resp

        rospy.logwarn("Brain: Wrench \n{}".format(f.wrench.force))
        if req.max_force and req.min_force and req.max_force > req.min_force:
            max_force = req.max_force
            min_force = req.min_force
        else:
            rospy.logerr("Brain: Contact force range not given")
            max_force = 1000
            min_force = 10

        if req.direction == "x":
            rospy.logwarn("Brain: Contact force in x direction %.3f", f.wrench.force.x)
            if abs(f.wrench.force.x) > min_force:
                if abs(f.wrench.force.x) < max_force:
                    resp.result_status = resp.IN_RANGE
                else:
                    resp.result_status = resp.HIGHER_THAN_MAX_FORCE
            else:
                resp.result_status = resp.LOWER_THAN_MIN_FORCE
        elif req.direction == "z":
            rospy.logwarn("Brain: Contact force in z direction %.3f", f.wrench.force.z)
            if abs(f.wrench.force.z) > min_force:
                if abs(f.wrench.force.z) < max_force:
                    resp.result_status = resp.IN_RANGE
                else:
                    resp.result_status = resp.HIGHER_THAN_MAX_FORCE
            else:
                resp.result_status = resp.LOWER_THAN_MIN_FORCE
        else:
            if abs(f.wrench.force.x) > min_force or abs(f.wrench.force.y) > min_force \
                    or abs(f.wrench.force.z) > min_force:
                if abs(f.wrench.force.x) < max_force and abs(f.wrench.force.y) < max_force \
                        and abs(f.wrench.force.z) < max_force:
                    resp.result_status = resp.IN_RANGE
                else:
                    resp.result_status = resp.HIGHER_THAN_MAX_FORCE
            else:
                resp.result_status = resp.LOWER_THAN_MIN_FORCE
        return resp

    def lb_us_cb(self, msg):
        self.lb_dis = msg.range

    def mb_us_cb(self, msg):
        self.mb_dis = msg.range

    def rb_us_cb(self, msg):
        self.rb_dis = msg.range

    def range_handle(self, req):
        resp = EstimateTargetPoseResponse()

        if self.lb_dis and self.mb_dis and self.rb_dis:
            rospy.logwarn("Brain: Back ultrasound distance, left %.3f, middle %.3f, right %.3f",
                          self.lb_dis, self.mb_dis, self.rb_dis)
            x_offset = self._best_distance - self.mb_dis
            resp.tgt_nav_pose.x = x_offset - 0.1
            resp.tgt_nav_pose.y = x_offset * 1.0

            if abs(self.lb_dis - self.rb_dis) < 0.01:
                resp.compensate_pose.theta = 0
                rospy.logwarn("Brain: No need to compensate rotation")
            else:
                # The distance between lb and rb ultrasound sensors is 0.184 mm
                resp.compensate_pose.theta = math.atan2(self.lb_dis - self.rb_dis, 0.184)
                # rospy.logwarn("Brain: Rotation compensate %.3f", resp.compensate_pose.theta)
            resp.result_status = resp.SUCCEEDED
        else:
            rospy.logerr("Brain: No single from back ultrasound sensors")
            resp.result_status = resp.FAILED
        return resp

    @staticmethod
    def sort_poses(pose_array):
        """Sort the given Pose list according to poses' y coordinates,
        return pose list with y descending (In the grasp cup case, left to right)

        :param pose_array: List[Pose] equal to PoseArray.poses
        """
        pose_array.sort(key=lambda p: p.position.y, reverse=True)
        return pose_array

    @staticmethod
    def get_tgt_pose(tgt_id):
        if tgt_id == 0:
            tgt_hover_pose = [-5, -71, -95, -86, -1, 3, 5]
            tgt_pre_grasp_pose = [10, -47, -69, -59, 26, 21, 14]
            tgt_grasp_pose = [17, -35, -74, -55, 29, 17, 15]
        elif tgt_id == 1:
            tgt_hover_pose = [50, -62, -35, -102, 9, 15, -6]
            tgt_pre_grasp_pose = [35, -42, -31, -82, 24, 23, -1]
            tgt_grasp_pose = [39, -35, -30, -79, 25, 20, -4]
        elif tgt_id == 2:
            tgt_hover_pose = [-70, -53, 29, -112, -1, -2, -24]
            tgt_pre_grasp_pose = [-52, -25, 12, -76, -23, -23, -15]
            tgt_grasp_pose = [-52, -22, 14, -75, -19, -22, -14]
        elif tgt_id == 3:
            tgt_hover_pose = [-18, -67, 58, -97, -8, -21, -2]
            tgt_pre_grasp_pose = [-14, -40, 57, -69, -28, -24, 10]
            tgt_grasp_pose = [-25, -31, 53, -66, -34, -18, 2]
        else:
            tgt_hover_pose = [22, -76, 107, -80, 2, -4, -18]
            tgt_pre_grasp_pose = [-6, -48, 74, -52, -18, -23, -6]
            tgt_grasp_pose = [-30, -36, 67, -26, -20, -23, 0]
        return tgt_hover_pose, tgt_pre_grasp_pose, tgt_grasp_pose

    def fixed_handle(self):
        resp = EstimateTargetPoseResponse()
        if self._tgt_id < 0 or self._tgt_id > 4:
            rospy.logerr("Brain: Invalid target id: {}".format(self._tgt_id))
            resp.result_status = resp.FAILED
            return resp

        hover_pose, pre_pose, grasp_pose = self.get_tgt_pose(self._tgt_id)
        resp.tgt_hover_pose = hover_pose
        resp.tgt_pre_grasp_pose = pre_pose
        resp.tgt_grasp_pose = grasp_pose

        resp.result_status = resp.SUCCEEDED
        return resp

    def handle(self, req):
        if self._fixed_pose:
            return self.fixed_handle()

        resp = EstimateTargetPoseResponse()

        pose_num = len(req.obj_poses.poses)
        rospy.logdebug("Brain: Detected pose # {}".format(pose_num))

        sorted_poses = self.sort_poses(req.obj_poses.poses)
        rospy.logdebug("Brain: Sorted poses \n{}".format(sorted_poses))

        ok, rotation = self.calculate_compensation(sorted_poses)
        if ok:
            resp.compensate_pose.theta = rotation
        else:
            resp.result_status = resp.FAILED
            return resp

        if pose_num >= 3:
            tgt_pose = sorted_poses[2]  # use the 3rd cup as reference
            rospy.logdebug("Brain: Got target pose \n{}".format(tgt_pose))
            resp.tgt_nav_pose = Pose2D()
            resp.tgt_nav_pose.x = tgt_pose.position.x - self._x_offset
            resp.tgt_nav_pose.y = tgt_pose.position.y + self._y_offset
            resp.tgt_nav_pose.theta = 0

            hover_pose, pre_pose, grasp_pose = self.get_tgt_pose(self._tgt_id)
            resp.tgt_hover_pose = hover_pose
            resp.tgt_pre_grasp_pose = pre_pose
            resp.tgt_grasp_pose = grasp_pose

        resp.result_status = resp.SUCCEEDED
        return resp

    @staticmethod
    def calculate_compensation(poses):
        num_pose = len(poses)
        if num_pose < 2:
            rospy.logerr("Brain: Not enough target for compensate calculation")
            return False, None

        head_pose = poses[0]
        tail_pose = poses[-1]
        delta_x = tail_pose.position.x - head_pose.position.x
        delta_y = tail_pose.position.y - head_pose.position.y
        rotation = math.atan2(delta_y, delta_x) + math.pi * 0.5
        rospy.logwarn("Brain: Rotation compensation is %.3f deg", rotation / math.pi * 180.)
        return True, rotation


if __name__ == "__main__":
    try:
        rospy.init_node('grasp_cup_helper')
        es = EstimateServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
