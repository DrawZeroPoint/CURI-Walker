#!/usr/bin/env python

from __future__ import print_function

import math
import numpy as np
import rospy

from geometry_msgs.msg import Pose2D, WrenchStamped
from sensor_msgs.msg import Range
from walker_brain.srv import EstimateTargetPose, EstimateTargetPoseResponse, \
                             EstimateContactForce, EstimateContactForceResponse, \
                             Dummy, DummyResponse


# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())


def quaternion_matrix(quaternion):
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
    ), dtype=np.float64)


def euler_from_matrix(matrix, axes='sxyz'):
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


class EstimateServer(object):

    def __init__(self):
        super(EstimateServer, self).__init__()

        self.l_arm_force_suber = rospy.Subscriber('/sensor/ft/lwrist', WrenchStamped, self.l_arm_force_cb)
        self.r_arm_force_suber = rospy.Subscriber('/sensor/ft/rwrist', WrenchStamped, self.r_arm_force_cb)

        self.lf_us_suber = rospy.Subscriber('/walker/ultrasound/leftFront', Range, self.lf_us_cb)
        self.rf_us_suber = rospy.Subscriber('/walker/ultrasound/rightFront', Range, self.rf_us_cb)
        self.lf_dis = 0
        self.rf_dis = 0

        self._server = rospy.Service('estimate_target_pose', EstimateTargetPose, self.handle)
        self._adjust_server = rospy.Service('estimate_adjust_pose', EstimateTargetPose, self.range_handle)
        self._force_server = rospy.Service('estimate_contact_force', EstimateContactForce, self.force_handle)

        self._x_offset = rospy.get_param('~x_offset')
        self._y_offset = rospy.get_param('~y_offset')
        self._r_th = rospy.get_param('~rotation_tolerance')

        self._range_max_tolerance = 0.2
        self._best_distance = 0.8

        self._l_arm_force = None
        self._r_arm_force = None

    def l_arm_force_cb(self, msg):
        self._l_arm_force = msg

    def r_arm_force_cb(self, msg):
        self._r_arm_force = msg

    def lf_us_cb(self, msg):
        self.lf_dis = msg.range

    def rf_us_cb(self, msg):
        self.rf_dis = msg.range

    @staticmethod
    def sort_poses(pose_array):
        """Sort the given Pose list according to poses' x coordinates,
        return pose list with x ascending

        :param pose_array: List[Pose] equal to PoseArray.poses
        """
        pose_array.sort(key=lambda p: p.position.x, reverse=False)
        return pose_array

    @staticmethod
    def get_rotation_along_z(pose):
        T = quaternion_matrix([pose.orientation.x, pose.orientation.y,
                               pose.orientation.z, pose.orientation.w])
        return euler_from_matrix(T)[-1]

    def handle(self, req):
        resp = EstimateTargetPoseResponse()

        valid_poses = []
        for p in req.obj_poses.poses:
            rotation = self.get_rotation_along_z(p)
            if abs(rotation) > self._r_th:
                continue
            valid_poses.append(p)
            rospy.logdebug("Brain: Unsorted pose {}\n".format(p))

        if not valid_poses:
            rospy.logerr("Brain: No valid pose")
            resp.result_status = resp.FAILED
            return resp

        sorted_poses = self.sort_poses(valid_poses)
        tgt_pose = sorted_poses[0]

        tgt_rotation = self.get_rotation_along_z(tgt_pose)
        rospy.logwarn("Brain: Target rotation wrt base_link %.3f (deg)", tgt_rotation * 180. / np.pi)

        resp.tgt_nav_pose = Pose2D()
        resp.tgt_nav_pose.x = tgt_pose.position.x - self._x_offset
        resp.tgt_nav_pose.y = tgt_pose.position.y + self._y_offset
        resp.tgt_nav_pose.theta = 0

        resp.compensate_pose = Pose2D()
        resp.compensate_pose.theta = tgt_rotation

        resp.result_status = resp.SUCCEEDED
        return resp

    def range_handle(self, req):
        resp = EstimateTargetPoseResponse()

        if self.lf_dis and self.rf_dis:
            rospy.logwarn("Brain: Front ultrasound distance, left %.3f, right %.3f", self.lf_dis, self.rf_dis)
            min_range = min(self.lf_dis, self.rf_dis)
            if min_range < self._best_distance:
                resp.tgt_nav_pose.x = min_range - self._best_distance
            if abs(self.lf_dis - self.rf_dis) <= self._range_max_tolerance:
                resp.tgt_nav_pose.y -= min_range * 0.25
            elif abs(self.lf_dis - self.rf_dis) > self._range_max_tolerance:
                resp.tgt_nav_pose.y -= min_range * 0.5
            resp.result_status = resp.SUCCEEDED
        else:
            rospy.logerr("Brain: No single from front ultrasound")
            resp.result_status = resp.FAILED
        return resp

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

        rospy.logwarn("Brain: Wrench {}".format(f.wrench.force))
        if req.max_force and req.min_force and req.max_force > req.min_force:
            max_force = req.max_force
            min_force = req.min_force
        else:
            rospy.logerr("Brain: Contact force range not given")
            max_force = 1000
            min_force = 10
        if abs(f.wrench.force.x) > min_force or abs(f.wrench.force.y) > min_force or abs(f.wrench.force.z) > min_force:
            if abs(f.wrench.force.x) < max_force and abs(f.wrench.force.y) < max_force and abs(f.wrench.force.z) < max_force:
                resp.result_status = resp.IN_RANGE
            else:
                resp.result_status = resp.HIGHER_THAN_MAX_FORCE
        else:
            resp.result_status = resp.LOWER_THAN_MIN_FORCE
        return resp


if __name__ == "__main__":
    try:
        rospy.init_node('open_fridge_helper')
        es = EstimateServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
