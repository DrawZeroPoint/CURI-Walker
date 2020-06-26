#!/usr/bin/env python

from __future__ import print_function

import math
import numpy
import rospy

from geometry_msgs.msg import Pose2D
from walker_brain.srv import EstimateTargetPose, EstimateTargetPoseResponse


# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

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
    q = numpy.array(quaternion[:4], dtype=numpy.float64, copy=True)
    nq = numpy.dot(q, q)
    if nq < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = numpy.outer(q, q)
    return numpy.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
    ), dtype=numpy.float64)


def euler_from_matrix(matrix, axes='sxyz'):
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
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

        self._server = rospy.Service('estimate_target_pose', EstimateTargetPose, self.handle)

        self._x_offset = rospy.get_param('~x_offset')
        self._y_offset = rospy.get_param('~y_offset')

    @staticmethod
    def sort_poses(pose_array):
        """Sort the given Pose list according to poses' x coordinates,
        return pose list with x ascending

        :param pose_array: List[Pose] equal to PoseArray.poses
        """
        pose_array.sort(key=lambda p: p.position.x, reverse=False)
        return pose_array

    def handle(self, req):
        resp = EstimateTargetPoseResponse()

        rospy.loginfo("Brain: Unsorted poses {}\n".format(req.obj_poses.poses))
        if not req.obj_poses.poses:
            resp.result_status = resp.FAILED
            return resp

        sorted_poses = self.sort_poses(req.obj_poses.poses)
        rospy.logdebug("Brain: Sorted poses {}\n".format(sorted_poses))

        tgt_pose = sorted_poses[0]
        T = quaternion_matrix([tgt_pose.orientation.x, tgt_pose.orientation.y,
                               tgt_pose.orientation.z, tgt_pose.orientation.w])
        rotation_on_z = euler_from_matrix(T)[-1]
        print(rotation_on_z)
        if abs(rotation_on_z) > 0.1:
            rospy.logwarn("Brain: Bad observing position, changing spot...")
            resp.result_status = resp.FAILED
            return resp

        rospy.loginfo("Brain: Got target pose \n {}\n".format(tgt_pose))
        resp.tgt_nav_pose = Pose2D()
        resp.tgt_nav_pose.x = tgt_pose.position.x - self._x_offset
        resp.tgt_nav_pose.y = tgt_pose.position.y + self._y_offset
        resp.tgt_nav_pose.theta = 0

        resp.result_status = resp.SUCCEEDED
        return resp


if __name__ == "__main__":
    try:
        rospy.init_node('open_fridge_helper')
        es = EstimateServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
