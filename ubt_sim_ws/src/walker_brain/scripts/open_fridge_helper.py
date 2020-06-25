#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Pose2D
from walker_brain.srv import EstimateTargetPose, EstimateTargetPoseResponse

from rotools.utility import transform


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
        T = transform.quaternion_matrix([tgt_pose.orientation.x, tgt_pose.orientation.y,
                                         tgt_pose.orientation.z, tgt_pose.orientation.w])
        rotation_on_z = transform.euler_from_matrix(T)[-1]
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
