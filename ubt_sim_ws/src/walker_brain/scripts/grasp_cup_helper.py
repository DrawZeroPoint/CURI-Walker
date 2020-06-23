#!/usr/bin/env python

from __future__ import print_function

import rospy

from walker_brain.srv import EstimateTargetPose, EstimateTargetPoseResponse


class EstimateServer(object):

    def __init__(self):
        super(EstimateServer, self).__init__()

        rospy.init_node('estimate_target_pose_server')
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
            resp.tgt_nav_pose.position.x = tgt_pose.position.x - self._x_offset
            resp.tgt_nav_pose.position.y = tgt_pose.position.y + self._y_offset
            resp.tgt_nav_pose.position.z = tgt_pose.position.z
            resp.tgt_nav_pose.orientation = tgt_pose.orientation

            resp.tgt_grasp_pose.position.x = self._x_offset
            resp.tgt_grasp_pose.position.y = -self._y_offset
            resp.tgt_grasp_pose.position.z = tgt_pose.position.z
            resp.tgt_grasp_pose.orientation = tgt_pose.orientation

            resp.tgt_pre_grasp_pose = resp.tgt_grasp_pose
            resp.tgt_pre_grasp_pose.position.z += self._z_offset

            resp.result_status = resp.SUCCEEDED

            return resp


if __name__ == "__main__":
    try:
        es = EstimateServer()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
