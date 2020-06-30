#!/usr/bin/env python

from __future__ import print_function

import rospy
import actionlib
import walker_movement.msg

import time
from walker_brain.srv import Dummy, DummyResponse, MoveToPose2D, MoveToPose2DRequest
from walker_srvs.srv import leg_motion_MetaFuncCtrl
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int64, String
from nav_msgs.msg import Odometry


class WalkerMovementUtils(object):

    def __init__(self):
        super(WalkerMovementUtils, self).__init__()

        self._server = rospy.Service('open_fridge', Dummy, self.handle)

        self.moveEeLeftClient = actionlib.SimpleActionClient('/walker/move_helper_left_arm/move_to_ee_pose',
                                                             walker_movement.msg.MoveToEePoseAction)
        self.moveJointsLeftClient = actionlib.SimpleActionClient('/walker/move_helper_left_arm/move_to_joint_pose',
                                                                 walker_movement.msg.MoveToJointPoseAction)
        self.graspLeftClient = actionlib.SimpleActionClient('/walker/hand_helper_left/grasp',
                                                            walker_movement.msg.GraspAction)
        self.moveEeRightClient = actionlib.SimpleActionClient('/walker/move_helper_right_arm/move_to_ee_pose',
                                                              walker_movement.msg.MoveToEePoseAction)
        self.moveJointsRightClient = actionlib.SimpleActionClient('/walker/move_helper_right_arm/move_to_joint_pose',
                                                                  walker_movement.msg.MoveToJointPoseAction)
        self.graspRightClient = actionlib.SimpleActionClient('/walker/hand_helper_right/grasp',
                                                             walker_movement.msg.GraspAction)

        rospy.loginfo("Waiting for action servers...")
        self.moveEeLeftClient.wait_for_server()
        self.moveJointsLeftClient.wait_for_server()
        self.graspLeftClient.wait_for_server()
        self.moveEeRightClient.wait_for_server()
        self.moveJointsRightClient.wait_for_server()
        self.graspRightClient.wait_for_server()
        rospy.loginfo("Action servers connected")

        rospy.wait_for_service('move_to_pose2d')
        self._walk_client = rospy.ServiceProxy('move_to_pose2d', MoveToPose2D)

    def moveToEePose(self, isLeft, position_xyz, orientation_xyzw, isRelative=False):
        rospy.loginfo("Moving to ee pose")
        if isLeft:
            c = self.moveEeLeftClient
            eel = "left_tcp"
        else:
            c = self.moveEeRightClient
            eel = "right_tcp"
        goal = walker_movement.msg.MoveToEePoseGoal()
        if isRelative:
            goal.pose.header.frame_id = eel
        else:
            goal.pose.header.frame_id = "base_link"
        goal.pose.pose.position.x = position_xyz[0]
        goal.pose.pose.position.y = position_xyz[1]
        goal.pose.pose.position.z = position_xyz[2]
        goal.pose.pose.orientation.x = orientation_xyzw[0]
        goal.pose.pose.orientation.y = orientation_xyzw[1]
        goal.pose.pose.orientation.z = orientation_xyzw[2]
        goal.pose.pose.orientation.w = orientation_xyzw[3]
        goal.end_effector_link = eel
        c.send_goal(goal)
        c.wait_for_result()
        rospy.loginfo("Moved")

    def graspCart(self, isLeft):
        rospy.loginfo("Grasping...")
        if isLeft:
            c = self.graspLeftClient
        else:
            c = self.graspRightClient
        c.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_CART_HANDLE_CORNER))
        c.wait_for_result()
        # print(c.get_result())
        rospy.loginfo("Grasped")

    def releaseGrasp(self, isLeft):
        rospy.loginfo("Opening hand...")
        if isLeft:
            c = self.graspLeftClient
        else:
            c = self.graspRightClient
        c.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_OPEN))
        c.wait_for_result()
        # print(c.get_result())
        rospy.loginfo("Opened")

    def moveToHome(self, isLeft):
        rospy.loginfo("Moving to home pose")
        if isLeft:
            c = self.moveJointsLeftClient
        else:
            c = self.moveJointsRightClient
        c.send_goal(walker_movement.msg.MoveToJointPoseGoal([0, 0, 0, 0, 0, 0, 0]))
        c.wait_for_result()
        # print(c.get_result())
        rospy.loginfo("Moved")

    def graspCan(self, isLeft):
        rospy.loginfo("Grasping...")
        if isLeft:
            c = self.graspLeftClient
        else:
            c = self.graspRightClient
        c.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_CAN))
        c.wait_for_result()
        # print(graspClient.get_result())
        rospy.loginfo("Grasped")

    def handle(self, req):
        resp = DummyResponse()

        rospy.loginfo("Brain: Received open fridge call")

        rightPosition = [0.419, -0.061, -0.008]
        rightOrientation = [0.020, 0.049, 0.704, 0.708]
        rightPosition2 = [0.428, -0.002, 0.002]
        rightOrientation2 = [0.020, 0.052, 0.690, 0.722]

        leftPosition1 = [0.368, 0.074, -0.090]
        leftOrientation1 = [0.095, -0.138, -0.451, 0.877]
        leftPositionF = [0.332, -0.055, -0.114]
        leftOrientationF = [0.125, -0.102, -0.664, 0.730]

        move_request = MoveToPose2DRequest()
        move_request.nav_pose.theta = -0.41  # 10: -0.55 11: -0.41
        self._walk_client.call(move_request)
        move_request = MoveToPose2DRequest()
        move_request.nav_pose.x = -0.1
        self._walk_client.call(move_request)

        self.moveToEePose(False, rightPosition, rightOrientation)
        self.moveToEePose(False, rightPosition2, rightOrientation2)
        self.graspCan(False)
        self.moveToEePose(False, [-0.15, 0.15, 0], [0, 0, 0, 1], True)
        self.moveToEePose(False, [-0.01, -0.01, 0], [0, 0, 0.087, 0.996], True)

        self.moveToEePose(True, leftPosition1, leftOrientation1)

        self.releaseGrasp(False)
        self.moveToHome(False)

        self.moveToEePose(True, (0.1, 0.05, 0), (0, 0, 0, 1), True)
        self.moveToEePose(True, leftPositionF, leftOrientationF, False)

        resp.result_status = resp.SUCCEEDED
        return resp


if __name__ == "__main__":
    try:
        rospy.init_node('open_fridge_server')
        s = WalkerMovementUtils()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
