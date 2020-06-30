#!/usr/bin/env python

from __future__ import print_function

import rospy
import actionlib
import walker_movement.msg

import time
from walker_brain.srv import Dummy, DummyResponse
from walker_srvs.srv import leg_motion_MetaFuncCtrl
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int64, String
from nav_msgs.msg import Odometry


class WalkerMovementUtils(object):

    def __init__(self):
        super(WalkerMovementUtils, self).__init__()

        self._server = rospy.Service('push_cart', Dummy, self.handle)

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

        rospy.wait_for_service('/Leg/TaskScheduler')
        self.legCommandService = rospy.ServiceProxy('/Leg/TaskScheduler', leg_motion_MetaFuncCtrl)
        rospy.loginfo("Leg service connected")
        self.nav_publisher = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=1)

        rospy.Subscriber("/Leg/leg_status", String, self.statusCallback)
        rospy.Subscriber("/Leg/walking_odom", Odometry, self.odomCallback)
        rospy.Subscriber("/Leg/StepNum", Int64, self.stepNumCallback)

        self.lastOdomMsg = None
        self.gaitStepsPerformed = 0
        self._status = ""

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

    def odomCallback(self, msg):
        """Receive the odometry message and publish the tf."""
        self.lastOdomMsg = msg

    def statusCallback(self, msg):
        self._status = msg.data

    def stepNumCallback(self, msg):
        """Receives the number of steps performed by the gait module."""
        self.gaitStepsPerformed = msg.data
        # rospy.loginfo("gaitStepsPerfomed = "+str(gaitStepsPerfomed))

    def waitSteps(self, stepsNumber):
        """Wait for a certain number of steps to be performed."""
        step0 = self.gaitStepsPerformed
        diff = self.gaitStepsPerformed - step0
        prevDiff = diff - 1
        while diff < stepsNumber:
            if diff != prevDiff:
                rospy.loginfo("Waiting " + str(stepsNumber) + " steps, " + str(diff) + " performed")
            prevDiff = diff
            rospy.sleep(0.35)  # half of the duration of a single step
            diff = self.gaitStepsPerformed - step0

    def waitStepAbsolute(self, stepNumber):
        """Wait for the step counter to reach the provided value."""
        while self.gaitStepsPerformed != stepNumber:
            rospy.sleep(0.35)  # half of the duration of a single step

    def moveGait(self, mode, amount, maxStep_constraint=None):
        if mode == "forward":
            maxStep = 0.32
        elif mode == "turn":
            maxStep = 0.35
        if (maxStep_constraint is not None) and (maxStep > maxStep_constraint):
            maxStep = maxStep_constraint

        stepsToDo = int(abs(amount) / maxStep) + 1  # 0.35 is the maximum step length
        stepLength = float(amount) / stepsToDo
        headroom_sec = 0.1
        stepDuration_Sec = 0.7

        if mode == "forward":
            lin = Vector3(stepLength, 0, 0)
            ang = Vector3(0, 0, 0)
        elif mode == "turn":
            lin = Vector3(0, 0, 0)
            ang = Vector3(0, 0, stepLength)

        rospy.loginfo("Will do " + str(stepsToDo) + " steps of " + str(stepLength) + " length")

        rospy.sleep(stepDuration_Sec + headroom_sec)  # duration of one step
        rospy.loginfo("Moving")
        self.nav_publisher.publish(Twist(linear=lin, angular=ang))
        self.waitSteps(stepsToDo * 2 - 1)
        rospy.loginfo("Stopping")
        self.nav_publisher.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
        rospy.sleep(stepDuration_Sec + headroom_sec)  # duration of one step, let it complete the last step

    def turnAround(self, angleZ):
        self.moveGait("turn", angleZ)

    def walkForward(self, length, maxStep=None):
        self.moveGait("forward", length, maxStep)

    def startGait(self):
        while True:
            self.legCommandService("dynamic", "", "start")
            if self._status == "dynamic":
                break
            else:
                rospy.logwarn("Brain: Gait status is {} after calling start".format(self._status))

    def stopGait(self):
        self.legCommandService("dynamic", "", "stop")
        rospy.sleep(1.4)

    def handle(self, req):
        resp = DummyResponse()

        graspPositionLeft = [0.395, 0.174, -0.260]
        graspOrientationLeft = [0.258, 0.190, -0.514, 0.796]

        graspPositionRight = [0.395, -0.174, -0.260]
        graspOrientationRight = [-0.258, 0.190, 0.514, 0.796]

        # ensure hands are open
        self.releaseGrasp(True)
        self.releaseGrasp(False)

        intermediatePosR = [graspPositionRight[0], graspPositionRight[1] - 0.1, graspPositionRight[2] + 0.1]
        self.moveToEePose(False, intermediatePosR, graspOrientationRight)
        self.moveToEePose(False, graspPositionRight, graspOrientationRight)
        intermediatePosL = [graspPositionLeft[0], graspPositionLeft[1] + 0.1, graspPositionLeft[2] + 0.1]
        self.moveToEePose(True, intermediatePosL, graspOrientationLeft)
        self.moveToEePose(True, graspPositionLeft, graspOrientationLeft)
        rospy.sleep(0.25)

        self.graspCart(True)
        self.graspCart(False)
        rospy.sleep(0.25)

        self.startGait()
        rospy.loginfo("Started gait")
        rospy.sleep(0.5)
        rospy.loginfo("Moving forward")
        self.walkForward(1.5, 0.17)

        self.stopGait()
        rospy.loginfo("Stopped gait")
        rospy.sleep(0.5)

        resp.result_status = resp.SUCCEEDED
        return resp


if __name__ == "__main__":
    try:
        rospy.init_node('push_cart_server')
        s = WalkerMovementUtils()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
