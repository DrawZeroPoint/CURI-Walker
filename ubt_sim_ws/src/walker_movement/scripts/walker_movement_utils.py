#!/usr/bin/env python
"""Utilities for solving control tasks."""

import rospy
import actionlib
import walker_movement.msg
from walker_srvs.srv import leg_motion_MetaFuncCtrl
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from walker_movement.srv import GetEePose
import moveit_commander
import sys

class WalkerMovementUtils:

    def __init__(self, disable_gait=False):

        self.moveEeLeftClient = actionlib.SimpleActionClient('/walker/move_helper_left_arm/move_to_ee_pose', walker_movement.msg.MoveToEePoseAction)
        self.moveJointsLeftClient = actionlib.SimpleActionClient('/walker/move_helper_left_arm/move_to_joint_pose', walker_movement.msg.MoveToJointPoseAction)
        self.graspLeftClient = actionlib.SimpleActionClient('/walker/hand_helper_left/grasp', walker_movement.msg.GraspAction)
        self.moveEeRightClient = actionlib.SimpleActionClient('/walker/move_helper_right_arm/move_to_ee_pose', walker_movement.msg.MoveToEePoseAction)
        self.moveJointsRightClient = actionlib.SimpleActionClient('/walker/move_helper_right_arm/move_to_joint_pose', walker_movement.msg.MoveToJointPoseAction)
        self.graspRightClient = actionlib.SimpleActionClient('/walker/hand_helper_right/grasp', walker_movement.msg.GraspAction)
        self.moveEeDualClient = actionlib.SimpleActionClient('/walker/dual_arm_control/move_to_ee_pose_mirrored', walker_movement.msg.DualArmMirroredEeMoveAction)
        self.moveJointsDualClient = actionlib.SimpleActionClient('/walker/dual_arm_control/move_to_joint_pose', walker_movement.msg.DualArmJointMoveAction)


        rospy.loginfo("Waiting for action servers...")
        self.moveEeLeftClient.wait_for_server()
        self.moveJointsLeftClient.wait_for_server()
        self.graspLeftClient.wait_for_server()
        self.moveEeRightClient.wait_for_server()
        self.moveJointsRightClient.wait_for_server()
        self.graspRightClient.wait_for_server()
        self.moveEeDualClient.wait_for_server()
        self.moveJointsDualClient.wait_for_server()
        rospy.loginfo("Action servers connected")

        if not disable_gait:
            rospy.wait_for_service('/Leg/TaskScheduler')
            self.legCommandService = rospy.ServiceProxy('/Leg/TaskScheduler', leg_motion_MetaFuncCtrl)
            rospy.loginfo("Leg service connected")
            self.nav_publisher = rospy.Publisher('/nav/cmd_vel_nav', Twist, queue_size=1)
            rospy.Subscriber("/Leg/walking_odom", Odometry, self.odomCallback)
            rospy.Subscriber("/Leg/StepNum", Int64, self.stepNumCallback)

        rospy.wait_for_service('/walker/move_helper_left_arm/get_ee_pose')
        self.getEePoseLeftService = rospy.ServiceProxy('/walker/move_helper_left_arm/get_ee_pose', GetEePose)
        rospy.wait_for_service('/walker/move_helper_right_arm/get_ee_pose')
        self.getEePoseRightService = rospy.ServiceProxy('/walker/move_helper_right_arm/get_ee_pose', GetEePose)

        rospy.loginfo("Subscibed to topics")

        self.lastOdomMsg=None
        self.gaitStepsPerfomed = 0
        self.collisionObjCounter = 0


        moveit_commander.roscpp_initialize(sys.argv)
        self.planningScene = moveit_commander.PlanningSceneInterface(ns="/walker")


    def buildPoseStamped(self,position_xyz, orientation_xyzw, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = position_xyz[0]
        pose.pose.position.y = position_xyz[1]
        pose.pose.position.z = position_xyz[2]
        pose.pose.orientation.x = orientation_xyzw[0]
        pose.pose.orientation.y = orientation_xyzw[1]
        pose.pose.orientation.z = orientation_xyzw[2]
        pose.pose.orientation.w = orientation_xyzw[3]
        return pose


    def getEePose(self, isLeft):
        if isLeft:
            s = self.getEePoseLeftService
            eel = "left_tcp"
        else:
            s = self.getEePoseRightService
            eel = "right_tcp"
        res = s(eel)
        return res.pose

    def moveToEePose(self,isLeft, position_xyz, orientation_xyzw, isRelative=False):
        rospy.loginfo("Moving to ee pose")
        if isLeft:
            c = self.moveEeLeftClient
            eel = "left_tcp"
        else:
            c = self.moveEeRightClient
            eel = "right_tcp"
        goal = walker_movement.msg.MoveToEePoseGoal()
        if isRelative:
            frame_id = eel
        else:
            frame_id = "base_link"
        goal.pose = self.buildPoseStamped(position_xyz,orientation_xyzw,frame_id)
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
        #print(c.get_result())
        rospy.loginfo("Grasped")

    def graspFridge(self, isLeft):
        rospy.loginfo("Grasping...")
        if isLeft:
            c = self.graspLeftClient
        else:
            c = self.graspRightClient
        c.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_FRIDGE_HANDLE))
        c.wait_for_result()
        #print(c.get_result())
        rospy.loginfo("Grasped")

    def releaseGrasp(self, isLeft):
        rospy.loginfo("Opening hand...")
        if isLeft:
            c = self.graspLeftClient
        else:
            c = self.graspRightClient
        c.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_OPEN))
        c.wait_for_result()
        #print(c.get_result())
        rospy.loginfo("Opened")

    def moveToHome(self, isLeft):
        rospy.loginfo("Moving to home pose")
        if isLeft:
            c = self.moveJointsLeftClient
        else:
            c = self.moveJointsRightClient
        c.send_goal(walker_movement.msg.MoveToJointPoseGoal([0, 0, 0, 0, 0, 0, 0]))
        c.wait_for_result()
        #print(c.get_result())
        rospy.loginfo("Moved")

    def graspCan(self, isLeft):
        rospy.loginfo("Grasping...")
        if isLeft:
            c = self.graspLeftClient
        else:
            c = self.graspRightClient
        c.send_goal(walker_movement.msg.GraspGoal(walker_movement.msg.GraspGoal.GRASP_TYPE_CAN))
        c.wait_for_result()
        #print(graspClient.get_result())
        rospy.loginfo("Grasped")


    def moveToJointDualArmSimmetrical(self,jointPoseLeft):
        t = [-1,1,-1,1,-1,-1,1]
        jointPoseRight = [t[i]*jointPoseLeft[i] for i in range(len(jointPoseLeft))]

        goal = walker_movement.msg.DualArmJointMoveGoal()
        goal.left_pose = jointPoseLeft
        goal.right_pose = jointPoseRight
        self.moveJointsDualClient.send_goal(goal)
        self.moveJointsDualClient.wait_for_result()


    def moveToEeDualArmMirrored(self,leftPosition, leftOrientation, isRelative=False, do_cartesian=False):
        rospy.loginfo("Dual moving to ee pose")

        eel_left = "left_tcp"
        eel_right = "right_tcp"
        goal = walker_movement.msg.DualArmEeMoveGoal()
        if isRelative:
            frame_id_left = eel_left
            frame_id_right = eel_right
        else:
            frame_id_left = "base_link"
            frame_id_right = "base_link"
        goal.left_pose = self.buildPoseStamped(leftPosition,leftOrientation,frame_id_left)
        goal.left_end_effector_link = eel_left
        goal.do_cartesian = do_cartesian
        self.moveEeDualClient.send_goal(goal)
        self.moveEeDualClient.wait_for_result()
        rospy.loginfo("Moved")


    def odomCallback(self,msg):
        """Receive the odometry message and publish the tf."""
        self.lastOdomMsg = msg


    def stepNumCallback(self,msg):
        """Receives the number of steps performed by the gait module."""
        self.gaitStepsPerfomed = msg.data
        #rospy.loginfo("gaitStepsPerfomed = "+str(gaitStepsPerfomed))

    def waitSteps(self,stepsNumber):
        """Wait for a certain number of steps to be performed."""
        step0 = self.gaitStepsPerfomed
        diff = self.gaitStepsPerfomed-step0
        prevDiff = diff-1
        while diff<stepsNumber:
            if diff!=prevDiff:
                rospy.loginfo("Waiting "+str(stepsNumber)+" steps, "+str(diff)+" performed")
            prevDiff = diff
            rospy.sleep(0.35) #half of the duration of a single step
            diff = self.gaitStepsPerfomed-step0

    def waitStepAbsolute(self,stepNumber):
        """Wait for the step counter to reach the provided value."""
        while self.gaitStepsPerfomed!=stepNumber:
            rospy.sleep(0.35) #half of the duration of a single step

    def moveGait(self,mode, amount, maxStep_constraint = None):
        if mode == "forward":
            maxStep = 0.32
        elif mode == "turn":
            maxStep = 0.35
        if (maxStep_constraint is not None) and (maxStep>maxStep_constraint):
            maxStep=maxStep_constraint

        stepsToDo = int(abs(amount) / maxStep) + 1 #0.35 is the maximum step length
        stepLength = float(amount)/stepsToDo
        headroom_sec = 0.1
        stepDuration_Sec = 0.7

        if mode == "forward":
            lin = Vector3(stepLength,0,0)
            ang = Vector3(0,0,0)
        elif mode == "turn":
            lin = Vector3(0,0,0)
            ang = Vector3(0,0,stepLength)

        rospy.loginfo("Will do "+str(stepsToDo)+" steps of "+str(stepLength)+" length")

        rospy.sleep(stepDuration_Sec + headroom_sec) #duration of one step
        rospy.loginfo("Moving")
        self.nav_publisher.publish(Twist(linear=lin,angular=ang))
        self.waitSteps(stepsToDo*2-1)
        rospy.loginfo("Stopping")
        self.nav_publisher.publish(Twist(linear=Vector3(0,0,0),angular=Vector3(0,0,0)))
        rospy.sleep(stepDuration_Sec + headroom_sec) #duration of one step, let it complete the last step

    def turnAround(self,angleZ):
        self.moveGait("turn",angleZ)

    def walkForward(self,length, maxStep=None):
        self.moveGait("forward",length, maxStep)


    def startGait(self):
        self.legCommandService("dynamic","","start")

    def stopGait(self):
        self.legCommandService("dynamic","","stop")
        rospy.sleep(1.4)


    def addCollisionBox(self, poseStamped, boxSize_xyz):
        box_pose = poseStamped
        objId = str(self.collisionObjCounter)
        self.collisionObjCounter+=1
        box_name = objId
        self.planningScene.add_box(box_name, box_pose, size=boxSize_xyz)

        start = rospy.get_time()
        while (rospy.get_time() - start < 10) and not rospy.is_shutdown():
            if box_name in self.planningScene.get_known_object_names():
                rospy.loginfo("Added CollisionObject")
                return objId #added
            rospy.sleep(0.1)

        rospy.loginfo("Failed to add CollisionObject")
        return None #failed

    def removeCollision(self, objectId):
        self.planningScene.remove_world_object(objectId)
