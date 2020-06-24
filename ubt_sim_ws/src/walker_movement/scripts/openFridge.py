#!/usr/bin/env python
"""Open the fridge without vision or navigation."""

import rospy
from geometry_msgs.msg import Twist, Vector3
from walker_movement_utils import WalkerMovementUtils


fridgeHandleJointPose_fromInitial = [-0.6471997844669032, -0.8026922328704162, 1.0668816324134622, -1.8211421279343931, -0.36460361415405457, 0.12372186529575947, 0.3866315963032178]
fridgeHandlePosition_fromInitial = [0.312, -0.181, -0.014]
fridgeHandleOrientation_fromInitial = [-0.035, -0.064, 0.684, 0.726]
#handlePrePosition_fromBack = [0.405, 0.331, 0.025]
handlePrePosition_fromBack = [0.344, 0.331, 0.008]
handlePreOrientation_fromBack = [-0.018, 0.041, -0.426, 0.904]

# rightPosition = [0.438, 0.004, -0.003]
# rightOrientation = [-0.006, -0.001, 0.645, 0.765]
# rightPosition = [0.467, -0.063, -0.005]
# rightOrientation = [0.012, 0.043, 0.631, 0.774]
rightPosition = [0.419, -0.061, -0.008]
rightOrientation = [0.020, 0.049, 0.704, 0.708]
rightPosition2 = [0.428, -0.002, 0.002]
rightOrientation2 = [0.020, 0.052, 0.690, 0.722]

leftPosition1 = [0.368, 0.074, -0.090]
leftOrientation1 = [0.095, -0.138, -0.451, 0.877]
leftPositionF = [0.332, -0.066, -0.114]
leftOrientationF = [0.125, -0.102, -0.664, 0.730]


rospy.init_node('open_fridge_py')

wmu = WalkerMovementUtils()


wmu.nav_publisher.publish(Twist(linear=Vector3(0,0,0),angular=Vector3(0,0,0)))#ensure no dangling messages
wmu.startGait()
wmu.turnAround(-0.785)
rospy.loginfo("Moved of "+str(wmu.lastOdomMsg.pose.pose))
wmu.walkForward(-0.10)
rospy.loginfo("Moved of "+str(wmu.lastOdomMsg.pose.pose))
wmu.stopGait()

wmu.moveToEePose(False,rightPosition,rightOrientation)
wmu.moveToEePose(False,rightPosition2,rightOrientation2)
wmu.graspCan(False)
wmu.moveToEePose(False,[-0.15,0.15,0],[0,0,0,1], True)
wmu.moveToEePose(False,[-0.01,-0.01,0],[0,0,0.087,0.996], True)

wmu.moveToEePose(True, leftPosition1, leftOrientation1)

wmu.releaseGrasp(False)
wmu.moveToHome(False)


wmu.moveToEePose(True, (0.1,0.05,0), (0,0,0,1), True)
wmu.moveToEePose(True, leftPositionF, leftOrientationF, False)
