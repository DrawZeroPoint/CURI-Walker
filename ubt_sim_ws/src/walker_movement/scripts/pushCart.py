#!/usr/bin/env python
"""Push the cart wotihout vision or navigation."""

import rospy
from walker_movement_utils import WalkerMovementUtils

# graspPositionLeft = [-0.170, -0.310, 0.390]
# graspOrientationLeft = [-0.295, -0.139, 0.560, 0.761]
graspPositionLeft = [0.395, 0.180, -0.259]
graspOrientationLeft = [0.258, 0.190, -0.514, 0.796]


rospy.init_node('push_cart_py')

wmu = WalkerMovementUtils()


#ensure initial state
wmu.moveToJointDualArmSimmetrical([0,0,0,0,0,0,0])
wmu.releaseGrasp(True)
wmu.releaseGrasp(False)

intermediatePosL = [graspPositionLeft[0],graspPositionLeft[1]+0.1,graspPositionLeft[2]+0.05]
wmu.moveToEeDualArmMirrored(intermediatePosL,graspOrientationLeft,do_cartesian=False)
wmu.moveToEeDualArmMirrored(graspPositionLeft,graspOrientationLeft,do_cartesian=False)
rospy.sleep(0.25)

wmu.graspCart(True)
wmu.graspCart(False)
rospy.sleep(0.25)


wmu.startGait()
rospy.loginfo("Started gait")
rospy.sleep(0.5)
rospy.loginfo("Moving forward")
wmu.walkForward(1.5, 0.17)

wmu.stopGait()
rospy.loginfo("Stopped gait")
rospy.sleep(0.5)

# wmu.releaseGrasp(True)
# wmu.releaseGrasp(False)
# time.sleep(0.25)
#
# wmu.moveToEePose(False,intermediatePosR,graspOrientationRight)
# wmu.moveToEePose(True,intermediatePosL,graspOrientationLeft)
# wmu.moveToHome(True)
# wmu.moveToHome(False)

rospy.loginfo("Finished")
