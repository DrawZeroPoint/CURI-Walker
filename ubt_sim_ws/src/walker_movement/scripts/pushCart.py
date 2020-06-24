#!/usr/bin/env python
"""Push the cart wotihout vision or navigation."""

import rospy
from walker_movement_utils import WalkerMovementUtils

# graspPositionLeft = [-0.170, -0.310, 0.390]
# graspOrientationLeft = [-0.295, -0.139, 0.560, 0.761]
graspPositionLeft = [0.395, 0.174, -0.260]
graspOrientationLeft = [0.258, 0.190, -0.514, 0.796]

# graspPositionRight = [-0.167, 0.297, 0.392]
# graspOrientationRight = [0.294, -0.136, -0.560, 0.763]
graspPositionRight = [0.395, -0.174, -0.260]
graspOrientationRight = [-0.258, 0.190, 0.514, 0.796]



rospy.init_node('push_cart_py')

wmu = WalkerMovementUtils()


#ensure hands are open
wmu.releaseGrasp(True)
wmu.releaseGrasp(False)

intermediatePosR = [graspPositionRight[0],graspPositionRight[1]-0.1,graspPositionRight[2]+0.1]
wmu.moveToEePose(False,intermediatePosR,graspOrientationRight)
wmu.moveToEePose(False,graspPositionRight,graspOrientationRight)
intermediatePosL = [graspPositionLeft[0],graspPositionLeft[1]+0.1,graspPositionLeft[2]+0.1]
wmu.moveToEePose(True,intermediatePosL,graspOrientationLeft)
wmu.moveToEePose(True,graspPositionLeft,graspOrientationLeft)
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
