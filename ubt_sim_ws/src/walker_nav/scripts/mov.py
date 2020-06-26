from tf.transformations import *
from geometry_msgs.msg import Pose
import walker_nav.srv
import rospy

def pose_trolley():
    pose = Pose()

    pose.position.x = -1.80
    pose.position.y = 3.35
    pose.position.z = -0.02

    pose.orientation.x = -0.00468332031371
    pose.orientation.y = -0.00216545852061
    pose.orientation.z = -0.644841606148
    pose.orientation.w = 0.764298816092

    return pose

def pose_can_table():
    pose = Pose()

    pose.position.x = 3.25
    pose.position.y = 1.60
    pose.position.z = 0.00

    pose.orientation.x = -0.00795002334806
    pose.orientation.y = 0.00127581680056
    pose.orientation.z = -0.0121379918341
    pose.orientation.w = 0.99989391366

    return pose

def pose_fridge():
    pose = Pose()

    pose.position.x = -0.70
    pose.position.y = 8.40
    pose.position.z = 0.02

    pose.orientation.x = -0.0110687064314
    pose.orientation.y = 0.0074370792366
    pose.orientation.z = 0.703028381245
    pose.orientation.w = 0.711036756262
    return pose


input_val = int(raw_input("Trolley: 0 | Can Table: 1 | Fridge: 2\n >"))
pose = []
if input_val == 0:
    pose = pose_trolley()
elif input_val == 1:
    pose = pose_can_table()
elif input_val == 2:
    pose = pose_fridge()

rospy.init_node("move_node")

move_service = rospy.ServiceProxy('/move_base/move_to_abs_pos', walker_nav.srv.MoveToAbsPos)
resp = move_service(pose)
print(resp)
