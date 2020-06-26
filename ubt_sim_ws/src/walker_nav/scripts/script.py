import rospy
from std_msgs.msg import String
from ubt_core_msgs.msg import *
from sensor_msgs.msg import JointState
rospy.init_node('talker', anonymous=True)

x = JointCommand()
x.mode = 7
x.command = [0,0.5]
msg=rospy.wait_for_message("/walker/head/joint_states", JointState)
print(msg)
pub = rospy.Publisher('/walker/head/controller', JointCommand, queue_size=10)
print("\n")
print(x)
for i in range(100):
    pub.publish(x)
    rospy.sleep(0.01)