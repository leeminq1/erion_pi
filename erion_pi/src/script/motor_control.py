#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16MultiArray

pub = rospy.Publisher('cmd_vel',Int16MultiArray,queue_size=10)
cmd_vel = Int16MultiArray()
cmd_vel.data = [0,0]

def callback1(msg):
    rospy.loginfo("aceel vel {} \t angular_vel :{}".format(msg.data[0],msg.data[1]))
    cmd_vel.data[0] = msg.data[0]
    cmd_vel.data[1] = msg.data[1]

    pub.publish(cmd_vel)

def motor_control():
    rospy.init_node('motor_control',anonymous=True)
    rospy.Subscriber("roscar_teleop_cmd_vel",Int16MultiArray,callback1)
    pub.publish(cmd_vel)
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_control()
    except rospy.ROSInterruptException:
        pass

