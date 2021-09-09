#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16MultiArray

pub = rospy.Publisher('mode_sel_cmd_vel', Int16MultiArray, queue_size=10)
cmd_vel = Int16MultiArray()
cmd_vel.data = [0, 0, 0]


def callback1(msg):

    # auto drive
    if msg.data[2] == 0:
        rospy.loginfo("autodrive_mode")
    elif msg.data[2] == 1:
        cmd_vel.data[0] = msg.data[0]
        cmd_vel.data[1] = msg.data[1]
        cmd_vel.data[1] = msg.data[2]
        pub.publish(cmd_vel)
        rospy.loginfo("aceel vel {} \t angular_vel :{} \t operate_mode : Key_board_mode".format(
            msg.data[0], msg.data[1]))
    elif msg.data[2] == 2:
        rospy.loginfo("app_control_mode")
    elif msg.data[2] == 3:
        rospy.loginfo("lift_mode")


def motor_control():
    rospy.init_node('mode_sel_motor_cmd', anonymous=True)
    rospy.Subscriber("roscar_teleop_cmd_vel", Int16MultiArray, callback1)
    rospy.spin()


if __name__ == '__main__':
    try:
        motor_control()
    except rospy.ROSInterruptException:
        pass
