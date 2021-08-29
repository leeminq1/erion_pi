#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray
import RPi.GPIO as GPIO
import time

cmd_vel = Int16MultiArray()
cmd_vel.data = [0, 0]


def callback(msg):
    cmd_vel.data[0] = int(msg.data[0])
    cmd_vel.data[1] = int(msg.data[1])
    if (cmd_vel.data[0] > 0) and (cmd_vel.data[1] == 0):
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("forward")
        print("----forward-----")

    elif (cmd_vel.data[0] < 0) and (cmd_vel.data[1] == 0):
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("backward")
        print("----backward-----")

    elif (cmd_vel.data[0] > 0) and (cmd_vel.data[1] > 0):
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("forward_right")
        print("----forward - right-----")

    elif (cmd_vel.data[0] > 0) and (cmd_vel.data[1] < 0):
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("forward_left")
        print("----forwad - left-----")

    elif (cmd_vel.data[0] < 0) and (cmd_vel.data[1] > 0):
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("backward_right")
        print("----backward - right-----")

    elif (cmd_vel.data[0] < 0) and (cmd_vel.data[1] < 0):
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("backward_left")
        print("----backward  left-----")

    elif (cmd_vel.data[0] == 0) and (cmd_vel.data[1] == 0):
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        print("--vehicle stop--")
        iMotor.stop()


class motor:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # pin setteing
        self.GPIO_right_RP = 23  # IN3
        self.GPIO_right_RN = 24  # IN4
        self.GPIO_right_EN = 12  # pwm

        self.GPIO_left_RP = 5  # IN1
        self.GPIO_left_RN = 6  # IN2
        self.GPIO_left_EN = 13  # pwm
        # right
        GPIO.setup(self.GPIO_right_RP, GPIO.OUT)
        GPIO.setup(self.GPIO_right_RN, GPIO.OUT)
        GPIO.setup(self.GPIO_right_EN, GPIO.OUT)
        self.pwm_right = GPIO.PWM(self.GPIO_right_EN, 50)
        self.pwm_right.start(50)

        # left
        GPIO.setup(self.GPIO_left_RP, GPIO.OUT)
        GPIO.setup(self.GPIO_left_RN, GPIO.OUT)
        GPIO.setup(self.GPIO_left_EN, GPIO.OUT)
        self.pwm_left = GPIO.PWM(self.GPIO_left_EN, 50)
        self.pwm_left.start(50)

        GPIO.output(self.GPIO_right_EN, False)
        GPIO.output(self.GPIO_left_EN, False)
        GPIO.output(self.GPIO_right_RP, False)
        GPIO.output(self.GPIO_right_RN, False)
        GPIO.output(self.GPIO_left_RP, False)
        GPIO.output(self.GPIO_left_RN, False)

    def __del__(self):
        GPIO.cleanup()

    def setDir(self, setV):
        if setV == 'forward':
            rospy.loginfo("forward")
            self.pwm_right.ChangeDutyCycle(80)
            self.pwm_left.ChangeDutyCycle(80)
            GPIO.output(self.GPIO_right_RP, True)
            GPIO.output(self.GPIO_right_RN, False)
            GPIO.output(self.GPIO_left_RP, True)
            GPIO.output(self.GPIO_left_RN, False)

        elif setV == 'backward':
            rospy.loginfo("backward")
            self.pwm_right.ChangeDutyCycle(80)
            self.pwm_left.ChangeDutyCycle(80)
            GPIO.output(self.GPIO_right_RP, False)
            GPIO.output(self.GPIO_right_RN, True)
            GPIO.output(self.GPIO_left_RP, False)
            GPIO.output(self.GPIO_left_RN, True)

        elif setV == 'forward_right':
            rospy.loginfo("forward-right")
            self.pwm_right.ChangeDutyCycle(30)
            self.pwm_left.ChangeDutyCycle(70)
            GPIO.output(self.GPIO_right_RP, True)
            GPIO.output(self.GPIO_right_RN, False)
            GPIO.output(self.GPIO_left_RP, True)
            GPIO.output(self.GPIO_left_RN, False)

        elif setV == 'forward_left':
            rospy.loginfo("forward_left")
            self.pwm_right.ChangeDutyCycle(70)
            self.pwm_left.ChangeDutyCycle(30)
            GPIO.output(self.GPIO_right_RP, True)
            GPIO.output(self.GPIO_right_RN, False)
            GPIO.output(self.GPIO_left_RP, True)
            GPIO.output(self.GPIO_left_RN, False)

        elif setV == 'backward_right':
            rospy.loginfo("backward_right")
            self.pwm_right.ChangeDutyCycle(30)
            self.pwm_left.ChangeDutyCycle(70)
            GPIO.output(self.GPIO_right_RP, False)
            GPIO.output(self.GPIO_right_RN, True)
            GPIO.output(self.GPIO_left_RP, False)
            GPIO.output(self.GPIO_left_RN, True)

        elif setV == 'backward_left':
            rospy.loginfo("backward_left")
            self.pwm_right.ChangeDutyCycle(70)
            self.pwm_left.ChangeDutyCycle(30)
            GPIO.output(self.GPIO_right_RP, False)
            GPIO.output(self.GPIO_right_RN, True)
            GPIO.output(self.GPIO_left_RP, False)
            GPIO.output(self.GPIO_left_RN, True)

        else:
            print("CCW setV : setDir('forward') ")
            print("CW setV : setDir('backword') ")

    def stop(self):
        GPIO.output(self.GPIO_right_EN, False)
        GPIO.output(self.GPIO_left_EN, False)
        GPIO.output(self.GPIO_right_RP, False)
        GPIO.output(self.GPIO_right_RN, False)
        GPIO.output(self.GPIO_left_RP, False)
        GPIO.output(self.GPIO_left_RN, False)

    def shortBreak(self):
        GPIO.output(self.GPIO_right_RP, False)
        GPIO.output(self.GPIO_right_RN, False)
        GPIO.output(self.GPIO_left_RP, False)
        GPIO.output(self.GPIO_left_RN, False)

    def start(self, setV):
        self.setDir(setV)


if __name__ == "__main__":
    try:
        rospy.init_node('rc_contorl', anonymous=True)
        rospy.loginfo("rc_car ready")
        rospy.Subscriber('cmd_vel', Int16MultiArray, callback)
        iMotor = motor()
        rospy.spin()
        # iMotor.start("forward")
        # time.sleep(1)

        # iMotor.start("right")
        # time.sleep(1)

        # iMotor.start("forward")
        # time.sleep(1)

        # iMotor.start("left")
        # time.sleep(1)

        # iMotor.start("backward")
        # time.sleep(1)
        # iMotor.stop()
    except KeyboardInterrupt:
        pass
    finally:
        del(iMotor)

