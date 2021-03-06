#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray

import RPi.GPIO as GPIO
import time

cmd_vel = Int16MultiArray()
cmd_vel.data = [0, 0, 0]

global time_detect
time_detect=0.0

@property
def is_detected():
    return (time.time()-time_detect < 1.0)


def callback(msg):
    cmd_vel.data[0] = int(msg.data[0])
    cmd_vel.data[1] = int(msg.data[1])
    cmd_vel.data[2] = int(msg.data[2])

    #Time update
    time_detect=time.time()
    print(time_detect)
    # drive deteciton
    f_forward = (cmd_vel.data[0] > 0) and (cmd_vel.data[1] == 0)
    f_backward = (cmd_vel.data[0] < 0) and (cmd_vel.data[1] == 0)
    f_forward_right = (cmd_vel.data[0] > 0) and (cmd_vel.data[1] > 0)
    f_forward_left = (cmd_vel.data[0] > 0) and (cmd_vel.data[1] < 0)
    f_backward_right = (cmd_vel.data[0] < 0) and (cmd_vel.data[1] > 0)
    f_backward_left = (cmd_vel.data[0] < 0) and (cmd_vel.data[1] < 0)
    f_vehicle_stop = (cmd_vel.data[0] == 0) and (cmd_vel.data[1] == 0)
    f_left_spin = (cmd_vel.data[0] == 0) and (cmd_vel.data[1] < 0)
    f_right_spin = (cmd_vel.data[0] == 0) and (cmd_vel.data[1] > 0)

    # keyboard control
    if f_forward:
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("forward")
        print("----keyboard forward-----")

    elif f_backward:
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("backward")
        print("----keyboard backward-----")

    elif f_forward_right:
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("forward_right")
        print("----keyboard forward - right-----")

    elif f_forward_left:
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("forward_left")
        print("----keyboard forwad - left-----")

    elif f_backward_right:
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("backward_right")
        print("----keyboard backward - right-----")

    elif f_backward_left:
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        iMotor.start("backward_left")
        print("----keyboard backward  left-----")

    elif f_vehicle_stop:
        # rospy.loginfo("accel : {} \t angular: {}".format(
        #     msg.data[0], msg.data[1]))
        print("--keyboard vehicle stop--")
        iMotor.stop()

    elif f_left_spin:
        # rospy.loginfo("accel : {} \t angular: {}".format(
        # msg.data[0], msg.data[1]))
        iMotor.start("left_spin")
        print("----keyboard vehicle left_spin-----")

    elif f_right_spin:
        # rospy.loginfo("accel : {} \t angular: {}".format(
        # msg.data[0], msg.data[1]))
        iMotor.start("right_spin")
        print("----keyboard vehicle right_spin-----")

    # # app control
    # elif int(msg.data[2]) == 2:
    #       input_value=get_firebase_input_value()
    #     if input_value=="go":
    #         iMotor.start("forward")
    #     elif input_value=="back":
    #         iMotor.start("backward")
    #     elif input_value=="left":


class motor:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # pin setteing
        self.GPIO_right_RP = 20  # IN3
        self.GPIO_right_RN = 21  # IN4
        self.GPIO_right_EN = 12  # pwm

        self.GPIO_left_RP = 5  # IN1
        self.GPIO_left_RN = 6  # IN2
        self.GPIO_left_EN = 13  # pwm
        # right
        GPIO.setup(self.GPIO_right_RP, GPIO.OUT)
        GPIO.setup(self.GPIO_right_RN, GPIO.OUT)
        GPIO.setup(self.GPIO_right_EN, GPIO.OUT)
        self.pwm_right = GPIO.PWM(self.GPIO_right_EN, 10)
        self.pwm_right.start(100)

        # left
        GPIO.setup(self.GPIO_left_RP, GPIO.OUT)
        GPIO.setup(self.GPIO_left_RN, GPIO.OUT)
        GPIO.setup(self.GPIO_left_EN, GPIO.OUT)
        self.pwm_left = GPIO.PWM(self.GPIO_left_EN, 10)
        self.pwm_left.start(100)

        GPIO.output(self.GPIO_right_EN, False)
        GPIO.output(self.GPIO_left_EN, False)
        GPIO.output(self.GPIO_right_RP, False)
        GPIO.output(self.GPIO_right_RN, False)
        GPIO.output(self.GPIO_left_RP, False)
        GPIO.output(self.GPIO_left_RN, False)


    def __del__(self):
        GPIO.cleanup()

    def setDir(self, setV):
        if setV == 'forward' and is_detected:
            rospy.loginfo("forward")
            self.pwm_right.ChangeDutyCycle(80)
            self.pwm_left.ChangeDutyCycle(80)
            GPIO.output(self.GPIO_right_RP, True)
            GPIO.output(self.GPIO_right_RN, False)
            GPIO.output(self.GPIO_left_RP, True)
            GPIO.output(self.GPIO_left_RN, False)

        elif setV == 'backward' and is_detected:
            rospy.loginfo("backward")
            self.pwm_right.ChangeDutyCycle(80)
            self.pwm_left.ChangeDutyCycle(80)
            GPIO.output(self.GPIO_right_RP, False)
            GPIO.output(self.GPIO_right_RN, True)
            GPIO.output(self.GPIO_left_RP, False)
            GPIO.output(self.GPIO_left_RN, True)

        elif setV == 'forward_right' and is_detected:
            rospy.loginfo("forward-right")
            self.pwm_right.ChangeDutyCycle(20)
            self.pwm_left.ChangeDutyCycle(80)
            GPIO.output(self.GPIO_right_RP, True)
            GPIO.output(self.GPIO_right_RN, False)
            GPIO.output(self.GPIO_left_RP, True)
            GPIO.output(self.GPIO_left_RN, False)

        elif setV == 'forward_left' and is_detected:
            rospy.loginfo("forward_left")
            self.pwm_right.ChangeDutyCycle(80)
            self.pwm_left.ChangeDutyCycle(20)
            GPIO.output(self.GPIO_right_RP, True)
            GPIO.output(self.GPIO_right_RN, False)
            GPIO.output(self.GPIO_left_RP, True)
            GPIO.output(self.GPIO_left_RN, False)

        elif setV == 'backward_right' and is_detected:
            rospy.loginfo("backward_right")
            self.pwm_right.ChangeDutyCycle(20)
            self.pwm_left.ChangeDutyCycle(40)
            GPIO.output(self.GPIO_right_RP, False)
            GPIO.output(self.GPIO_right_RN, True)
            GPIO.output(self.GPIO_left_RP, False)
            GPIO.output(self.GPIO_left_RN, True)

        elif setV == 'backward_left' and is_detected:
            rospy.loginfo("backward_left")
            self.pwm_right.ChangeDutyCycle(40)
            self.pwm_left.ChangeDutyCycle(20)
            GPIO.output(self.GPIO_right_RP, False)
            GPIO.output(self.GPIO_right_RN, True)
            GPIO.output(self.GPIO_left_RP, False)
            GPIO.output(self.GPIO_left_RN, True)

        elif setV == 'left_spin' and is_detected:
            rospy.loginfo("left_spin")
            self.pwm_right.ChangeDutyCycle(80)
            self.pwm_left.ChangeDutyCycle(0)
            GPIO.output(self.GPIO_right_RP, True)
            GPIO.output(self.GPIO_right_RN, False)
            GPIO.output(self.GPIO_left_RP, False)
            GPIO.output(self.GPIO_left_RN, True)

        elif setV == 'right_spin' and is_detected:
            rospy.loginfo("right_spin")
            self.pwm_right.ChangeDutyCycle(0)
            self.pwm_left.ChangeDutyCycle(80)
            GPIO.output(self.GPIO_right_RP, False)
            GPIO.output(self.GPIO_right_RN, True)
            GPIO.output(self.GPIO_left_RP, True)
            GPIO.output(self.GPIO_left_RN, False)

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
        rospy.init_node('rc_dc_auto_motor', anonymous=True)
        rospy.loginfo("rc_car_auto_mode ready")
        rospy.Subscriber('/auto_drive_control/cmd_vel',
                         Int16MultiArray, callback)
        iMotor=motor()
        rospy.spin()

    except KeyboardInterrupt:
        pass
    finally:
        pass
