#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray

#
import RPi.GPIO as GPIO
import time


class lift_mode:
    def __init__(self):
        self.time_detectd = 0.0
        self.mode_sel = 0
        self.lft_md_sel = 0
        rospy.Subscriber('mode_sel_cmd_vel', Int16MultiArray, self.callback)

    # time update chk

    @property
    def is_received(self):
        return(time.time() - self.time_detectd < 1.0)

    @property
    def mode_reset(self):
        return(time.time() - self.time_detectd > 3.0)

    def mode_by_clock(self, get_time):
        if self.time_detectd > 2.0:
            # mode cmd
            # 1: iw_motor_drive / 2: rack_bldc_drive / 3: whl_bldc_drive
            # time cal
            INWEEL_TIME_END = 6
            RACK_TIME_START = 7
            RACK_TIME_END = 13
            WHL_TIME_START = RACK_TIME_END
            WHL_TIME_END = 21
            MODE_TIME_END = 22

            f_iw_mode = get_time < INWEEL_TIME_END
            f_rack_mode = (
                get_time < RACK_TIME_END and get_time > RACK_TIME_START)
            f_whl_mode = (get_time < WHL_TIME_END and get_time >
                          WHL_TIME_START)
            f_none_mode = get_time > MODE_TIME_END or self.mode_reset

            if f_iw_mode:
                self.lft_md_sel = 1
            elif f_rack_mode:
                self.lft_md_sel = 2
            elif f_whl_mode:
                self.lft_md_sel = 3
            elif f_none_mode:
                rospy.loginfo("No Condition lift_mode")
                self.lft_md_sel = 0
        return self.lft_md_sel

    def callback(self, msg):
        self.time_detectd = time.time()
        f_lift_mode = msg.data[2] == 3
        get_time = msg.data[3]
        if f_lift_mode and self.is_received:
            self.mode_sel = self.mode_by_clock(get_time)
            if self.mode_sel == 1:
                iw_mot.run()
            elif self.mode_sel == 2:
                rack_bldc_mot.run()
            elif self.mode_sel == 3:
                whl_bldc_mot.run()
        else:
            print("else work")
            iw_mot.stop()
            rack_bldc_mot.stop()
            whl_bldc_mot.stop()
            self.time_detectd = 0.0


class Iw_motor:
    def __init__(self):
        # pin setteing
        self.pwmLPin = 12  # Jetson Nano PIN 32
        self.pwmRPin = 13
        self.dirLPin = 6  # Jetson Nano PIN 31
        self.dirRPin = 5
        self.brkLPin = 4
        self.brkRPin = 17

        # GPIO SET
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.pwmLPin, self.pwmRPin,
                   self.dirLPin, self.dirRPin, self.brkLPin, self.brkRPin], GPIO.OUT)

        self.pL = GPIO.PWM(self.pwmLPin, 100)  # 12pin , strength 20%
        self.pL.start(0)

        self.pR = GPIO.PWM(self.pwmRPin, 100)  # 12pin , strength 20%
        self.pR.start(0)

    def run(self):
        rospy.loginfo("IW_Motor_drive")
        self.pR.ChangeDutyCycle(50)
        self.pL.ChangeDutyCycle(50)
        GPIO.output(self.dirLPin, True)
        GPIO.output(self.dirRPin, False)
        GPIO.output(self.brkLPin, False)
        GPIO.output(self.brkRPin, False)

    def __del__(self):
        GPIO.cleanup()

    def stop(self):
        rospy.loginfo("IW_Motor_stop")
        self.pR.ChangeDutyCycle(0)
        self.pL.ChangeDutyCycle(0)
        # chk direction  / stop pin
        GPIO.output(self.brkLPin, False)
        GPIO.output(self.brkRPin, False)


class Bldc_motor:
    def __init__(self, name):
        self.bldc_name = name
        if self.bldc_name == "rack":
            # rack bldc
            self.pwmPin = 18
            self.dirPin = 27
            self.brkPin = 22
            print("rack create")

        elif self.bldc_name == "whl":
            # whl pin
            self.pwmPin = 19
            self.dirPin = 8
            self.brkPin = 7
            print("whl create")

        # GPIO SET
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.pwmPin, self.dirPin, self.brkPin], GPIO.OUT)
        self.pwmPin = GPIO.PWM(self.pwmPin, 100)
        self.pwmPin.start(0)

    def run(self):
        rospy.loginfo("{} drive.".format(self.bldc_name))
        self.pwmPin.ChangeDutyCycle(100)
        GPIO.output(self.dirPin, True)
        GPIO.output(self.brkPin, False)

    def __del__(self):
        GPIO.cleanup()

    def stop(self):
        rospy.loginfo("{} stop.".format(self.bldc_name))
        self.pwmPin.ChangeDutyCycle(0)
        # chk direction  / stop pin
        GPIO.output(self.brkPin, False)


if __name__ == "__main__":
    try:
        rospy.init_node('lift_mode', anonymous=True)
        rospy.loginfo("erion_lift_mode")
        iw_mot = Iw_motor()
        rack_bldc_mot = Bldc_motor("rack")
        whl_bldc_mot = Bldc_motor("whl")
        lift = lift_mode()
        rospy.spin()

    except KeyboardInterrupt:
        pass
    # finally:
    #     del(iw_mot)
    #     del(rack_bldc_mot)
    #     del(whl_bldc_mot)

