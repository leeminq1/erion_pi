#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray

#
import RPi.GPIO as IO
import time

# firebase
from firebase_admin import firestore
from firebase_admin import credentials
import firebase_admin


import RPi.GPIO as GPIO
import time

cmd_vel = Int16MultiArray()
cmd_vel.data = [0, 0, 0]


# firebase func
# get operating mode info
def get_firebase_input_value():
    db = firestore.client()
    doc_ref = db.collection(u'pi').document(u'key')
    doc = doc_ref.get()
    input_value = doc.to_dict()['input']
    # print("check the initial_value")
    # print(f'Document data: {input_value}')
    return input_value


def callback(msg):
    print("get msg")
    cmd_vel.data[0] = int(msg.data[0])
    cmd_vel.data[1] = int(msg.data[1])
    cmd_vel.data[2] = int(msg.data[2])

   # input_value = get_firebase_input_value()
    input_value = "None"
    # print(input_value)

    # mode detection
    f_autodrvie = cmd_vel.data[2] == 0
    f_keyboard = cmd_vel.data[2] == 1
    f_app_control = cmd_vel.data[2] == 2
    f_lift_mode = cmd_vel.data[2] == 3
    if f_autodrvie:
        print("autodrive mode")
    elif f_keyboard:
        print("keyboard mode")
    elif f_app_control:
        print("appcontrol mode")
    elif f_lift_mode:
        print("lift mode")

    # time
    global time_detected
    time_detected = time.time()

    # drive deteciton
    f_forward = (cmd_vel.data[0] > 0) and (cmd_vel.data[1] == 0)
    #f_backward = (cmd_vel.data[0] < 0) and (cmd_vel.data[1] == 0)

    f_forward_right = (cmd_vel.data[0] > 0) and (cmd_vel.data[1] > 0)
    f_forward_left = (cmd_vel.data[0] > 0) and (cmd_vel.data[1] < 0)

    f_right_spin = (cmd_vel.data[0] == 0) and (cmd_vel.data[1] > 0)
    f_left_spin = (cmd_vel.data[0] == 0) and (cmd_vel.data[1] < 0)

    f_vehicle_stop = (cmd_vel.data[0] == 0) and (cmd_vel.data[1] == 0)

    # keyboard control
    if f_autodrvie:
        if f_forward:
            # rospy.loginfo("accel : {} \t angular: {}".format(
            #     msg.data[0], msg.data[1]))
            iMotor.start("forward", cmd_vel.data[0], cmd_vel.data[1])

        elif f_forward_right:
            # rospy.loginfo("accel : {} \t anlsgular: {}".format(
            #     msg.data[0], msg.data[1]))
            iMotor.start("forward_right", cmd_vel.data[0], cmd_vel.data[1])

        elif f_forward_left:
            # rospy.loginfo("accel : {} \t angular: {}".format(
            #     msg.data[0], msg.data[1]))
            iMotor.start("forward_left", cmd_vel.data[0], cmd_vel.data[1])

        elif f_right_spin:
            # rospy.loginfo("accel : {} \t angular: {}".format(
            #     msg.data[0], msg.data[1]))
            iMotor.start("right_spin", cmd_vel.data[0], cmd_vel.data[1])

        elif f_left_spin:
            # rospy.loginfo("accel : {} \t angular: {}".format(
            #     msg.data[0], msg.data[1]))
            iMotor.start("left_spin", cmd_vel.data[0], cmd_vel.data[1])

        elif f_vehicle_stop:
            # rospy.loginfo("accel : {} \t angular: {}".format(
            #     msg.data[0], msg.data[1]))
            iMotor.stop()

    elif f_app_control:
        print("--app control mode--")
        iMotor.stop()
    elif f_keyboard:
        print("--key board mode--")
        iMotor.stop()
    elif f_lift_mode:
        print("--lift mode--")
        iMotor.stop()


class motor:
    def __init__(self):
        # pin setteing
        self.pwmLPin = 12  # Jetson Nano PIN 32
        self.pwmRPin = 13
        self.dirLPin = 6  # Jetson Nano PIN 31
        self.dirRPin = 5
        self.brkLPin = 4
        self.brkRPin = 17

        self.encLPinA = 23  # Jetson Nano PIN 16
        self.encLPinB = 24  # Jetson Nano PIN 18
        self.encRPinA = 19
        self.encRPinB = 26

        # GPIO SET
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.encLPinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encLPinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encRPinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encRPinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup([self.pwmLPin, self.pwmRPin,
                   self.dirLPin, self.dirRPin, self.brkLPin, self.brkRPin], GPIO.OUT)

        self.pL = GPIO.PWM(self.pwmLPin, 1000)  # 12pin , strength 20%
        self.pL.start(0)

        self.pR = GPIO.PWM(self.pwmRPin, 1000)  # 12pin , strength 20%
        self.pR.start(0)

        self.encoderLPos = 0
        self.encoderLPos_prev = 0
        self.encoderRPos = 0
        self.encoderRPos_prev = 0
        self.pwmR = 0
        self.pwmL = 0

        GPIO.add_event_detect(self.encLPinA, GPIO.BOTH,
                              callback=self.encoderLA)
        GPIO.add_event_detect(self.encLPinB, GPIO.BOTH,
                              callback=self.encoderLB)

        GPIO.add_event_detect(self.encRPinA, GPIO.BOTH,
                              callback=self.encoderRA)
        GPIO.add_event_detect(self.encRPinB, GPIO.BOTH,
                              callback=self.encoderRB)

    def encoderLA(self, channel):
        if GPIO.input(self.encLPinA) == GPIO.input(self.encLPinB):
            self.encoderLPos += 1
        else:
            self.encoderLPos -= 1
        #print('PinA: %d, encoder: %d' %(channel, encoderPos))

    def encoderLB(self, channel):
        if GPIO.input(self.encLPinA) == GPIO.input(self.encLPinB):
            self.encoderLPos -= 1
        else:
            self.encoderLPos += 1
        #print('PinB: %d, encoder: %d' %(channel, encoderPos))

    def encoderRA(self, channel):
        if GPIO.input(self.encRPinA) == GPIO.input(self.encRPinB):
            self.encoderRPos += 1
        else:
            self.encoderRPos -= 1
        #print('PinA: %d, encoder: %d' %(channel, encoderPos))

    def encoderRB(self, channel):
        if GPIO.input(self.encRPinA) == GPIO.input(self.encRPinB):
            self.encoderRPos -= 1
        else:
            self.encoderRPos += 1
        #print('PinB: %d, encoder: %d' %(channel, encoderPos))

    def __del__(self):
        GPIO.cleanup()

    @property
    def is_detected(self):
        return(time.time() - time_detected < 1.0)

    def setDir(self, setV, accel, steer):
        if setV == 'forward' and self.is_detected:
            rospy.loginfo("forward")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm(accel, steer)
            # cmd oper
            self.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            self.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(self.dirLPin, True)
            GPIO.output(self.dirRPin, False)
            GPIO.output(self.brkLPin, False)
            GPIO.output(self.brkRPin, False)

        elif setV == 'forward_right' and self.is_detected:
            rospy.loginfo("forward-right")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm(accel, steer)
            # cmd oper
            self.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            self.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(self.dirLPin, True)
            GPIO.output(self.dirRPin, False)
            GPIO.output(self.brkLPin, False)
            GPIO.output(self.brkRPin, False)

        elif setV == 'forward_left' and self.is_detected:
            rospy.loginfo("forward_left")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm(accel, steer)
            # cmd oper
            self.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            self.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(self.dirLPin, True)
            GPIO.output(self.dirRPin, False)
            GPIO.output(self.brkLPin, False)
            GPIO.output(self.brkRPin, False)

        elif setV == 'right_spin' and self.is_detected:
            rospy.loginfo("right_spin")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm(accel, steer)
            # cmd oper
            self.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            self.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(self.dirLPin, True)
            GPIO.output(self.dirRPin, True)
            GPIO.output(self.brkLPin, False)
            GPIO.output(self.brkRPin, False)

        elif setV == 'left_spin' and self.is_detected:
            rospy.loginfo("left_spin")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm(accel, steer)
            # cmd oper
            self.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            self.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(self.dirLPin, False)
            GPIO.output(self.dirRPin, False)
            GPIO.output(self.brkLPin, False)
            GPIO.output(self.brkRPin, False)

        else:
            print("CCW setV : setDir('forward') ")
            print("CW setV : setDir('backword') ")

    def cal_pwm(self, accel, steer):
        # target basic value
        target = 2
        # spd reset
        # self.encoderLPos=0
        # self.encoderRPos=0

        # PDI Gain
        ratio = 360/4096/3
        Kp = 4
        Kd = 0
        Ki = 0
        dt = 0
        dt_sleep = 100
        tolerance = 1
        errL_prev = 0
        errR_prev = 0
        time_prev = 0
        # self.encoderLPos_prev=0
        # self.encoderRPos_prev=0

        # target cal
        # forward right
        if accel > 0 and steer > 0:
            self.target_left_motor = accel*target
            self.target_right_motor = accel*0.3*target
        # forward left
        elif accel > 0 and steer < 0:
            self.target_left_motor = accel*0.3*target
            self.target_right_motor = accel*target
        # backward right
        elif accel < 0 and steer > 0:
            self.target_left_motor = accel*target
            self.target_right_motor = accel*0.3*target
        # backward left
        elif accel < 0 and steer > 0:
            self.target_left_motor = accel*0.3*target
            self.target_right_motor = accel*target
        else:
            self.target_left_motor = accel*target
            self.target_right_motor = accel*target

        # left
        self.motorLDeg = self.encoderLPos * ratio
        self.encoderLPos = abs(self.encoderLPos)
        self.spdLrpm = (self.encoderLPos-self.encoderLPos_prev) * \
            60 * 1/dt_sleep * 1/4096
        self.encoderLPos_prev = self.encoderLPos
        self.errL = abs(self.target_left_motor)-abs(self.spdLrpm)
        self.derrL = self.errL - errL_prev
        self.dtL = time.time() - time_prev
        self.controlL = Kp*self.errL + Kd*self.derrL/self.dtL + Ki*self.errL*self.dtL

        # right
        self.motorRDeg = self.encoderRPos * ratio
        self.encoderRPos = abs(self.encoderRPos)
        self.spdRrpm = (self.encoderRPos-self.encoderRPos_prev) * \
            60 * 1/dt_sleep * 1/4096
        self.encoderRPos_prev = self.encoderRPos
        self.errR = abs(self.target_right_motor)-abs(self.spdRrpm)
        self.derrR = self.errR - errR_prev
        self.dtR = time.time() - time_prev
        self.controlR = Kp*self.errR + Kd*self.derrR/self.dtR + Ki*self.errR*self.dtR
        print("Tar_L_rpm : {}, Tar_R_rpm :{}, L_rpm : {}, R_rpm : {}, err_L:{}, err_R : {}, control_L : {}, control_R :{}".format(
            self.target_left_motor, self.target_right_motor, self.spdLrpm, self.spdRrpm, self.errL, self.errR, self.controlL, self.controlR))
        print("L_encoder_delta:{}, R_encoder_delta:{}".format(
            self.encoderLPos-self.encoderLPos_prev, self.encoderRPos-self.encoderRPos_prev))
        return self.controlL, self.controlR

    def stop(self):
        if self.is_detected:
            print("stop")
           # self.pR.ChangeDutyCycle(0)
           # self.pL.ChangeDutyCycle(0)
            self.encoderLPos = 0
            self.encoderRPos = 0
            self.encoderLPos_prev = 0
            self.encoderRPos_prev = 0
            # chk direction  / stop pin
            GPIO.output(self.brkLPin, True)
            GPIO.output(self.brkRPin, True)

    def shortBreak(self):
        GPIO.output(self.GPIO_right_RP, False)
        GPIO.output(self.GPIO_right_RN, False)
        GPIO.output(self.GPIO_left_RP, False)
        GPIO.output(self.GPIO_left_RN, False)

    def start(self, setV, accel, steer):
        self.setDir(setV, accel, steer)


if __name__ == "__main__":
    try:
        # firebase init
        cred = credentials.Certificate(
            "/home/ubuntu/catkin_ws/src/erion_pi/src/script/erion_key.json")
        firebase_admin.initialize_app(cred)
        print("firebase certificated")

        rospy.init_node('erion_auto_mode', anonymous=True)
        rospy.loginfo("erion_auto_mode")
        iMotor = motor()
        rospy.Subscriber('/auto_drive_control/cmd_vel',
                         Int16MultiArray, callback)
       # iMotor = motor()
        rospy.spin()

    except KeyboardInterrupt:
        pass
    finally:
        del(iMotor)

