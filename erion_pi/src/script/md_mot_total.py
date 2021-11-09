#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray

#
import RPi.GPIO as GPIO
import time

# firebase
from firebase_admin import firestore
from firebase_admin import credentials
import firebase_admin


class Motor_key_app:
    @property
    def is_detected_key_app(self):
        return(time.time() - time_detected_key_app < 1.0)

    def get_firebase_input_value():
        db = firestore.client()
        doc_ref = db.collection(u'pi').document(u'key')
        doc = doc_ref.get()
        input_value = doc.to_dict()['input']
        # print("check the initial_value")
        # print(f'Document data: {input_value}')
        return input_value

    def setDir(self, setV, accel, steer):
        if setV == 'forward' and self.is_detected_key_app:
            rospy.loginfo("forward")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm_key_app(accel, steer)
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            server.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(server.dirLPin, True)
            GPIO.output(server.dirRPin, False)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'backward' and self.is_detected_key_app:
            rospy.loginfo("backward")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm_key_app(accel, steer)
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            server.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(server.dirLPin, False)
            GPIO.output(server.dirRPin, True)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'forward_right' and self.is_detected_key_app:
            rospy.loginfo("forward-right")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm_key_app(accel, steer)
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            server.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(server.dirLPin, True)
            GPIO.output(server.dirRPin, True)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'forward_left' and self.is_detected_key_app:
            rospy.loginfo("forward_left")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm_key_app(accel, steer)
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            server.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(server.dirLPin, True)
            GPIO.output(server.dirRPin, False)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'backward_right' and self.is_detected_key_app:
            rospy.loginfo("backward_right")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm_key_app(accel, steer)
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            server.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(server.dirLPin, False)
            GPIO.output(server.dirRPin, True)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'backward_left' and self.is_detected_key_app:
            rospy.loginfo("backward_left")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm_key_app(accel, steer)
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            server.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(server.dirLPin, False)
            GPIO.output(server.dirRPin, True)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        else:
            print("CCW setV : setDir('forward') ")
            print("CW setV : setDir('backword') ")

    def setDir_app(self, setV):
        if setV == 'forward' and self.is_detected_key_app:
            rospy.loginfo("forward")
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(50), 100))
            server.pL.ChangeDutyCycle(min(abs(50), 100))
            GPIO.output(server.dirLPin, True)
            GPIO.output(server.dirRPin, False)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'backward' and self.is_detected_key_app:
            rospy.loginfo("backward")
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(50), 100))
            server.pL.ChangeDutyCycle(min(abs(50), 100))
            GPIO.output(server.dirLPin, False)
            GPIO.output(server.dirRPin, True)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'forward_right' and self.is_detected_key_app:
            rospy.loginfo("forward-right")
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(30), 100))
            server.pL.ChangeDutyCycle(min(abs(60), 100))
            GPIO.output(server.dirLPin, True)
            GPIO.output(server.dirRPin, False)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'forward_left' and self.is_detected_key_app:
            rospy.loginfo("forward_left")
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(60), 100))
            server.pL.ChangeDutyCycle(min(abs(30), 100))
            GPIO.output(server.dirLPin, True)
            GPIO.output(server.dirRPin, False)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

    def cal_pwm_key_app(self, accel, steer):
        # target basic value
        target = 7
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
        if self.is_detected_key_app:
            print("stop")
            server.pR.ChangeDutyCycle(0)
            server.pL.ChangeDutyCycle(0)
            self.encoderLPos = 0
            self.encoderRPos = 0
            self.encoderLPos_prev = 0
            self.encoderRPos_prev = 0
            # chk direction  / stop pin
            GPIO.output(server.brkLPin, True)
            GPIO.output(server.brkRPin, True)

    def shortBreak(self):
        GPIO.output(self.GPIO_right_RP, False)
        GPIO.output(self.GPIO_right_RN, False)
        GPIO.output(self.GPIO_left_RP, False)
        GPIO.output(self.GPIO_left_RN, False)

    def start(self, setV, accel, steer):
        self.setDir(setV, accel, steer)

    def start_app(self, setV):
        self.setDir_app(setV)


class Motor_auto:
    @property
    def is_detected_auto(self):
        return(time.time() - time_detected_auto < 1.0)

    def setDir(self, setV, accel, steer):
        if setV == 'forward' and self.is_detected_auto:
            rospy.loginfo("forward")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm_auto(accel, steer)
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            server.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(server.dirLPin, True)
            GPIO.output(server.dirRPin, False)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'forward_right' and self.is_detected_auto:
            rospy.loginfo("forward-right")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm_auto(accel, steer)
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            server.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(server.dirLPin, True)
            GPIO.output(server.dirRPin, False)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'forward_left' and self.is_detected_auto:
            rospy.loginfo("forward_left")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm_auto(accel, steer)
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            server.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(server.dirLPin, True)
            GPIO.output(server.dirRPin, False)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'right_spin' and self.is_detected_auto:
            rospy.loginfo("right_spin")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm_auto(accel, steer)
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            server.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(server.dirLPin, True)
            GPIO.output(server.dirRPin, True)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        elif setV == 'left_spin' and self.is_detected_auto:
            rospy.loginfo("left_spin")
            # enc cal
            self.pwmL, self.pwmR = self.cal_pwm_auto(accel, steer)
            # cmd oper
            server.pR.ChangeDutyCycle(min(abs(self.pwmR), 100))
            server.pL.ChangeDutyCycle(min(abs(self.pwmL), 100))
            GPIO.output(server.dirLPin, False)
            GPIO.output(server.dirRPin, False)
            GPIO.output(server.brkLPin, False)
            GPIO.output(server.brkRPin, False)

        else:
            print("CCW setV : setDir('forward') ")
            print("CW setV : setDir('backword') ")

    def cal_pwm_auto(self, accel, steer):
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
        if self.is_detected_auto:
            print("stop")
            server.pR.ChangeDutyCycle(0)
            server.pL.ChangeDutyCycle(0)
            self.encoderLPos = 0
            self.encoderRPos = 0
            self.encoderLPos_prev = 0
            self.encoderRPos_prev = 0
            # chk direction  / stop pin
            GPIO.output(server.brkLPin, True)
            GPIO.output(server.brkRPin, True)

    def shortBreak(self):
        GPIO.output(self.GPIO_right_RP, False)
        GPIO.output(self.GPIO_right_RN, False)
        GPIO.output(self.GPIO_left_RP, False)
        GPIO.output(self.GPIO_left_RN, False)

    def start(self, setV, accel, steer):
        self.setDir(setV, accel, steer)


class Server:
    def __init__(self):
        self.motor_key_app = Motor_key_app()
        self.motor_auto = Motor_auto()
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
        # print('PinA: %d, encoder: %d' %(channel, encoderPos))

    def encoderLB(self, channel):
        if GPIO.input(self.encLPinA) == GPIO.input(self.encLPinB):
            self.encoderLPos -= 1
        else:
            self.encoderLPos += 1
        # print('PinB: %d, encoder: %d' %(channel, encoderPos))

    def encoderRA(self, channel):
        if GPIO.input(self.encRPinA) == GPIO.input(self.encRPinB):
            self.encoderRPos += 1
        else:
            self.encoderRPos -= 1
        # print('PinA: %d, encoder: %d' %(channel, encoderPos))

    def encoderRB(self, channel):

        if GPIO.input(self.encRPinA) == GPIO.input(self.encRPinB):
            self.encoderRPos -= 1
        else:
            self.encoderRPos += 1
        # print('PinB: %d, encoder: %d' %(channel, encoderPos))

    def __del__(self):
        GPIO.cleanup()

    def md_motor_cmd_sel(self, msg):
        # mode detection
        msg_get_mode = int(msg.data[2])
        f_autodrvie = msg_get_mode == 0
        f_keyboard = msg_get_mode == 1
        f_app_control = msg_get_mode == 2

        # time
        global time_detected_key_app
        time_detected_key_app = time.time()

        if f_keyboard:
            # key  mode
            print("keyborad mode condition")
            # md mot ctrl
            self.md_motor_keyboard_mode(msg)
        elif f_app_control:
            # app mode
            print("app mode condition")
            # md mot ctrl
            self.md_motor_app_mode(msg)

        elif f_autodrvie:
            # auto drive mode
            print("autodrvie mode conditon")
            self.get_msg_by_auto_mode()

    # md_mot_ctrl_callback
    def md_motor_keyboard_mode(self, msg):

        # get msg
        accel = msg.data[0]
        steer = msg.data[1]

        f_forward = (accel > 0) and (steer == 0)
        f_backward = (accel < 0) and (steer == 0)
        f_forward_right = (accel > 0) and (steer > 0)
        f_forward_left = (accel > 0) and (steer < 0)
        f_backward_right = (accel < 0) and (steer > 0)
        f_backward_left = (accel < 0) and (steer < 0)
        f_vehicle_stop = (accel == 0) and (steer == 0)

        if f_forward:
            self.motor_key_app.start("forward", accel, steer)
            print("----keyboard forward-----")

        elif f_backward:
            self.motor_key_app.start("backward",  accel, steer)
            print("----keyboard backward-----")

        elif f_forward_right:
            self.motor_key_app.start("forward_right", accel, steer)
            print("----keyboard forward - right-----")

        elif f_forward_left:
            self.motor_key_app.start("forward_left", accel, steer)
            print("----keyboard forwad - left-----")

        elif f_backward_right:
            self.motor_key_app.start("backward_right", accel, steer)
            print("----keyboard backward - right-----")

        elif f_backward_left:
            self.motor_key_app.start("backward_left", accel, steer)
            print("----keyboard backward  left-----")

        elif f_vehicle_stop:
            print("--keyboard vehicle stop--")
            self.motor_key_app.stop()

    def md_motor_app_mode(self):
        input_value = self.motor_key_app.get_firebase_input_value
        if input_value == "go":
            # rospy.loginfo("accel : {} \t angular: {}".format(
            #     msg.data[0], msg.data[1]))
            self.motor_key_app.start_app("forward")
            print("----app forward-----")

        elif input_value == "back":
            # rospy.loginfo("accel : {} \t angular: {}".format(
            #     msg.data[0], msg.data[1]))
            self.motor_key_app.start_app("backward")
            print("----app backward-----")

        elif input_value == "right":
            # rospy.loginfo("accel : {} \t anlsgular: {}".format(
            #     msg.data[0], msg.data[1]))
            self.motor_key_app.start_app("forward_right")
            print("----app forward - right-----")

        elif input_value == "left":
            # rospy.loginfo("accel : {} \t angular: {}".format(
            #     msg.data[0], msg.data[1]))
            self.motor_key_app.start_app("forward_left")
            print("----app forwad - left-----")

        elif input_value == "stop":
            self.motor_key_app.stop()

        # md_mot_ctrl_auto_callback

    def get_msg_by_auto_mode(self):
        # msg get only autodrive mode
        rospy.Subscriber('/auto_drive_control/cmd_vel',
                         Int16MultiArray, self.md_motor_auto_mode)

    def md_motor_auto_mode(self, msg):
        # get msg
        accel = msg.data[0]
        steer = msg.data[1]

        # time
        global time_detected_auto
        time_detected_auto = time.time()

        # drive deteciton
        f_forward = (accel > 0) and (steer == 0)

        f_forward_right = (accel > 0) and (steer > 0)
        f_forward_left = (accel > 0) and (steer < 0)

        f_right_spin = (accel == 0) and (steer > 0)
        f_left_spin = (accel == 0) and (steer < 0)

        f_vehicle_stop = (accel == 0) and (steer == 0)

        if f_forward:
            self.motor_auto.start("forward", accel, steer)

        elif f_forward_right:
            self.motor_auto.start("forward_right", accel, steer)

        elif f_forward_left:
            self.motor_auto.start("forward_left", accel, steer)

        elif f_right_spin:
            self.motor_auto.start("right_spin", accel, steer)

        elif f_left_spin:
            self.motor_auto.start("left_spin", accel, steer)

        elif f_vehicle_stop:
            self.motor_auto.stop()


if __name__ == "__main__":
    try:
        # firebase init
        cred = credentials.Certificate(
            "erion_key.json")
        firebase_admin.initialize_app(cred)
        print("firebase certificated")

        # total node set
        rospy.init_node('erion_md_mot_total', anonymous=True)
        rospy.loginfo("erion_md_mot_total")

        # server class
        server = Server()

        # multiple subscirber
        # [accel / steer / mode / time ]
        # msg get every mode
        rospy.Subscriber('mode_sel_cmd_vel',
                         Int16MultiArray, server.md_motor_cmd_sel)
        rospy.spin()

    except KeyboardInterrupt:
        print("interrrupt")
        Motor_key_app.stop()
        Motor_auto.stop()
    finally:
        pass

