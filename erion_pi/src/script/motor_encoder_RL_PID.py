import RPi.GPIO as IO
import time

pwmLPin = 12 #Jetson Nano PIN 32
pwmRPin = 21
dirLPin = 6 #Jetson Nano PIN 31
dirRPin = 13
#pwmPin = 19 #Jetson Nano PIN 35
#dirPin = 13 #Jetson Nano PIN 33

encLPinA = 23 #Jetson Nano PIN 16
encLPinB = 24 #Jetson Nano PIN 18
encRPinA = 11
encRPinB = 25

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encLPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encLPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encRPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encRPinB, IO.IN, pull_up_down=IO.PUD_UP)

IO.setup([pwmLPin,pwmRPin,dirLPin,dirRPin], IO.OUT)


pL = IO.PWM(pwmLPin,60) #12pin , strength 20%
pL.start(0)

pR = IO.PWM(pwmRPin,60) #12pin , strength 20%
pR.start(0)

encoderLPos = 0
encoderRPos = 0
cnt_LA=0
cnt_LB=0

def encoderLA(channel):
    global encoderLPos
    global cnt_LA
    cnt_LA += 1
    if IO.input(encLPinA) == IO.input(encLPinB):
        encoderLPos += 1
    else:
        encoderLPos -= 1
    #print('PinA: %d, encoder: %d' %(channel, encoderPos))


def encoderLB(channel):
    global encoderLPos
    global cnt_LB
    cnt_LB += 1
    if IO.input(encLPinA) == IO.input(encLPinB):
        encoderLPos -= 1
    else:
        encoderLPos += 1
    #print('PinB: %d, encoder: %d' %(channel, encoderPos))

def encoderRA(channel):
    global encoderRPos
    if IO.input(encRPinA) == IO.input(encRPinB):
        encoderRPos += 1
    else:
        encoderRPos -= 1
    #print('PinA: %d, encoder: %d' %(channel, encoderPos))


def encoderRB(channel):
    global encoderRPos
    if IO.input(encRPinA) == IO.input(encRPinB):
        encoderRPos -= 1
    else:
        encoderRPos += 1
    #print('PinB: %d, encoder: %d' %(channel, encoderPos))

IO.add_event_detect(encLPinA, IO.BOTH, callback=encoderLA)
IO.add_event_detect(encLPinB, IO.BOTH, callback=encoderLB)

IO.add_event_detect(encRPinA, IO.BOTH, callback=encoderRA)
IO.add_event_detect(encRPinB, IO.BOTH, callback=encoderRB)

# targetLDeg = 360
targetLSpd = 10

# targetRDeg = 360
targetRSpd = 0

ratio = 360/4096/3
Kp = 2
Kd = 0
Ki = 0
dt = 0
dt_sleep = 100
tolerance = 1

start_time = time.time()
errL_prev = 0
errR_prev = 0
time_prev = 0

while True:
    motorLDeg = encoderLPos * ratio
    spdLrpm = encoderLPos * 60 * 1/dt_sleep * 1/4096
    errL = targetLSpd - spdLrpm
    derrL = errL - errL_prev
    dtL = time.time() - time_prev
    controlL = Kp*errL + Kd*derrL/dtL + Ki*errL*dtL

    # errorL = targetLDeg - motorLDeg
    # de = errorL - error_prev
    # dt = time.time() - time_prev
    # controlL = Kp*error + Kd*de/dt + Ki*error*dt

    # errorL_prev = errorL
    # time_prev = time.time()

    IO.output(dirLPin, controlL <= 0)
    pL.ChangeDutyCycle(min(abs(controlL),100))


    motorRDeg = encoderRPos * ratio

    spdRrpm = encoderRPos * 60 * 1000/dt_sleep * 1/4096
    errR = targetRSpd - spdRrpm
    derrR = errR -errR_prev
    dtR = time.time() - time_prev
    controlR = Kp*errR + Kd*derrR/dtR + Ki*errR*dtR

    # errorL = targetLDeg - motorLDeg
    # de = errorL - error_prev
    # dt = time.time() - time_prev
    # controlL = Kp*error + Kd*de/dt + Ki*error*dt

    # errorL_prev = errorL
    # time_prev = time.time()

    IO.output(dirRPin, controlR <= 0)
    pR.ChangeDutyCycle(min(abs(controlR),100))

    print(f'time={time.time()-start_time:.2f},spdLrpm={spdLrpm:.2f},tarrpm={targetLSpd:.2f} encL={encoderLPos:.2f}, encR={encoderRPos:.2f}, degL={motorLDeg:.2f}, degR ={motorRDeg:.2f}, errL ={errL:.2f}, errR ={errR:.2f}, controlL = {controlL:.2f}, controlR = {controlR:.2f}')
    time.sleep(1)

