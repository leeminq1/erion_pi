import RPi.GPIO as IO
import time

print("run")
#pwm_bdlc_rack_Pin = 18
pwm_bdlc_whl_Pin = 19

#dir_bldc_rack_pin = 27
dir_bldc_whl_pin = 8

#run_bldc_rack_pin = 22
run_bldc_whl_pin = 7

# inwhl
pwmLPin = 12  # Jetson Nano PIN 32
pwmRPin = 13
dirLPin = 6  # Jetson Nano PIN 31
dirRPin = 5
brkLPin = 4
brkRPin = 17


IO.setmode(IO.BCM)
IO.setwarnings(False)

IO.setup([pwm_bdlc_whl_Pin,dir_bldc_whl_pin,run_bldc_whl_pin,pwmLPin,pwmRPin,dirLPin,dirRPin,brkLPin,brkRPin], IO.OUT)

pwm_bdlc_whl_Pin = IO.PWM(pwm_bdlc_whl_Pin, 1000)  # 12pin , strength 20%
pwm_bdlc_whl_Pin.start(50)

pwmLPin = IO.PWM(pwmLPin, 1000)  # 12pin , strength 20%
pwmLPin.start(50)

pwmRPin = IO.PWM(pwmRPin, 1000)  # 12pin , strength 20%
pwmRPin.start(50)

while True:
    try:
        ##forward
	print("forward")
       
        ##### bldc
        pwm_bdlc_whl_Pin.ChangeDutyCycle(100)
    
        # whl False ==> go to car  /  True ==> go out car
        IO.output(dir_bldc_whl_pin, False)
    
        #  run/brk Pin , False ==> RUN / True ==> STOP 
        IO.output(run_bldc_whl_pin, False)

        ###### inwhl
        pwmLPin.ChangeDutyCycle(60)
	pwmRPin.ChangeDutyCycle(60)
	# dir
        IO.output(dirLPin, True)
        IO.output(dirRPin, False)
        # run / brk
        IO.output(brkLPin, False)
        IO.output(brkRPin, False)

	time.sleep(4)
        print("backward")

        ##### bldc
        pwm_bdlc_whl_Pin.ChangeDutyCycle(20)
    
        # whl False ==> go to car  /  True ==> go out car
        IO.output(dir_bldc_whl_pin, True)
    
        #  run/brk Pin , False ==> RUN / True ==> STOP 
        IO.output(run_bldc_whl_pin, False)

        ###### inwhl
        pwmLPin.ChangeDutyCycle(60)
	pwmRPin.ChangeDutyCycle(60)
	# dir
        IO.output(dirLPin, False)
        IO.output(dirRPin, True)
        # run / brk
        IO.output(brkLPin, False)
        IO.output(brkRPin, False)


	time.sleep(100000000000)
	

    except KeyboardInterrupt:
        print("interrrupt")
        pwm_bdlc_whl_Pin.stop()
        pwmLPin.stop()
	pwmRPin.stop() 
        IO.cleanup()
        break

