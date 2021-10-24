import RPi.GPIO as IO
import time

pwm_bdlc_rack_Pin = 18
pwm_bdlc_whl_Pin = 19

dir_bldc_rack_pin = 27
dir_bldc_whl_pin = 8

run_bldc_rack_pin = 22
run_bldc_whl_pin = 7

## inwhl
pwmLPin = 12  # Jetson Nano PIN 32
pwmRPin = 13
dirLPin = 6  # Jetson Nano PIN 31
dirRPin = 5
brkLPin = 4
brkRPin = 17

IO.setmode(IO.BCM)
IO.setwarnings(False)

IO.setup([pwm_bdlc_rack_Pin,dir_bldc_rack_pin,run_bldc_rack_pin,pwm_bdlc_whl_Pin,dir_bldc_whl_pin,run_bldc_whl_pin,pwmLPin,pwmRPin,dirLPin,dirRPin,brkLPin,brkRPin], IO.OUT)

# bldc rack
pwm_bdlc_rack_Pin = IO.PWM(pwm_bdlc_rack_Pin, 1000)  
pwm_bdlc_rack_Pin.start(0)

# bldc whl
pwm_bdlc_whl_Pin = IO.PWM(pwm_bdlc_whl_Pin, 1000)  
pwm_bdlc_rack_Pin.start(0)

# inwhl
pwmLPin = IO.PWM(pwmLPin, 1000)  
pwmLPin.start(0)

pwmRPin = IO.PWM(pwmRPin, 1000)  
pwmRPin.start(0)



while True:
    try:
        ##fold     
        ##### bldc rack
        pwm_bdlc_rack_Pin.ChangeDutyCycle(10)
    
        # whl False ==> go to car  /  True ==> go out car
        IO.output(dir_bldc_rack_pin, True)
    
        #  run/brk Pin , False ==> RUN / True ==> STOP 
        IO.output(run_bldc_rack_pin, False)

        ##### bldc whl
        pwm_bdlc_rack_Pin.ChangeDutyCycle(50)
    
        # whl False ==> go to car  /  True ==> go out car
        IO.output(dir_bldc_rack_pin, False)
    
        #  run/brk Pin , False ==> RUN / True ==> STOP 
        IO.output(run_bldc_rack_pin, False)

        ###### inwhl
        pwmLPin.ChangeDutyCycle(0)
	pwmRPin.ChangeDutyCycle(0)
	# dir
        # forward direction
        IO.output(dirLPin, True)
        IO.output(dirRPin, False)
        # run / brk
        IO.output(brkLPin, False)
        IO.output(brkRPin, False)

	time.sleep(4)
        
        #unfold
        ##fold     
        ##### bldc rack
        pwm_bdlc_rack_Pin.ChangeDutyCycle(10)
    
        # whl False ==> go to car  /  True ==> go out car
        IO.output(dir_bldc_rack_pin, False)
    
        #  run/brk Pin , False ==> RUN / True ==> STOP 
        IO.output(run_bldc_rack_pin, False)

        ##### bldc whl
        pwm_bdlc_rack_Pin.ChangeDutyCycle(50)
    
        # whl False ==> go to car  /  True ==> go out car
        IO.output(dir_bldc_rack_pin, True)
    
        #  run/brk Pin , False ==> RUN / True ==> STOP 
        IO.output(run_bldc_rack_pin, False)

        ###### inwhl
        pwmLPin.ChangeDutyCycle(0)
	pwmRPin.ChangeDutyCycle(0)
	# dir
        # backward direction
        IO.output(dirLPin, True)
        IO.output(dirRPin, False)
        # run / brk
        IO.output(brkLPin, False)
        IO.output(brkRPin, False)


        


	time.sleep(100000000000)
	

    except KeyboardInterrupt:
        print("interrrupt")
        pwm_bdlc_rack_Pin.stop()
        IO.cleanup()
        break

