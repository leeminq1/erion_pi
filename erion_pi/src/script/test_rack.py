import RPi.GPIO as IO
import time

print("run")
pwm_bdlc_rack_Pin = 18

dir_bldc_rack_pin = 27

run_bldc_rack_pin = 22




## inwhl
#pwmLPin = 12  # Jetson Nano PIN 32
#pwmRPin = 13
#dirLPin = 6  # Jetson Nano PIN 31
#dirRPin = 5
#brkLPin = 4
#brkRPin = 17


IO.setmode(IO.BCM)
IO.setwarnings(False)

IO.setup([pwm_bdlc_rack_Pin,dir_bldc_rack_pin,run_bldc_rack_pin], IO.OUT)

pwm_bdlc_rack_Pin = IO.PWM(pwm_bdlc_rack_Pin, 1000)  # 12pin , strength 20%
pwm_bdlc_rack_Pin.start(50)


while True:
    try:
        ##fold
	print("fold")
       
        ##### bldc
        pwm_bdlc_rack_Pin.ChangeDutyCycle(10)
    
        # whl False ==> go to car  /  True ==> go out car
        IO.output(dir_bldc_rack_pin, True)
    
        #  run/brk Pin , False ==> RUN / True ==> STOP 
        IO.output(run_bldc_rack_pin, False)


	time.sleep(4)
        
        #unfold
#	print("unfold")
#       
#        ##### bldc
#        pwm_bdlc_rack_Pin.ChangeDutyCycle(10)
#    
#        # whl False ==> go to car  /  True ==> go out car
#        IO.output(dir_bldc_rack_pin, False)
#    
#        #  run/brk Pin , False ==> RUN / True ==> STOP 
#        IO.output(run_bldc_rack_pin, False)

        


	time.sleep(100000000000)
	

    except KeyboardInterrupt:
        print("interrrupt")
        pwm_bdlc_rack_Pin.stop()
        IO.cleanup()
        break

