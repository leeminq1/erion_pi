import RPi.GPIO as IO
import time

pwm_bdlc_rack_Pin = 18
pwm_bdlc_whl_Pin = 19

dir_bldc_rack_pin = 27
dir_bldc_whl_pin = 8

run_bldc_rack_pin = 22
run_bldc_whl_pin = 7


IO.setmode(IO.BCM)
IO.setwarnings(False)

IO.setup([pwm_bdlc_rack_Pin, pwm_bdlc_whl_Pin, dir_bldc_rack_pin,
         dir_bldc_whl_pin, run_bldc_rack_pin, run_bldc_whl_pin], IO.OUT)

# IO.setup([pwm_bdlc_whl_Pin, dir_bldc_whl_pin, run_bldc_whl_pin], IO.OUT)


pwm_bdlc_rack_Pin = IO.PWM(pwm_bdlc_rack_Pin, 1000)  # 12pin , strength 20%
pwm_bdlc_rack_Pin.start(50)

pwm_bdlc_whl_Pin = IO.PWM(pwm_bdlc_whl_Pin, 1000)  # 12pin , strength 20%
pwm_bdlc_whl_Pin.start(50)

while True:
    try:
        print("code 123123 work")
        pwm_bdlc_rack_Pin.ChangeDutyCycle(10)
        pwm_bdlc_whl_Pin.ChangeDutyCycle(50)

        IO.output(dir_bldc_rack_pin, True)
        IO.output(dir_bldc_whl_pin, True)

        IO.output(run_bldc_rack_pin, False)
        IO.output(run_bldc_whl_pin, True)

	time.sleep(3)
        print("mode change")
        IO.output(dir_bldc_rack_pin, True)
        IO.output(dir_bldc_whl_pin, True)

        IO.output(run_bldc_rack_pin, True)
        IO.output(run_bldc_whl_pin, False)
	time.sleep(100000000000)
	

    except KeyboardInterrupt:
        print("interrrupt")
        pwm_bdlc_rack_Pin.stop()
        pwm_bdlc_whl_Pin.stop()
        IO.cleanup()
        break

