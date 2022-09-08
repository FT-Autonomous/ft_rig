import RPi.GPIO as GPIO
from time import sleep
import sys
from .steering import get_pinion_displacement

steering_pwm_pin = 13 # 18 # yellow
steering_in1_pin = 16 # 32 # black
steering_in2_pin = 22 # 31 # white
        
GPIO.setmode(GPIO.BOARD)

# Set up GPIO output pins
GPIO.setup(steering_in1_pin, GPIO.OUT)
GPIO.setup(steering_in2_pin, GPIO.OUT)
steering_pwm_obj = GPIO.PWM(throttle_lpwm_pin, 500)

# Start at duty cycle (speed zero)
steering_pwm_obj.start(0)

GPIO.output(steering_in2_pin, GPIO.LOW)
while True:
    GPIO.output(steering_in1_pin, GPIO.HIGH)
    steering_pwm_obj.ChangeDutyCycle(19)
    sleep(1)
    steering_pwm_obj.ChangeDutyCycle(0)
    GPIO.output(steering_in1_pin, GPIO.LOW)
    sleep(1)
    GPIO.output(steering_in2_pin, GPIO.HIGH)
    steering_pwm_obj.ChangeDutyCycle(19)
    sleep(1)
    steering_pwm_obj.ChangeDutyCycle(0)
    GPIO.output(steering_in2_pin, GPIO.LOW)
    sleep(1)

