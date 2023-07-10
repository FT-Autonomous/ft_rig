import RPi.GPIO as GPIO
import math
from .steering import get_pinion_distance

def init():
    GPIO.setmode(GPIO.BOARD)

def deinit():
    GPIO.cleanup()

class SteeringMotor:
    SAFE_DUTY = 40
    DEATH_AND_FIRE_DUTY = 80

    def __init__(self):
        self.in1_pin = 16 # brown
        self.in2_pin = 22 # green
        self.pwm_pin = 13 # red
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        self.pwm_obj = GPIO.PWM(self.pwm_pin, 500)
        self.pwm_obj.start(SteeringMotor.SAFE_DUTY)

        self.max_angle = math.radians(29)
        self.max_pinion_displacement = 60.0
        self.pinion_home = 80.0

    def compute_error(self, displacement):
        return get_pinion_distance() - (self.pinion_home + displacement)

    def steer_to(self, displacement, block=True):
        if block:
            while abs(self.compute_error()) > 5:
                self.steer_to(displacement, block=False)
            return True
        else:
            error = self.compute_error(displacement)
            if abs(error) < 5:
                self.stop()
                return True
            elif error > 0:
                print("we need to go right")
                self.right()
            else:
                print("we need to go left")
                self.left()

    def right(self):
        GPIO.output(self.in1_pin, GPIO.HIGH)
        GPIO.output(self.in2_pin, GPIO.LOW)

    def stop(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.LOW)

    def left(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.HIGH)

class ThrottleMotor:
    SAFE_DUTY = 20
    DEATH_AND_FIRE_DUTY = 90

    def __init__(self, target_duty=10):
        self.lpwm_pin = 18 # yellow
        self.rpwm_pin = 15 # orange
        self.len_pin = 32 # black
        self.ren_pin = 31 # white

        # Set up GPIO output pins
        GPIO.setup(self.len_pin, GPIO.OUT)
        GPIO.setup(self.ren_pin, GPIO.OUT)
        GPIO.output(self.len_pin, GPIO.HIGH)
        GPIO.output(self.ren_pin, GPIO.HIGH)

        # Create PWM clients
        self.lpwm_obj = GPIO.PWM(self.lpwm_pin, 500)
        self.rpwm_obj = GPIO.PWM(self.rpwm_pin, 500)
        self.lpwm_obj.start(0)
        self.rpwm_obj.start(0)

        # Set nominal PWM value
        self.target_duty = target_duty

    def stop(self):
        self.lpwm_obj.ChangeDutyCycle(0)
        self.rpwm_obj.ChangeDutyCycle(0)

    def forwards(self):
        self.lpwm_obj.ChangeDutyCycle(0)
        self.rpwm_obj.ChangeDutyCycle(self.target_duty)

    # Cursed method, wont exist for DDT
    def backwards(self):
        self.rpwm_obj.ChangeDutyCycle(0)
        self.lpwm_obj.ChangeDutyCycle(self.target_duty)
