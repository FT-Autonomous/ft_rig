import RPi.GPIO as GPIO
import rclpy, rclpy.node
from ackermann_msgs.msg import AckermannDriveStamped
import subprocess
import math
from time import sleep

def get_pinion_distance(i2c_bus=8):
    i2c_bus = str(i2c_bus)
    subprocess.Popen(["i2cset", "-y" ,i2c_bus, "0x52", "0x00", "0x00"]) 
    upper = subprocess.Popen(["i2cget", "-y" ,i2c_bus, "0x52", "0x00"], stdout=subprocess.PIPE).stdout.read()[2:-1]
    lower = subprocess.Popen(["i2cget", "-y" ,i2c_bus, "0x52", "0x01"], stdout=subprocess.PIPE).stdout.read()[2:-1]
    return int(upper + lower, 16)

class Rig(rclpy.node.Node):
    def __init__(self):
        super().__init__("ft_rig_node")
        self.declare_parameter("max_steering_angle", math.radians(29))
        self.declare_parameter("max_pinion_displacement", 75.0) # True value is 75
        self.declare_parameter("pinion_home", 100.0)

        self.throttle_state = THROTTLE_STOP

        self.steering_in1_pin = 16 # brown
        self.steering_in2_pin = 22 # green
        self.steering_pwm_pin = 13 # red
        self.throttle_lpwm_pin = 18 # yellow
        self.throttle_rpwm_pin = 15 # orange
        self.throttle_len_pin = 32 # black
        self.throttle_ren_pin = 31 # white
        
        GPIO.setmode(GPIO.BOARD)

        # Set up GPIO output pins
        GPIO.setup(self.steering_in1_pin, GPIO.OUT)
        GPIO.setup(self.steering_in2_pin, GPIO.OUT)
        GPIO.setup(self.throttle_len_pin, GPIO.OUT)
        GPIO.setup(self.throttle_ren_pin, GPIO.OUT)

        # Create PWM clients
        self.steering_pwm_obj = GPIO.PWM(self.steering_pwm_pin, 500)
        self.throttle_lpwm_obj = GPIO.PWM(self.throttle_lpwm_pin, 500)
        self.throttle_rpwm_obj = GPIO.PWM(self.throttle_rpwm_pin, 500)

        # Start at duty cycle (speed zero)
        self.steering_pwm_obj.start(30)
        self.throttle_lpwm_obj.start(0)
        self.throttle_rpwm_obj.start(0)

        self.target_pinion_displacement = 0.0
        self.target_throttle_pwm = 0.0
        self.target_pwm = 0.0

        self.drive_stamped_sub = self.create_subscription(AckermannDriveStamped, "/cmd", self.drive_stamped_cb, 1)
        self.timer = self.create_timer(0.1, self.timer_cb)

        GPIO.output(self.throttle_len_pin, GPIO.HIGH)
        GPIO.output(self.throttle_ren_pin, GPIO.HIGH)

    def drive_stamped_cb(self, drive_stamped: AckermannDriveStamped):
        self.max_steering_angle = self.get_parameter("max_steering_angle").get_parameter_value().double_value
        self.max_pinion_displacement = self.get_parameter("max_pinion_displacement").get_parameter_value().double_value
        self.target_pinion_displacement = max(-self.max_steering_angle, min(self.max_steering_angle, drive_stamped.drive.steering_angle)) / self.max_steering_angle * self.max_pinion_displacement
        self.target_pwm = 15 if drive_stamped.drive.acceleration > 0 else 0

    def timer_cb(self):
        if self.target_pwm == 0: self.throttle_stop()
        elif self.target_pwm > 0: self.throttle_forwards()
        else: self.throttle_backwards()
        pinion_home = self.get_parameter("pinion_home").get_parameter_value().double_value
        pinion_distance = get_pinion_distance()
        pinion_target = pinion_home + self.target_pinion_displacement
        error = pinion_target - pinion_distance
        if abs(error) < 5: self.steer_stop()
        elif error > 0: self.steer_left()
        else: self.steer_right()
        self.get_logger().info(f"error is {error}, target pwm  is {self.target_pwm}, current distance is {pinion_distance}, target is {self.target_pinion_displacement}, state is {self.throttle_state}")

    def steer_right(self):
        GPIO.output(self.steering_in1_pin, GPIO.LOW)
        GPIO.output(self.steering_in2_pin, GPIO.HIGH)

    def steer_stop(self):
        GPIO.output(self.steering_in1_pin, GPIO.LOW)
        GPIO.output(self.steering_in2_pin, GPIO.LOW)

    def steer_left(self):
        GPIO.output(self.steering_in1_pin, GPIO.HIGH)
        GPIO.output(self.steering_in2_pin, GPIO.LOW)

    def throttle_stop(self):
        self.throttle_lpwm_obj.ChangeDutyCycle(0)
        self.throttle_rpwm_obj.ChangeDutyCycle(0)

    def throttle_forwards(self):
        self.throttle_lpwm_obj.ChangeDutyCycle(0)
        self.throttle_rpwm_obj.ChangeDutyCycle(self.target_pwm)

    # Cursed method, wont exist for DDT
    def throttle_backwards(self):
        self.throttle_rpwm_obj.ChangeDutyCycle(0)
        self.throttle_lpwm_obj.ChangeDutyCycle(self.target_pwm)
            

def main():
    rclpy.init()
    try:
        rig = Rig()
        rclpy.spin(Rig())
    except KeyboardInterrupt as e:
        print("Interrupt caught. Performing GPIO cleanup. WARNING: Cancelling now may brick your Jetson!!!")
        GPIO.cleanup()
    rclpy.shutdown()
