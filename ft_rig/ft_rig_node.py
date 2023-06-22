import rclpy, rclpy.node
from ackermann_msgs.msg import AckermannDriveStamped
from .motors import SteeringMotor, ThrottleMotor, init
from .steering import get_pinion_distance

class Rig(rclpy.node.Node):
    def __init__(self):
        super().__init__("ft_rig_node")

        self.steering = SteeringMotor()
        self.throttle = ThrottleMotor()

        self.declare_parameter("max_steering_angle", self.steering.max_angle)
        self.declare_parameter("max_pinion_displacement", self.steering.max_pinion_displacement)
        self.declare_parameter("pinion_home", self.steering.pinion_home)

        self.drive_stamped_sub = self.create_subscription(AckermannDriveStamped, "/cmd", self.drive_stamped_cb, 1)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.direction = None
        self.target_pinion_displacement = 0.0

    def drive_stamped_cb(self, drive_stamped: AckermannDriveStamped):
        self.steering.max_angle = self.get_parameter("max_steering_angle").get_parameter_value().double_value
        self.steering.max_pinion_displacement = self.get_parameter("max_pinion_displacement").get_parameter_value().double_value
        self.target_pinion_displacement = max(-self.steering.max_angle, min(self.steering.max_angle, drive_stamped.drive.steering_angle)) / self.steering.max_angle * self.steering.max_pinion_displacement
        self.throttle.target_duty = ThrottleMotor.SAFE_DUTY if abs(drive_stamped.drive.speed) > 0 else 0
        if drive_stamped.drive.speed > 0:
            self.direction = "forwards"
        elif drive_stamped.drive.speed < 0:
            self.direction = "backwards"
        else:
            self.direction = None

    def timer_cb(self):
        if self.direction == "forwards":
            self.throttle.forwards()
        elif self.direction == "backwards":
            self.throttle.backwards()
        else:
            self.throttle.stop()
        self.steering.pinion_home = self.get_parameter("pinion_home").get_parameter_value().double_value
        self.steering.steer_to(self.target_pinion_displacement, block=False)
        error = self.steering.compute_error(self.target_pinion_displacement)
        self.get_logger().info(f"pinionhome is {self.steering.pinion_home}, {'we need to go right' if error > 0 else 'we need to go left'} error is {self.steering.compute_error(self.target_pinion_displacement)}, target pwm  is {self.throttle.target_duty}, current distance is {get_pinion_distance()}, target is {self.steering.pinion_home + self.target_pinion_displacement}")
            
def main():
    init()
    rclpy.init()
    try:
        rclpy.spin(Rig())
    except KeyboardInterrupt as e:
        print("Interrupt caught. Performing GPIO cleanup. WARNING: Cancelling now may brick your Jetson!!!")
        GPIO.cleanup()
    rclpy.shutdown()
