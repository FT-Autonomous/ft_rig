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
        self.declare_parameter("throttle", True)
        self.declare_parameter("steering", True)

        self.drive_stamped_sub = self.create_subscription(AckermannDriveStamped, "/cmd", self.drive_stamped_cb, 1)
        self.timer = self.create_timer(0.2, self.timer_cb)
        self.direction = None
        self.target_pinion_displacement = 0.0

    def drive_stamped_cb(self, drive_stamped: AckermannDriveStamped):
        self.steering.max_angle = self.get_parameter("max_steering_angle").get_parameter_value().double_value
        self.steering.max_pinion_displacement = self.get_parameter("max_pinion_displacement").get_parameter_value().double_value

        # Only steer if the steering parameter is unset or not "no"
        if not self.get_parameter("steering").get_parameter_value().bool_value:
            self.target_pinion_displacement = 0.0
        else:
            self.target_pinion_displacement = max(-self.steering.max_angle, min(self.steering.max_angle, drive_stamped.drive.steering_angle)) \
                                                              / self.steering.max_angle * self.steering.max_pinion_displacement

        # Only move if the steering parameter is unset or not "no"
        if not self.get_parameter("throttle").get_parameter_value().bool_value:
            self.throttle.target_duty = 0.0
            self.direction = None
        else:
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
        self.get_logger().info(
            f"pinion_home (base): {self.steering.pinion_home}, " \
            f"pinion_offset (base): {get_pinion_distance()}, " \
            f"pinion_target (base): {self.steering.pinion_home + self.target_pinion_displacement}, " \
            f"error: {error} (moving {'right' if error > 0 else 'left'}), " \
            f"target_pwm: {self.throttle.target_duty}"
        )
            
def main():
    init()
    rclpy.init()
    try:
        rclpy.spin(Rig())
    except KeyboardInterrupt as e:
        print("Interrupt caught. Performing GPIO cleanup. WARNING: Cancelling now may brick your Jetson!!!")
        GPIO.cleanup()
    rclpy.shutdown()
