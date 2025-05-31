import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .pwm_driver import PWMDriver
from .pwm_software_driver import SoftwarePWMDriver

class SigynTesticleTwisterNode(Node):
    def __init__(self):
        super().__init__('sigyn_testicle_twister_node')
        
        # Use software PWM driver for Pi 5 compatibility
        # Hardware PWM has issues on Pi 5 with Ubuntu 24.04 kernel 6.8.0
        self.use_software_pwm = self.declare_parameter('use_software_pwm', True).value
        
        if self.use_software_pwm:
            self.get_logger().info("Using Software PWM (recommended for Pi 5)")
            gpio_pin = self.declare_parameter('gpio_pin', 18).value  # Default GPIO 18 (Pin 12)
            self.driver = SoftwarePWMDriver(gpio_pin=gpio_pin, logger=self.get_logger())
        else:
            self.get_logger().info("Using Hardware PWM (may not work on Pi 5)")
            pwm_chip = self.declare_parameter('pwm_chip', 1).value
            pwm_channel = self.declare_parameter('pwm_channel', 0).value
            self.driver = PWMDriver(chip=pwm_chip, channel=pwm_channel, logger=self.get_logger())
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_testicle_twister',
            self.listener_callback,
            10)
        self.period_ns = 20000000  # 20ms period (50Hz)

    def listener_callback(self, msg):
        x = msg.linear.x
        # Clamp and map x to pulse width (ns)
        min_x = -1000
        max_x = 1000
        min_pulse = 1100000  # 1.1ms
        max_pulse = 2000000  # 2.0ms
        if x <= min_x:
            pulse = min_pulse
        elif x >= max_x:
            pulse = max_pulse
        else:
            # Linear interpolation between min_x and max_x
            pulse = int(min_pulse + (x - min_x) * (max_pulse - min_pulse) / (max_x - min_x))
        self.get_logger().info(f"Setting PWM pulse: {pulse} ns for x={x}")
        self.driver.set_pwm(self.period_ns, pulse)

    def destroy_node(self):
        self.driver.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SigynTesticleTwisterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
