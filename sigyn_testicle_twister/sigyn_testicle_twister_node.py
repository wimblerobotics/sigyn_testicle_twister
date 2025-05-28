import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .pwm_driver import PWMDriver

class SigynTesticleTwisterNode(Node):
    def __init__(self):
        super().__init__('sigyn_testicle_twister_node')
        self.driver = PWMDriver(logger=self.get_logger())
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
