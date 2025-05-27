import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .pwm_driver import PWMDriver

class SigynTesticleTwisterNode(Node):
    def __init__(self):
        super().__init__('sigyn_testicle_twister_node')
        self.driver = PWMDriver()
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_testicle_twister',
            self.listener_callback,
            10)
        self.period_ns = 20000000  # 20ms period (50Hz)

    def listener_callback(self, msg):
        x = msg.linear.x
        # Clamp and map x to pulse width (ns)
        if x <= -1000:
            pulse = 1100000
        elif x >= 1000:
            pulse = 2000000
        else:
            # Linear interpolation between 1.1ms and 2.0ms
            pulse = int(1500000 + (x / 1000) * 400000 / 2)
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
