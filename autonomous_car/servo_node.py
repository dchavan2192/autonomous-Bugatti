import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import os

class ServoNode(Node):
    CHIP = "/sys/class/pwm/pwmchip0"
    CHANNEL = 0
    PERIOD_NS = 20_000_000
    CENTER_US = 1500
    LEFT_US = 850
    RIGHT_US = 2050

    def __init__(self):
        super().__init__('servo_node')
        self.subscription = self.create_subscription(
            Float32, 'steering_angle', self.angle_callback, 10)
        self._setup_pwm()
        self.get_logger().info('Servo node started!')

    def _setup_pwm(self):
        pwm_dir = f"{self.CHIP}/pwm{self.CHANNEL}"
        if not os.path.exists(pwm_dir):
            self._write(f"{self.CHIP}/export", self.CHANNEL)
        self._write(f"{pwm_dir}/period", self.PERIOD_NS)
        self._write(f"{pwm_dir}/enable", 1)
        self.pwm_dir = pwm_dir

    def _write(self, path, value):
        with open(path, "w") as f:
            f.write(str(value))

    def angle_callback(self, msg):
        deg = max(-90.0, min(90.0, msg.data))
        if deg < 0:
            pulse_us = self.CENTER_US + (deg / 90.0) * (self.CENTER_US - self.LEFT_US)
        else:
            pulse_us = self.CENTER_US + (deg / 90.0) * (self.RIGHT_US - self.CENTER_US)
        self._write(f"{self.pwm_dir}/duty_cycle", int(pulse_us * 1000))
        self.get_logger().info(f'Steering: {deg:.1f} degrees')

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
