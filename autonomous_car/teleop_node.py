import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import sys
import tty
import termios
import select

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.cmd_publisher = self.create_publisher(String, 'car_command', 10)
        self.steering_publisher = self.create_publisher(Float32, 'steering_angle', 10)
        self.current_angle = 0.0
        self.get_logger().info('Teleop node started!')
        self.get_logger().info('W=Forward S=Backward A=Left D=Right Space=Stop Q=Quit')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0)[0] else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_steering(self, angle):
        angle = max(-90.0, min(90.0, angle))
        self.current_angle = angle
        msg = Float32()
        msg.data = angle
        self.steering_publisher.publish(msg)

    def publish_drive(self, command):
        msg = String()
        msg.data = command
        self.cmd_publisher.publish(msg)

    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        last_command = ''
        try:
            while True:
                key = self.get_key()

                if key == 'w':
                    if last_command != 'FORWARD':
                        self.publish_drive('FORWARD')
                        last_command = 'FORWARD'
                elif key == 's':
                    if last_command != 'BACKWARD':
                        self.publish_drive('BACKWARD')
                        last_command = 'BACKWARD'
                elif key == 'a':
                    self.publish_steering(self.current_angle - 25.0)
                elif key == 'd':
                    self.publish_steering(self.current_angle + 25.0)
                elif key == ' ':
                    self.publish_drive('STOP')
                    self.publish_steering(0.0)
                    last_command = 'STOP'
                elif key == 'q':
                    self.publish_drive('STOP')
                    self.publish_steering(0.0)
                    break
                elif key == '':
                    if last_command != 'STOP':
                        self.publish_drive('STOP')
                        last_command = 'STOP'

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()

if __name__ == '__main__':
    main()
