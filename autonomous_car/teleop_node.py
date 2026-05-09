import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import tty
import termios
import select

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(String, 'car_command', 10)
        self.get_logger().info('Teleop node started!')
        self.get_logger().info('Hold W=Forward S=Backward A=Left D=Right Space=Stop Q=Quit')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0)[0] else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        last_command = ''
        try:
            while True:
                key = self.get_key()
                msg = String()
                if key == 'w':
                    msg.data = 'FORWARD'
                elif key == 's':
                    msg.data = 'BACKWARD'
                elif key == 'a':
                    msg.data = 'LEFT'
                elif key == 'd':
                    msg.data = 'RIGHT'
                elif key == ' ':
                    msg.data = 'STOP'
                elif key == 'q':
                    msg.data = 'STOP'
                    self.publisher.publish(msg)
                    break
                elif key == '':
                    msg.data = 'STOP'
                else:
                    continue

                if msg.data != last_command:
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Sent: {msg.data}')
                    last_command = msg.data
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()

if __name__ == '__main__':
    main()
