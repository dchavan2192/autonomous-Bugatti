import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher = self.create_publisher(String, 'arduino_response', 10)
        self.subscription = self.create_subscription(String, 'car_command', self.command_callback, 10)
        self.ser = serial.Serial('/dev/arduino', 115200, timeout=1)
        time.sleep(0.1)
        self.get_logger().info('Serial node started!')

    def command_callback(self, msg):
        command = msg.data + '\r\n'
        self.ser.write(command.encode())
        time.sleep(0.01)
        if self.ser.in_waiting > 0:
            response = self.ser.readline().decode().strip()
            out = String()
            out.data = response
            self.publisher.publish(out)
            self.get_logger().info(f'Arduino said: {response}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
