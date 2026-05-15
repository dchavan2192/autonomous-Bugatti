import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher = self.create_publisher(String, 'arduino_response', 10)
        self.distance_pub = self.create_publisher(Float32, 'distance', 10)
        self.subscription = self.create_subscription(String, 'car_command', self.command_callback, 10)
        self.ser = serial.Serial('/dev/arduino', 115200, timeout=1)
        time.sleep(0.1)
        self.create_timer(0.05, self.read_serial)
        self.get_logger().info('Serial node started!')

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode().strip()
            if line.startswith('DIST:'):
                try:
                    dist = float(line.split(':')[1])
                    msg = Float32()
                    msg.data = dist
                    self.distance_pub.publish(msg)
                except:
                    pass
            elif line:
                out = String()
                out.data = line
                self.publisher.publish(out)

    def command_callback(self, msg):
        command = msg.data + '\r\n'
        self.ser.write(command.encode())

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
