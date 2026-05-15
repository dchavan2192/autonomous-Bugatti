
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import serial
import time

class UltrasonicNode(Node):
    STOP_DISTANCE = 20  # cm

    def __init__(self):
        super().__init__('ultrasonic_node')
        self.distance_pub = self.create_publisher(Float32, 'distance', 10)
        self.cmd_pub = self.create_publisher(String, 'car_command', 10)
        self.ser = serial.Serial('/dev/arduino', 115200, timeout=1)
        time.sleep(2)
        self.get_logger().info('Ultrasonic node started!')
        self.create_timer(0.1, self.read_distance)

    def read_distance(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode().strip()
            if line.startswith('DIST:'):
                dist = float(line.split(':')[1])
                msg = Float32()
                msg.data = dist
                self.distance_pub.publish(msg)
                self.get_logger().info(f'Distance: {dist}cm')

                if dist < self.STOP_DISTANCE and dist > 0:
                    stop = String()
                    stop.data = 'STOP'
                    self.cmd_pub.publish(stop)
                    self.get_logger().warn(f'OBSTACLE! {dist}cm - STOPPING')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
