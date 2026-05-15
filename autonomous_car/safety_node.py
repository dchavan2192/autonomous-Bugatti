import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class SafetyNode(Node):
    STOP_DISTANCE = 20.0  # cm

    def __init__(self):
        super().__init__('safety_node')
        self.distance_sub = self.create_subscription(Float32, 'distance', self.distance_callback, 10)
        self.cmd_pub = self.create_publisher(String, 'car_command', 10)
        self.blocked = False
        self.get_logger().info('Safety node started!')

    def distance_callback(self, msg):
        dist = msg.data
        if dist > 0 and dist < self.STOP_DISTANCE:
            if not self.blocked:
                stop = String()
                stop.data = 'STOP'
                self.cmd_pub.publish(stop)
                self.blocked = True
                self.get_logger().warn(f'OBSTACLE at {dist:.1f}cm - STOPPING!')
        else:
            self.blocked = False

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
