import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30fps
        self.cap = cv2.VideoCapture(0)
        self.get_logger().info('Camera node started!')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height, msg.width, _ = frame.shape
            msg.encoding = 'bgr8'
            msg.data = frame.tobytes()
            msg.step = msg.width * 3
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
