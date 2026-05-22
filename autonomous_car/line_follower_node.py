import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import cv2
import numpy as np

class LineFollowerNode(Node):
    KP = 0.15
    MAX_ANGLE = 45.0

    def __init__(self):
        super().__init__('line_follower_node')
        self.steering_pub = self.create_publisher(Float32, 'steering_angle', 10)
        self.cmd_pub = self.create_publisher(String, 'car_command', 10)
        self.cap = cv2.VideoCapture(0)
        self.running = False
        self.create_subscription(String, 'follower_cmd', self.follower_cmd_callback, 10)
        self.create_timer(0.05, self.process_frame)
        self.get_logger().info('Line follower node started!')

    def follower_cmd_callback(self, msg):
        if msg.data == 'START':
            self.running = True
            self.get_logger().info('Line following started!')
        elif msg.data == 'STOP':
            self.running = False
            cmd = String()
            cmd.data = 'STOP'
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Line following stopped!')

    def process_frame(self):
        if not self.running:
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        height, width = frame.shape[:2]
        frame = frame[height//2:, :]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (15, 15), 0)
        _, thresh = cv2.threshold(blur, 80, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 500:
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    image_center = width // 2
                    error = cx - image_center

                    angle = self.KP * error
                    angle = max(-self.MAX_ANGLE, min(self.MAX_ANGLE, angle))

                    steering = Float32()
                    steering.data = float(angle)
                    self.steering_pub.publish(steering)

                    cmd = String()
                    cmd.data = 'FORWARD'
                    self.cmd_pub.publish(cmd)

                    self.get_logger().info(f'Error: {error} Angle: {angle:.1f}')
                    return

        cmd = String()
        cmd.data = 'STOP'
        self.cmd_pub.publish(cmd)
        self.get_logger().warn('Line lost - stopping!')

    def destroy_node(self):
        cmd = String()
        cmd.data = 'STOP'
        self.cmd_pub.publish(cmd)
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
