import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import time
import random

class ExplorerNode(Node):
    
    FAST_SPEED = 160
    SLOW_SPEED = 110
    REVERSE_SPEED = 140
    STOP_DISTANCE = 35.0
    SLOW_DISTANCE = 80.0
    REVERSE_DURATION = 2.0   # seconds — enough clearance at REVERSE_SPEED
    TURN_DURATION = 1.6      # seconds — ~90° heading change at SLOW_SPEED
    PERSON_DEBOUNCE = 0.8    # seconds person must be visible before reversing

    def __init__(self):
        super().__init__('explorer_node')
        
        self.cmd_pub = self.create_publisher(String, 'car_command', 10)
        self.steering_pub = self.create_publisher(Float32, 'steering_angle', 10)
        
        self.create_subscription(Float32, 'distance', self.distance_callback, 10)
        self.create_subscription(String, 'detections', self.detections_callback, 10)
        self.create_subscription(String, 'explorer_cmd', self.explorer_cmd_callback, 10)

        self.distance = 999.0
        self.detections = []
        self.spatial_memory = {'left': [], 'right': [], 'ahead': []}
        self.state = 'idle'
        self.reverse_start = None
        self.turn_start = None
        self.turn_direction = 0.0
        self.current_angle = 0.0
        self.person_detected_start = None

        self.create_timer(0.1, self.navigate)
        self.get_logger().info('Explorer node started! Send START to /explorer_cmd')

    def explorer_cmd_callback(self, msg):
        if msg.data == 'START':
            self.state = 'exploring'
            self.get_logger().info('Exploration started!')
        elif msg.data == 'STOP':
            self.state = 'idle'
            self.send_drive('STOP')
            self.send_steering(0.0)
            self.get_logger().info('Exploration stopped!')

    def distance_callback(self, msg):
        self.distance = msg.data

    def detections_callback(self, msg):
        try:
            self.detections = json.loads(msg.data)
            labels = [d['label'] for d in self.detections if d['confidence'] > 0.5]
            if self.current_angle < -20:
                self.spatial_memory['left'] = labels[-5:]
            elif self.current_angle > 20:
                self.spatial_memory['right'] = labels[-5:]
            else:
                self.spatial_memory['ahead'] = labels[-5:]
        except:
            pass

    def send_drive(self, command):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)

    def send_steering(self, angle):
        angle = max(-90.0, min(90.0, angle))
        self.current_angle = angle
        msg = Float32()
        msg.data = float(angle)
        self.steering_pub.publish(msg)

    def choose_turn_direction(self):
        left_objects = len(self.spatial_memory['left'])
        right_objects = len(self.spatial_memory['right'])
        if left_objects <= right_objects:
            return -70.0
        else:
            return 70.0

    def person_ahead(self):
        return any(d['label'] == 'person' and d['confidence'] > 0.7
                   for d in self.detections)

    def navigate(self):
        if self.state == 'idle':
            return

        now = time.time()

        if self.state == 'reversing':
            elapsed = now - self.reverse_start
            if elapsed < self.REVERSE_DURATION:
                self.send_drive(f'BACKWARD:{self.REVERSE_SPEED}')
                self.send_steering(0.0)
            else:
                self.turn_direction = self.choose_turn_direction()
                self.turn_start = now
                self.state = 'turning'
            return

        if self.state == 'turning':
            elapsed = now - self.turn_start
            if elapsed < self.TURN_DURATION:
                self.send_drive(f'FORWARD:{self.SLOW_SPEED}')
                self.send_steering(self.turn_direction)
            else:
                self.send_steering(0.0)
                self.state = 'exploring'
                self.spatial_memory = {'left': [], 'right': [], 'ahead': []}
            return

        # Sonar obstacle detection
        if self.distance > 0 and self.distance < self.STOP_DISTANCE:
            self.get_logger().warn(f'Sonar obstacle at {self.distance:.1f}cm - reversing!')
            self.reverse_start = now
            self.state = 'reversing'
            return

        # Person detection
        if self.person_ahead():
            if self.person_detected_start is None:
                self.person_detected_start = now
            elif now - self.person_detected_start > self.PERSON_DEBOUNCE:
                self.get_logger().warn('Person detected - reversing!')
                self.reverse_start = now
                self.state = 'reversing'
                self.person_detected_start = None
            return
        else:
            self.person_detected_start = None

        # Speed based on sonar distance
        if self.distance > 0 and self.distance < self.SLOW_DISTANCE:
            speed = self.SLOW_SPEED
        else:
            speed = self.FAST_SPEED

        # Random wander
        if random.random() < 0.05:
            wander = random.uniform(-15, 15)
            self.send_steering(wander)

        self.send_drive(f'FORWARD:{speed}')
        self.get_logger().info(f'Exploring - dist: {self.distance:.0f}cm speed: {speed} objects: {[d["label"] for d in self.detections if d["confidence"] > 0.5]}')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
