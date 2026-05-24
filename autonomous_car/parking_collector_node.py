import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import csv
import os
import time
from collections import deque
from datetime import datetime

TRAINING_DIR = os.path.expanduser('~/ros2_ws/parking_data')

class ParkingCollectorNode(Node):
    """
    Records teleop driving toward the parking spot.

    Drive from outside the spot, align, and pull in — press Q when parked.
    Each demo is one START→STOP cycle. Do 30-50 demos.

    Publishes stats every 5s. CSV saved to ~/ros2_ws/parking_data/
    """

    RECORD_HZ = 20

    def __init__(self):
        super().__init__('parking_collector_node')

        self.create_subscription(String, 'parking_spot', self.spot_cb, 10)
        self.create_subscription(Float32, 'distance', self.distance_cb, 10)
        self.create_subscription(String, 'car_command', self.command_cb, 10)
        self.create_subscription(Float32, 'steering_angle', self.steering_cb, 10)
        self.create_subscription(String, 'parking_collector_cmd', self.cmd_cb, 10)

        self.spot = {'visible': 0, 'cx': 0.0, 'cy': 0.0, 'area': 0.0}
        self.distance = 0.0
        self.distance_history = deque(maxlen=5)
        self.current_speed = 0.0
        self.current_steering = 0.0

        self.recording = False
        self.csv_writer = None
        self.csv_file = None
        self.sample_count = 0
        self.demo_count = 0
        self.session_start = None
        self.csv_path = None

        self.create_timer(1.0 / self.RECORD_HZ, self.record_sample)
        self.create_timer(5.0, self.print_stats)

        self.get_logger().info('Parking collector ready.')
        self.get_logger().info('Send START to /parking_collector_cmd, drive to spot, send STOP.')

    def spot_cb(self, msg):
        try:
            self.spot = json.loads(msg.data)
        except Exception:
            pass

    def distance_cb(self, msg):
        self.distance = msg.data
        if msg.data > 0:
            self.distance_history.append(msg.data)

    def command_cb(self, msg):
        raw = msg.data.strip()
        if raw == 'STOP':
            self.current_speed = 0.0
        elif raw.startswith('FORWARD:'):
            try:
                self.current_speed = float(raw.split(':')[1])
            except ValueError:
                pass
        elif raw.startswith('BACKWARD:'):
            try:
                self.current_speed = -float(raw.split(':')[1])
            except ValueError:
                pass

    def steering_cb(self, msg):
        self.current_steering = msg.data

    def cmd_cb(self, msg):
        if msg.data == 'START' and not self.recording:
            if self.csv_file is None:
                self._open_csv()
            self.recording = True
            self.demo_count += 1
            self.get_logger().info(f'Demo {self.demo_count} started — drive to spot then STOP')
        elif msg.data == 'STOP' and self.recording:
            self.recording = False
            self.get_logger().info(
                f'Demo {self.demo_count} saved ({self.sample_count} total samples so far)')
        elif msg.data == 'SAVE':
            self._close_csv()

    def _sonar_trend(self):
        h = list(self.distance_history)
        if len(h) < 4:
            return 0
        delta = h[-1] - h[0]
        if delta < -8:
            return -1
        if delta > 8:
            return 1
        return 0

    def record_sample(self):
        if not self.recording:
            return

        row = {
            'spot_visible':  self.spot.get('visible', 0),
            'spot_cx':       round(self.spot.get('cx', 0.0), 3),
            'spot_cy':       round(self.spot.get('cy', 0.0), 3),
            'spot_area':     round(self.spot.get('area', 0.0), 3),
            'sonar_cm':      round(self.distance, 1),
            'sonar_trend':   self._sonar_trend(),
            'speed':         round(self.current_speed, 1),
            'steering':      round(self.current_steering, 1),
        }
        self.csv_writer.writerow(row)
        self.sample_count += 1

    def print_stats(self):
        if not self.recording and self.sample_count == 0:
            return
        self.get_logger().info(
            f'Demos: {self.demo_count} | Samples: {self.sample_count} | '
            f'spot={"YES" if self.spot.get("visible") else "NO"} '
            f'cx={self.spot.get("cx", 0):.2f} area={self.spot.get("area", 0):.3f} | '
            f'speed={self.current_speed:.0f} steer={self.current_steering:.0f}°'
        )

    def _open_csv(self):
        os.makedirs(TRAINING_DIR, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(TRAINING_DIR, f'parking_{stamp}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        fields = ['spot_visible', 'spot_cx', 'spot_cy', 'spot_area',
                  'sonar_cm', 'sonar_trend', 'speed', 'steering']
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fields)
        self.csv_writer.writeheader()
        self.sample_count = 0
        self.session_start = time.time()
        self.get_logger().info(f'CSV opened: {self.csv_path}')

    def _close_csv(self):
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.get_logger().info(
                f'Saved {self.sample_count} samples from {self.demo_count} demos → {self.csv_path}')

    def destroy_node(self):
        if self.recording or self.csv_file:
            self._close_csv()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ParkingCollectorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
