import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import csv
import os
import time
from collections import deque
from datetime import datetime

TRAINING_DIR = os.path.expanduser('~/ros2_ws/training_data')

class DataCollectorNode(Node):
    """
    Records teleop driving sessions to CSV for behavior cloning.
    Drive the car with WASD while this node is recording.

    Topics listened to:
      /distance        - sonar (Float32)
      /detections      - YOLO objects (JSON String)
      /car_command     - motor command from teleop (String: FORWARD:N / BACKWARD:N / STOP)
      /steering_angle  - servo angle from teleop (Float32)
      /collector_cmd   - START / STOP recording (String)

    CSV columns:
      sonar_cm, sonar_trend, obj_left, obj_center, obj_right,
      person_present, speed, steering
    """

    RECORD_HZ = 20
    SONAR_TREND_WINDOW = 5

    def __init__(self):
        super().__init__('data_collector_node')

        self.create_subscription(Float32, 'distance', self.distance_cb, 10)
        self.create_subscription(String, 'detections', self.detections_cb, 10)
        self.create_subscription(String, 'car_command', self.command_cb, 10)
        self.create_subscription(Float32, 'steering_angle', self.steering_cb, 10)
        self.create_subscription(String, 'collector_cmd', self.collector_cmd_cb, 10)

        self.distance = 0.0
        self.distance_history = deque(maxlen=self.SONAR_TREND_WINDOW)
        self.detections = []
        self.current_speed = 0.0       # parsed from FORWARD:N / BACKWARD:N
        self.current_steering = 0.0

        self.recording = False
        self.csv_writer = None
        self.csv_file = None
        self.sample_count = 0
        self.session_start = None

        self.create_timer(1.0 / self.RECORD_HZ, self.record_sample)
        self.create_timer(5.0, self.print_stats)

        self.get_logger().info('Data collector ready. Send START to /collector_cmd to begin.')

    # ── callbacks ────────────────────────────────────────────────────

    def distance_cb(self, msg):
        self.distance = msg.data
        if msg.data > 0:
            self.distance_history.append(msg.data)

    def detections_cb(self, msg):
        try:
            self.detections = json.loads(msg.data)
        except Exception:
            pass

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

    def collector_cmd_cb(self, msg):
        if msg.data == 'START' and not self.recording:
            self._open_csv()
            self.recording = True
            self.session_start = time.time()
            self.get_logger().info(f'Recording started → {self.csv_path}')
        elif msg.data == 'STOP' and self.recording:
            self._close_csv()

    # ── feature extraction ────────────────────────────────────────────

    def _sonar_trend(self):
        h = list(self.distance_history)
        if len(h) < self.SONAR_TREND_WINDOW:
            return 0
        delta = h[-1] - h[0]
        if delta < -8:
            return -1   # approaching wall
        if delta > 8:
            return 1    # opening up
        return 0

    def _object_counts(self):
        left = center = right = 0
        person = 0
        for d in self.detections:
            if d.get('confidence', 0) < 0.4:
                continue
            if d.get('label') == 'person' and d.get('confidence', 0) > 0.5:
                person = 1
            bbox = d.get('bbox')
            if bbox and len(bbox) == 4:
                cx = (bbox[1] + bbox[3]) / 2
                if cx < 0.33:
                    left += 1
                elif cx > 0.66:
                    right += 1
                else:
                    center += 1
        return min(left, 5), min(center, 5), min(right, 5), person

    # ── recording ─────────────────────────────────────────────────────

    def record_sample(self):
        if not self.recording:
            return

        obj_left, obj_center, obj_right, person = self._object_counts()
        trend = self._sonar_trend()

        row = {
            'sonar_cm':      round(self.distance, 1),
            'sonar_trend':   trend,
            'obj_left':      obj_left,
            'obj_center':    obj_center,
            'obj_right':     obj_right,
            'person_present': person,
            'speed':         round(self.current_speed, 1),
            'steering':      round(self.current_steering, 1),
        }
        self.csv_writer.writerow(row)
        self.sample_count += 1

    def print_stats(self):
        if not self.recording:
            return
        elapsed = time.time() - self.session_start
        self.get_logger().info(
            f'Recording: {self.sample_count} samples | {elapsed:.0f}s | '
            f'sonar={self.distance:.0f}cm | speed={self.current_speed:.0f} | '
            f'steer={self.current_steering:.0f}°'
        )

    def _open_csv(self):
        os.makedirs(TRAINING_DIR, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(TRAINING_DIR, f'session_{stamp}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        fields = ['sonar_cm', 'sonar_trend', 'obj_left', 'obj_center',
                  'obj_right', 'person_present', 'speed', 'steering']
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fields)
        self.csv_writer.writeheader()
        self.sample_count = 0

    def _close_csv(self):
        self.recording = False
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
        elapsed = time.time() - self.session_start
        self.get_logger().info(
            f'Recording stopped. {self.sample_count} samples saved in {elapsed:.0f}s → {self.csv_path}'
        )

    def destroy_node(self):
        if self.recording:
            self._close_csv()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
