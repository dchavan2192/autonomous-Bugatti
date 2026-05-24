import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import os
import pickle
import numpy as np
import time
from collections import deque

MODEL_PATH = os.path.expanduser('~/ros2_ws/trained_parking.pkl')

SONAR_MAX       = 200.0
STEER_MAX       = 90.0
SAFETY_STOP_CM  = 25.0   # tighter for parking — we want to get close
PARKED_AREA     = 0.35   # spot fills 35% of frame → we're inside it
APPROACH_SPEED  = 100    # slow and controlled
ALIGN_SPEED     = 80     # even slower while steering to align
STEERING_ALPHA  = 0.3    # EMA smoothing


class ParkingPolicyNode(Node):
    """
    Runs the trained parking policy.

    Phases (automatic):
      SEARCHING  — spot not visible, creep forward slowly
      ALIGNING   — spot visible, steer toward center
      PARKING    — spot centered, drive in
      PARKED     — spot area large enough, stop

    Send START/STOP to /parking_cmd.
    """

    def __init__(self):
        super().__init__('parking_policy_node')

        self.cmd_pub      = self.create_publisher(String, 'car_command', 10)
        self.steering_pub = self.create_publisher(Float32, 'steering_angle', 10)
        self.status_pub   = self.create_publisher(String, 'parking_status', 10)

        self.create_subscription(String, 'parking_spot', self.spot_cb, 10)
        self.create_subscription(Float32, 'distance', self.distance_cb, 10)
        self.create_subscription(String, 'parking_cmd', self.cmd_cb, 10)

        self.spot     = {'visible': 0, 'cx': 0.0, 'cy': 0.0, 'area': 0.0}
        self.distance = 0.0
        self.distance_history = deque(maxlen=5)
        self.running  = False
        self.phase    = 'idle'

        # Kalman for sonar
        self._kf_x = None
        self._kf_p = 50.0
        self._kf_q = 2.0
        self._kf_r = 15.0

        self.smoothed_steering = 0.0
        self.last_speed = 0.0
        self.last_steering = 0.0

        self.model, self.scaler = self._load_model()
        self.create_timer(1.0 / 20.0, self.control_loop)
        self.get_logger().info('Parking policy ready. Send START to /parking_cmd.')

    def _load_model(self):
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f'No model at {MODEL_PATH} — run train_parking.py first')
            raise FileNotFoundError(MODEL_PATH)
        with open(MODEL_PATH, 'rb') as f:
            p = pickle.load(f)
        self.get_logger().info('Parking model loaded.')
        return p['model'], p['scaler']

    def spot_cb(self, msg):
        try:
            self.spot = json.loads(msg.data)
        except Exception:
            pass

    def distance_cb(self, msg):
        if msg.data > 0:
            self.distance = self._kalman(msg.data)
            self.distance_history.append(self.distance)

    def cmd_cb(self, msg):
        if msg.data == 'START':
            self.running = True
            self.phase = 'searching'
            self.smoothed_steering = 0.0
            self.get_logger().info('Parking started.')
        elif msg.data == 'STOP':
            self.running = False
            self.phase = 'idle'
            self._drive('STOP')
            self._steer(0.0)

    def _drive(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def _steer(self, angle):
        angle = max(-90.0, min(90.0, float(angle)))
        self.last_steering = angle
        msg = Float32()
        msg.data = angle
        self.steering_pub.publish(msg)

    def _kalman(self, z):
        if self._kf_x is None:
            self._kf_x = z
            return z
        x = self._kf_x
        p = self._kf_p + self._kf_q
        k = p / (p + self._kf_r)
        self._kf_x = x + k * (z - x)
        self._kf_p = (1 - k) * p
        return self._kf_x

    def _sonar_trend(self):
        h = list(self.distance_history)
        if len(h) < 4:
            return 0
        d = h[-1] - h[0]
        return -1 if d < -8 else (1 if d > 8 else 0)

    def _infer_steering(self):
        visible = float(self.spot.get('visible', 0))
        cx      = float(self.spot.get('cx', 0.0))
        cy      = float(self.spot.get('cy', 0.0))
        area    = float(self.spot.get('area', 0.0))
        sonar_norm = min(self.distance if self.distance > 0 else SONAR_MAX, SONAR_MAX) / SONAR_MAX
        trend_norm = (self._sonar_trend() + 1) / 2.0

        features = np.array([[visible, cx, cy, area, sonar_norm, trend_norm]], dtype=np.float32)
        features_s = self.scaler.transform(features)
        pred = self.model.predict(features_s)[0]
        return float(np.clip(pred, -1.0, 1.0) * STEER_MAX)

    def control_loop(self):
        if not self.running:
            return

        visible = self.spot.get('visible', 0)
        cx      = self.spot.get('cx', 0.0)
        area    = self.spot.get('area', 0.0)

        # Hard sonar safety
        if 0 < self.distance < SAFETY_STOP_CM:
            self._drive('STOP')
            self._steer(0.0)
            self._publish(f'SAFETY STOP {self.distance:.0f}cm')
            return

        # Phase transitions
        if self.phase == 'searching':
            if visible:
                self.phase = 'aligning'
                self.get_logger().info('Spot found — aligning')

        if self.phase == 'aligning':
            if not visible:
                self.phase = 'searching'
            elif abs(cx) < 0.15 and area > 0.05:
                self.phase = 'parking'
                self.get_logger().info('Aligned — parking')

        if self.phase == 'parking':
            if area >= PARKED_AREA:
                self.phase = 'parked'
                self.get_logger().info('PARKED!')

        # Phase actions
        if self.phase == 'parked':
            self._drive('STOP')
            self._steer(0.0)
            self.running = False
            self._publish('PARKED')
            return

        if self.phase == 'searching':
            # Creep forward, no steering — wait for spot to appear
            self._drive(f'FORWARD:{APPROACH_SPEED}')
            self._steer(0.0)
            self._publish('searching')
            return

        # aligning or parking — run the learned policy
        raw_steer = self._infer_steering()
        self.smoothed_steering = (STEERING_ALPHA * raw_steer +
                                   (1 - STEERING_ALPHA) * self.smoothed_steering)

        speed = ALIGN_SPEED if self.phase == 'aligning' else APPROACH_SPEED
        self.last_speed = speed
        self._drive(f'FORWARD:{speed}')
        self._steer(self.smoothed_steering)
        self._publish(self.phase)

    def _publish(self, label):
        msg = String()
        msg.data = (
            f'[{label}] sonar={self.distance:.0f}cm | '
            f'spot={"YES" if self.spot.get("visible") else "NO"} '
            f'cx={self.spot.get("cx", 0):.2f} area={self.spot.get("area", 0):.3f} | '
            f'steer={self.last_steering:.0f}°'
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingPolicyNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
