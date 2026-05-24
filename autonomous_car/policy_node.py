import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import os
import pickle
import numpy as np
import time
from collections import deque

MODEL_PATH = os.path.expanduser('~/ros2_ws/trained_policy.pkl')

SONAR_MAX = 200.0
SPEED_MAX = 255.0
STEER_MAX = 90.0

SAFETY_STOP_CM   = 35.0
REVERSE_SPEED    = 130
REVERSE_DURATION = 1.8
REVERSE_TRIGGER  = 1.0
TURN_SPEED       = 100
TURN_DURATION    = 1.5


class PolicyNode(Node):
    """
    Runs the trained behavior cloning policy at 20 Hz.

    Send START to /policy_cmd to begin, STOP to halt.
    Hard safety overrides (sonar, person) are enforced in code —
    the model cannot override them.
    """

    def __init__(self):
        super().__init__('policy_node')

        self.cmd_pub      = self.create_publisher(String, 'car_command', 10)
        self.steering_pub = self.create_publisher(Float32, 'steering_angle', 10)
        self.status_pub   = self.create_publisher(String, 'policy_status', 10)

        self.create_subscription(Float32, 'distance', self.distance_cb, 10)
        self.create_subscription(String, 'detections', self.detections_cb, 10)
        self.create_subscription(String, 'policy_cmd', self.policy_cmd_cb, 10)

        self.distance = 0.0
        self.distance_history = deque(maxlen=5)
        self.detections = []
        self.running = False

        # 1-D Kalman filter for sonar distance
        self._kf_x = None   # state estimate (cm)
        self._kf_p = 50.0   # estimate uncertainty
        self._kf_q = 2.0    # process noise  (how fast distance can change)
        self._kf_r = 15.0   # measurement noise (sonar spec ~1-3cm, but spikes happen)

        # Safety state machine: normal → stopped → reversing → turning → normal
        self.safety_state    = 'normal'
        self.stop_time       = None
        self.reverse_time    = None
        self.turn_time       = None
        self.turn_angle      = 0.0

        self.last_speed    = 0.0
        self.last_steering = 0.0
        self.smoothed_steering = 0.0
        self.STEERING_ALPHA = 0.25  # lower = smoother but slower to respond

        self.model, self.scaler = self._load_model()
        self.create_timer(1.0 / 20.0, self.control_loop)
        self.get_logger().info('Policy node ready. Send START to /policy_cmd')

    # ── model loading ─────────────────────────────────────────────────

    def _load_model(self):
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f'Model not found at {MODEL_PATH} — run train_policy.py first')
            raise FileNotFoundError(MODEL_PATH)
        with open(MODEL_PATH, 'rb') as f:
            payload = pickle.load(f)
        self.get_logger().info('Trained policy loaded.')
        return payload['model'], payload['scaler']

    # ── subscriptions ─────────────────────────────────────────────────

    def distance_cb(self, msg):
        raw = msg.data
        if raw > 0:
            self.distance = self._kalman_update(raw)
            self.distance_history.append(self.distance)
        # raw == 0 means out of range — keep last filtered estimate

    def _kalman_update(self, measurement):
        if self._kf_x is None:
            self._kf_x = measurement  # first reading initialises the filter
            return measurement

        # Predict
        x_pred = self._kf_x
        p_pred = self._kf_p + self._kf_q

        # Update
        k = p_pred / (p_pred + self._kf_r)          # Kalman gain
        self._kf_x = x_pred + k * (measurement - x_pred)
        self._kf_p = (1 - k) * p_pred

        return self._kf_x

    def detections_cb(self, msg):
        try:
            self.detections = json.loads(msg.data)
        except Exception:
            pass

    def policy_cmd_cb(self, msg):
        if msg.data == 'START':
            self.running = True
            self.safety_state = 'normal'
            self.get_logger().info('Policy started!')
        elif msg.data == 'STOP':
            self.running = False
            self.safety_state = 'normal'
            self._drive('STOP')
            self._steer(0.0)

    # ── helpers ───────────────────────────────────────────────────────

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

    def _person_in_center(self):
        for d in self.detections:
            if d.get('label') != 'person' or d.get('confidence', 0) < 0.6:
                continue
            bbox = d.get('bbox')
            if bbox and len(bbox) == 4:
                cx = (bbox[1] + bbox[3]) / 2
                if 0.25 < cx < 0.75:
                    return True
        return False

    def _choose_turn_direction(self):
        left = sum(
            1 for d in self.detections
            if d.get('confidence', 0) > 0.4 and 'bbox' in d
            and (d['bbox'][1] + d['bbox'][3]) / 2 < 0.5
        )
        right = sum(
            1 for d in self.detections
            if d.get('confidence', 0) > 0.4 and 'bbox' in d
            and (d['bbox'][1] + d['bbox'][3]) / 2 >= 0.5
        )
        import random
        if left == right:
            return random.choice([-60.0, 60.0])
        return -60.0 if left < right else 60.0

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

    def _publish_status(self, state_label):
        msg = String()
        msg.data = (
            f'[{self.safety_state}] sonar={self.distance:.0f}cm | '
            f'speed={self.last_speed:.0f} steer={self.last_steering:.0f}° | '
            f'{state_label}'
        )
        self.status_pub.publish(msg)

    # ── inference ─────────────────────────────────────────────────────

    def _infer(self):
        obj_l, obj_c, obj_r, person = self._object_counts()
        trend = self._sonar_trend()

        sonar_norm = min(self.distance if self.distance > 0 else SONAR_MAX, SONAR_MAX) / SONAR_MAX
        trend_norm = (trend + 1) / 2.0

        features = np.array([[
            sonar_norm, trend_norm,
            obj_l / 5.0, obj_c / 5.0, obj_r / 5.0,
            float(person)
        ]], dtype=np.float32)

        features_s = self.scaler.transform(features)
        pred = self.model.predict(features_s)[0]

        # Model controls steering only — speed is rule-based (longitudinal/lateral split)
        steering = float(np.clip(pred[1], -1.0, 1.0) * STEER_MAX)
        speed = self._rule_based_speed()

        return speed, steering

    def _rule_based_speed(self):
        """Longitudinal controller — sonar-based, independent of learned policy."""
        if self._person_in_center():
            return 0.0
        d = self.distance
        if d == 0 or d > 120:
            return 160.0   # clear — cruise
        if d > 80:
            return 130.0   # opening up — steady
        if d > SAFETY_STOP_CM:
            # linear ramp from 130 down to 0 between 80cm and stop distance
            ratio = (d - SAFETY_STOP_CM) / (80.0 - SAFETY_STOP_CM)
            return max(0.0, 130.0 * ratio)
        return 0.0

    # ── control loop ──────────────────────────────────────────────────

    def control_loop(self):
        if not self.running:
            return

        now = time.time()

        # Safety state machine
        if self.safety_state == 'stopped':
            self._drive('STOP')
            self._steer(0.0)
            self._publish_status('safety stop')
            if self.distance >= SAFETY_STOP_CM or self.distance == 0:
                self.safety_state = 'normal'
                self.get_logger().info('Obstacle cleared')
            elif now - self.stop_time > REVERSE_TRIGGER:
                self.safety_state = 'reversing'
                self.reverse_time = now
                self.get_logger().warn('Reversing...')
            return

        if self.safety_state == 'reversing':
            if now - self.reverse_time < REVERSE_DURATION:
                self._drive(f'BACKWARD:{REVERSE_SPEED}')
                self._steer(0.0)
                self.last_speed = -REVERSE_SPEED
                self._publish_status('reversing')
            else:
                self.turn_angle = self._choose_turn_direction()
                self.turn_time = now
                self.safety_state = 'turning'
                self.get_logger().info(f'Turning {"left" if self.turn_angle < 0 else "right"}')
            return

        if self.safety_state == 'turning':
            if now - self.turn_time < TURN_DURATION:
                self._drive(f'FORWARD:{TURN_SPEED}')
                self._steer(self.turn_angle)
                self.last_speed = TURN_SPEED
                self._publish_status('post-reverse turn')
            else:
                self._steer(0.0)
                self.smoothed_steering = 0.0  # reset so EMA starts fresh
                self.safety_state = 'normal'
                self.get_logger().info('Resuming policy')
            return

        # Hard sonar stop
        if 0 < self.distance < SAFETY_STOP_CM:
            self._drive('STOP')
            self._steer(0.0)
            self.last_speed = 0.0
            self.safety_state = 'stopped'
            self.stop_time = now
            self.get_logger().warn(f'SAFETY STOP — {self.distance:.0f}cm')
            return

        # Run policy
        speed, steering = self._infer()
        self.last_speed = speed

        # Smooth steering with exponential moving average
        self.smoothed_steering = (
            self.STEERING_ALPHA * steering +
            (1.0 - self.STEERING_ALPHA) * self.smoothed_steering
        )

        if speed > 5:
            self._drive(f'FORWARD:{int(speed)}')
        else:
            self._drive('STOP')
        self._steer(self.smoothed_steering)
        self._publish_status('policy')


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
