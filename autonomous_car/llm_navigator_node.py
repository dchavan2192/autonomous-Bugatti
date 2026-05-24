import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import re
import random
import requests
import threading
import time
from collections import deque

class LLMNavigatorNode(Node):

    OLLAMA_URL = "http://localhost:11434/api/generate"
    MODEL = "qwen2.5:1.5b"
    DECISION_INTERVAL = 8.0
    SAFETY_STOP_CM = 35.0
    REVERSE_SPEED = 140
    REVERSE_DURATION = 1.8
    REVERSE_TRIGGER_DELAY = 1.0
    TURN_SPEED = 110
    TURN_DURATION = 1.6   # seconds to turn after reversing

    def __init__(self):
        super().__init__('llm_navigator_node')

        self.cmd_pub = self.create_publisher(String, 'car_command', 10)
        self.steering_pub = self.create_publisher(Float32, 'steering_angle', 10)
        self.status_pub = self.create_publisher(String, 'llm_status', 10)

        self.create_subscription(Float32, 'distance', self.distance_callback, 10)
        self.create_subscription(String, 'detections', self.detections_callback, 10)
        self.create_subscription(String, 'llm_cmd', self.llm_cmd_callback, 10)

        self.distance = 999.0
        self.distance_history = deque(maxlen=6)  # for trend detection
        self.detections = []
        self.running = False
        self.last_decision_time = 0
        self.current_speed = 0
        self.current_steering = 0.0
        self.thinking = False

        # State machine: normal → stopped → reversing → turning → normal
        self.safety_state = 'normal'
        self.safety_stop_time = None
        self.reverse_start_time = None
        self.turn_start_time = None
        self.post_reverse_turn_angle = 0.0

        self.last_reason = 'waiting for first decision'
        self.last_scene = ''

        self.create_timer(0.1, self.execute_current_command)
        self.get_logger().info('LLM Navigator ready! Send START to /llm_cmd')

    def llm_cmd_callback(self, msg):
        if msg.data == 'START':
            self.running = True
            self.safety_state = 'normal'
            self.get_logger().info('LLM Navigation started!')
        elif msg.data == 'STOP':
            self.running = False
            self.safety_state = 'normal'
            self.send_drive('STOP')
            self.send_steering(0.0)

    def distance_callback(self, msg):
        self.distance = msg.data
        if msg.data > 0:
            self.distance_history.append(msg.data)

    def detections_callback(self, msg):
        try:
            self.detections = json.loads(msg.data)
        except Exception:
            pass

    def send_drive(self, command):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)

    def send_steering(self, angle):
        angle = max(-90.0, min(90.0, float(angle)))
        self.current_steering = angle
        msg = Float32()
        msg.data = angle
        self.steering_pub.publish(msg)

    def publish_status(self):
        status = String()
        status.data = (
            f"[{self.safety_state}] "
            f"sonar={self.distance:.0f}cm | "
            f"speed={self.current_speed} steer={self.current_steering:.0f}° | "
            f'LLM: "{self.last_reason}"'
        )
        self.status_pub.publish(status)

    def _sonar_trend(self):
        """Returns 'decreasing', 'increasing', or 'stable' based on recent history."""
        if len(self.distance_history) < 4:
            return 'stable'
        recent = list(self.distance_history)[-4:]
        delta = recent[-1] - recent[0]
        if delta < -10:
            return 'decreasing'
        if delta > 10:
            return 'increasing'
        return 'stable'

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
        """Turn toward the side with fewer detected objects."""
        left_count = sum(
            1 for d in self.detections
            if d.get('confidence', 0) > 0.4 and 'bbox' in d
            and (d['bbox'][1] + d['bbox'][3]) / 2 < 0.5
        )
        right_count = sum(
            1 for d in self.detections
            if d.get('confidence', 0) > 0.4 and 'bbox' in d
            and (d['bbox'][1] + d['bbox'][3]) / 2 >= 0.5
        )
        if left_count == right_count:
            return random.choice([-65.0, 65.0])
        return -65.0 if left_count < right_count else 65.0

    def build_prompt(self):
        left_objects, center_objects, right_objects = [], [], []

        for det in self.detections:
            if det.get('confidence', 0) > 0.5 and 'bbox' in det:
                cx = (det['bbox'][1] + det['bbox'][3]) / 2
                if cx < 0.33:
                    left_objects.append(det['label'])
                elif cx > 0.66:
                    right_objects.append(det['label'])
                else:
                    center_objects.append(det['label'])

        scene_parts = []
        if left_objects:
            scene_parts.append(f"{', '.join(set(left_objects))} on left")
        if center_objects:
            scene_parts.append(f"{', '.join(set(center_objects))} ahead")
        if right_objects:
            scene_parts.append(f"{', '.join(set(right_objects))} on right")
        if not scene_parts:
            scene_parts.append("clear path")

        scene_str = ', '.join(scene_parts)
        sonar_str = f"{self.distance:.0f}cm" if 0 < self.distance < 500 else "clear"
        trend = self._sonar_trend()
        if trend == 'decreasing' and 0 < self.distance < 200:
            sonar_str += " (approaching wall)"
        elif trend == 'increasing':
            sonar_str += " (opening up)"

        self.last_scene = f"scene={scene_str} sonar={sonar_str}"

        return f"""You are a robot car. Pick one action letter. Reply with ONLY the letter, nothing else.

Scene: {scene_str}
Sonar: {sonar_str}

A = go straight (path clear, sonar > 80cm)
B = turn left (obstacle or object on right side)
C = turn right (obstacle or object on left side)
D = stop (sonar < 40cm, approaching wall, or person detected)

Letter:"""

    def _parse_action(self, text):
        """Extract A/B/C/D from LLM response. Returns (steering, speed, reason) or None."""
        for ch in text.upper():
            if ch in ('A', 'B', 'C', 'D'):
                if ch == 'A':
                    return 0, 140, 'go straight'
                if ch == 'B':
                    return -50, 110, 'turn left'
                if ch == 'C':
                    return 50, 110, 'turn right'
                if ch == 'D':
                    return 0, 0, 'stop'
        return None

    def _apply_safety_overrides(self, steering, speed):
        if self._person_in_center():
            self.get_logger().warn('Safety: person centered — forcing stop')
            return steering, 0
        if 0 < self.distance < 60:
            ratio = max(0.0, (self.distance - self.SAFETY_STOP_CM) / (60.0 - self.SAFETY_STOP_CM))
            speed = int(speed * ratio)
        return steering, max(0, speed)

    def ask_llm(self):
        self.thinking = True
        try:
            prompt = self.build_prompt()
            self.get_logger().info('Querying LLM...')

            response = requests.post(self.OLLAMA_URL, json={
                "model": self.MODEL,
                "prompt": prompt,
                "stream": False,
                "options": {"temperature": 0.1, "top_p": 0.9}
            }, timeout=15)

            raw = response.json().get('response', '').strip()
            result = self._parse_action(raw)

            if result is None:
                self.get_logger().warn(f'LLM gave no valid action: "{raw[:60]}"')
                return

            steering, speed, reason = result
            steering, speed = self._apply_safety_overrides(steering, speed)
            self.last_reason = reason
            self.get_logger().info(
                f'LLM: {raw.strip()[:3]} → steer={steering} speed={speed} | {self.last_scene}'
            )
            self.current_speed = speed
            self.current_steering = float(steering)

        except requests.Timeout:
            self.get_logger().warn('LLM request timed out')
        except Exception as e:
            self.get_logger().warn(f'LLM error: {e}')
        finally:
            self.thinking = False

    def execute_current_command(self):
        if not self.running:
            return

        now = time.time()

        # --- Safety state machine ---

        if self.safety_state == 'stopped':
            self.send_drive('STOP')
            self.send_steering(0.0)
            self.publish_status()
            if self.distance >= self.SAFETY_STOP_CM or self.distance == 0:
                self.safety_state = 'normal'
                self.get_logger().info('Obstacle cleared — resuming')
            elif now - self.safety_stop_time > self.REVERSE_TRIGGER_DELAY:
                self.safety_state = 'reversing'
                self.reverse_start_time = now
                self.current_speed = 0
                self.get_logger().warn('Obstacle persists — reversing')
            return

        if self.safety_state == 'reversing':
            elapsed = now - self.reverse_start_time
            if elapsed < self.REVERSE_DURATION:
                self.send_drive(f'BACKWARD:{self.REVERSE_SPEED}')
                self.send_steering(0.0)
                self.publish_status()
            else:
                self.post_reverse_turn_angle = self._choose_turn_direction()
                self.turn_start_time = now
                self.safety_state = 'turning'
                self.get_logger().info(
                    f'Reverse done — turning {"left" if self.post_reverse_turn_angle < 0 else "right"} '
                    f'({self.post_reverse_turn_angle:.0f}°)'
                )
            return

        if self.safety_state == 'turning':
            elapsed = now - self.turn_start_time
            if elapsed < self.TURN_DURATION:
                self.send_drive(f'FORWARD:{self.TURN_SPEED}')
                self.send_steering(self.post_reverse_turn_angle)
                self.publish_status()
            else:
                self.send_steering(0.0)
                self.safety_state = 'normal'
                self.last_decision_time = 0  # trigger LLM query immediately
                self.get_logger().info('Turn complete — resuming LLM control')
            return

        # --- Normal operation ---

        if 0 < self.distance < self.SAFETY_STOP_CM:
            self.send_drive('STOP')
            self.send_steering(0.0)
            self.safety_state = 'stopped'
            self.safety_stop_time = now
            self.get_logger().warn(f'SAFETY STOP — sonar {self.distance:.0f}cm')
            return

        if not self.thinking and now - self.last_decision_time > self.DECISION_INTERVAL:
            self.last_decision_time = now
            threading.Thread(target=self.ask_llm, daemon=True).start()

        if self.current_speed > 0:
            self.send_drive(f'FORWARD:{self.current_speed}')
        else:
            self.send_drive('STOP')
        self.send_steering(self.current_steering)
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = LLMNavigatorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
