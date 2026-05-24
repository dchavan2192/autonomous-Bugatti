import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import json

class ParkingDetectorNode(Node):
    """
    Detects a black-tape parking rectangle on a wooden floor.

    Subscribes: /camera/image_raw
    Publishes:
      /parking_spot  — JSON: {visible, cx, cy, area, angle}
      /parking_debug — annotated image for tuning
    """

    # Tuning — adjust if detection is unreliable
    DARK_THRESHOLD  = 80    # pixels below this value (0-255) = black tape
    MIN_AREA        = 800   # px² — individual tape lines are thin
    MAX_AREA        = 200000  # px² — ignore full-frame blobs
    FRAME_CROP_TOP  = 0.35  # ignore top 35% of frame (ceiling/walls, not floor)

    def __init__(self):
        super().__init__('parking_detector_node')

        self.create_subscription(Image, 'camera/image_raw', self.image_cb, 10)
        self.spot_pub  = self.create_publisher(String, 'parking_spot', 10)
        self.debug_pub = self.create_publisher(Image, 'parking_debug', 10)

        self.get_logger().info('Parking detector ready.')

    def image_cb(self, msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)

        spot, debug_frame = self._detect(frame)

        # Publish spot features
        out = String()
        out.data = json.dumps(spot)
        self.spot_pub.publish(out)

        # Publish debug image
        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height, debug_msg.width, _ = debug_frame.shape
        debug_msg.encoding = 'bgr8'
        debug_msg.step = debug_msg.width * 3
        debug_msg.data = debug_frame.tobytes()
        self.debug_pub.publish(debug_msg)

    def _detect(self, frame):
        h, w = frame.shape[:2]
        debug = frame.copy()

        # Crop to floor region
        crop_y = int(h * self.FRAME_CROP_TOP)
        roi = frame[crop_y:, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)

        # Threshold: black tape → white mask
        _, mask = cv2.threshold(blurred, self.DARK_THRESHOLD,
                                255, cv2.THRESH_BINARY_INV)

        # Clean up salt/pepper noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # Filter valid contours (individual tape lines can be elongated — no aspect check)
        valid = [c for c in contours
                 if self.MIN_AREA < cv2.contourArea(c) < self.MAX_AREA]

        cv2.line(debug, (0, crop_y), (w, crop_y), (0, 255, 255), 1)

        if not valid:
            cv2.putText(debug, 'NO SPOT', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return {'visible': 0, 'cx': 0.0, 'cy': 0.0, 'area': 0.0, 'angle': 0.0}, debug

        # Merge all valid tape contours → unified bounding box of the whole spot
        all_points = np.vstack([c for c in valid])
        rect = cv2.minAreaRect(all_points)   # rotated bounding rect of merged region
        box  = cv2.boxPoints(rect)
        box_shifted = (box + np.array([0, crop_y])).astype(np.int0)

        # Center of merged region
        cx_px = rect[0][0]
        cy_px = rect[0][1] + crop_y

        cx_norm   = (cx_px / w) * 2.0 - 1.0
        cy_norm   = cy_px / h
        rw, rh    = rect[1]
        area_norm = min((rw * rh) / (w * h * (1 - self.FRAME_CROP_TOP)), 1.0)
        angle     = rect[2]

        # Draw each tape line contour + merged bounding box
        for c in valid:
            shifted = c + np.array([0, crop_y])
            cv2.drawContours(debug, [shifted], -1, (0, 255, 0), 1)
        cv2.drawContours(debug, [box_shifted], 0, (0, 0, 255), 2)
        cv2.circle(debug, (int(cx_px), int(cy_px)), 8, (255, 0, 0), -1)
        cv2.putText(debug, f'cx={cx_norm:.2f} area={area_norm:.3f} n={len(valid)}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        spot = {
            'visible': 1,
            'cx':   round(cx_norm, 3),
            'cy':   round(cy_norm, 3),
            'area': round(area_norm, 3),
            'angle': round(angle, 1),
        }
        return spot, debug


def main(args=None):
    rclpy.init(args=args)
    node = ParkingDetectorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
