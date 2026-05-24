import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import hailo_platform
from hailo_platform import VDevice, HEF, ConfigureParams, InputVStreamParams, OutputVStreamParams, FormatType, HailoStreamInterface
import json

COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
    'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
    'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.detections_pub = self.create_publisher(String, 'detections', 10)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        self.device = VDevice()
        hef = HEF('/home/dchavan2192/yolov8s.hef')
        configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        self.network_groups = self.device.configure(hef, configure_params)
        self.network_group = self.network_groups[0]
        self.network_group_params = self.network_group.create_params()

        self.input_vstreams_params = InputVStreamParams.make(self.network_group, format_type=FormatType.UINT8)
        self.output_vstreams_params = OutputVStreamParams.make(self.network_group, format_type=FormatType.FLOAT32)

        self.create_timer(0.05, self.process_frame)
        self.get_logger().info('Detection node started!')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Camera read failed!')
            return

        input_frame = cv2.resize(frame, (640, 640))
        input_frame = np.expand_dims(input_frame, axis=0)

        with self.network_group.activate(self.network_group_params):
            with hailo_platform.InferVStreams(self.network_group, self.input_vstreams_params, self.output_vstreams_params) as infer_pipeline:
                input_data = {self.network_group.get_input_vstream_infos()[0].name: input_frame}
                results = infer_pipeline.infer(input_data)
                output = list(results.values())[0][0]

                detections = []
                for class_id, class_detections in enumerate(output):
                    for det in class_detections:
                        if len(det) >= 5:
                            confidence = float(det[4])
                            if confidence > 0.3:
                                label = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else 'unknown'
                                detections.append({
                                    'label': label,
                                    'confidence': round(confidence, 2),
                                    'bbox': [float(det[0]), float(det[1]), float(det[2]), float(det[3])]
                                })

                if detections:
                    msg = String()
                    msg.data = json.dumps(detections)
                    self.detections_pub.publish(msg)
                    self.get_logger().info(f'Detected: {[d["label"] for d in detections]}')

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
