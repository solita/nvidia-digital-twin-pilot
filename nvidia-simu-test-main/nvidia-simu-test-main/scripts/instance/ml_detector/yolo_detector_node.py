#!/usr/bin/env python3
"""
yolo_detector_node.py — YOLOv8n ML object detector.

Subscribes to /camera/image/compressed (JPEG from Isaac Sim),
runs YOLOv8n inference on GPU, publishes detections to
/camera/detections_ml as JSON.

Runs in a dedicated container: yolo-detector.
"""

import json
import os
import time

import cv2
import numpy as np

# ── FastDDS SHM fix (must be set before any ROS2 import) ──────────────────────
_XML_PATH = "/tmp/fastdds_no_shm.xml"
if not os.path.exists(_XML_PATH):
    with open(_XML_PATH, "w") as f:
        f.write(
            '<?xml version="1.0" encoding="UTF-8" ?>\n'
            '<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">\n'
            '  <transport_descriptors>\n'
            '    <transport_descriptor>\n'
            '      <transport_id>udp_only</transport_id>\n'
            '      <type>UDPv4</type>\n'
            '      <sendBufferSize>1048576</sendBufferSize>\n'
            '      <receiveBufferSize>1048576</receiveBufferSize>\n'
            '    </transport_descriptor>\n'
            '  </transport_descriptors>\n'
            '  <participant profile_name="participant_profile" is_default_profile="true">\n'
            '    <rtps>\n'
            '      <useBuiltinTransports>false</useBuiltinTransports>\n'
            '      <userTransports>\n'
            '        <transport_id>udp_only</transport_id>\n'
            '      </userTransports>\n'
            '    </rtps>\n'
            '  </participant>\n'
            '</profiles>\n'
        )
os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = _XML_PATH

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.get_logger().info("Loading YOLOv8n model...")
        self.model = YOLO("yolov8n.pt")

        # Warm up with a dummy image to trigger CUDA init
        dummy = np.zeros((320, 480, 3), dtype=np.uint8)
        self.model.predict(dummy, verbose=False)
        device = next(self.model.model.parameters()).device
        self.get_logger().info(f"YOLOv8n ready on device: {device}")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            CompressedImage,
            "/camera/image/compressed",
            self._on_image,
            qos,
        )

        self.pub = self.create_publisher(String, "/camera/detections_ml", qos)

        self._count = 0
        self._skip = 3  # process every 3rd received frame
        self._published = 0

    def _on_image(self, msg):
        self._count += 1
        if self._count % self._skip != 0:
            return

        # Decode JPEG
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        # Run inference
        t0 = time.monotonic()
        results = self.model.predict(img, verbose=False, conf=0.20)
        dt_ms = (time.monotonic() - t0) * 1000

        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                detections.append({
                    "label": r.names[int(box.cls)],
                    "confidence": round(float(box.conf), 2),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                })

        out = String()
        out.data = json.dumps({
            "detections": detections,
            "width": img.shape[1],
            "height": img.shape[0],
            "model": "yolov8n",
            "inference_ms": round(dt_ms, 1),
        })
        self.pub.publish(out)
        self._published += 1

        if self._published == 1:
            self.get_logger().info(
                f"First inference: {len(detections)} detections in {dt_ms:.0f}ms"
            )


def main():
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
