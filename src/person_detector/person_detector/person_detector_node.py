import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge
from ultralytics import YOLO

import numpy as np


class PersonDetector(Node):

    def __init__(self):
        super().__init__('person_detector')

        self.model = YOLO("yolov8n.pt")

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.depth_image = None

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.alpha = 0.3
        self.smooth_x = 0.0
        self.smooth_y = 0.0

        self.last_time = self.get_clock().now()
        self.min_period = 0.1  # 10 Hz

        self.fx = 570.3405082258201
        self.fy = 570.3405082258201
        self.ppx = 319.5
        self.ppy = 239.5

        self.get_logger().info("YOLO person detector READY")

    def image_callback(self, msg):

        if self.depth_image is None:
            return

        now = self.get_clock().now()
        if (now - self.last_time).nanoseconds < self.min_period * 1e9:
            return
        self.last_time = now

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.model(frame, verbose=False)[0]

        best_box = None
        best_area = 0

        for box in results.boxes:
            cls = int(box.cls[0])
            name = self.model.names[cls]

            if name != "person":
                continue

            if float(box.conf[0]) < 0.5:
                continue

            x1, y1, x2, y2 = box.xyxy[0]

            area = (x2 - x1) * (y2 - y1)

            if area > best_area:
                best_area = area
                best_box = (x1, y1, x2, y2)

        if best_box is None:
            return

        x1, y1, x2, y2 = best_box

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        h, w = self.depth_image.shape[:2]

        if not (0 <= cx < w and 0 <= cy < h):
            return

        roi = self.depth_image[
            max(0, cy - 5):min(h, cy + 5),
            max(0, cx - 5):min(w, cx + 5)
        ]

        valid = roi[roi > 0]

        if len(valid) == 0:
            return

        depth = float(np.median(valid))

        # RealSense -> mm
        if depth > 20:  # mm vs m
            depth = depth / 1000.0

        X = (cx - self.ppx) * depth / self.fx
        Y = (cy - self.ppy) * depth / self.fy
        Z = depth

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "camera_link"

        pose.pose.position.x = float(Z)
        pose.pose.position.y = float(-X)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        self.pub.publish(pose)

        self.get_logger().info(
            f"Person: Z={Z:.2f}m, X={X:.2f}m"
        )


    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )


def main():
    rclpy.init()
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()