import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from ros2cozmo_interfaces.msg import DetectionBox, DetectionBoxes
from cv_bridge import CvBridge

import cv2
import numpy as np


class FaceDetectNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, namespace=node_name)

        self.faces_detected_pub = self.create_publisher(
            DetectionBoxes, "faces_detected", 10
        )
        self.main_face_pub = self.create_publisher(DetectionBox, "main_face", 10)

        self.subscription = self.create_subscription(
            Image, "/cozmo/camera", self.listener_callback, 10
        )
        self.detector = cv2.FaceDetectorYN.create(
            "/home/magraz/cozmo/ros2cozmo_ws/src/ros2cozmo/data/face_detection_yunet_2022mar.onnx",
            "",
            (320, 240),
        )

    def listener_callback(self, msg):
        """Handle new images, coming from the robot."""
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        image_array = np.asarray(cv_image)

        self.detector.setInputSize((image_array.shape[1], image_array.shape[0]))
        results = self.detector.detect(image_array)
        detection_data = results[1]

        if isinstance(detection_data, np.ndarray):
            detection_boxes_msg = DetectionBoxes()
            main_box_msg = DetectionBox()

            for detection in results[1]:
                xy_top_l_corner = detection[:2]
                d_box_width_height = detection[2:4]
                confidence = detection[-1]

                detection_box = DetectionBox()
                detection_box.confidence = int(confidence * 100)
                detection_box.top_left_corner.x = float(xy_top_l_corner[0])
                detection_box.top_left_corner.y = float(xy_top_l_corner[1])
                detection_box.width = int(d_box_width_height[0])
                detection_box.height = int(d_box_width_height[1])

                detection_boxes_msg.detection_boxes.append(detection_box)

                # Select primary box
                if (main_box_msg.width * main_box_msg.height) < (
                    detection_box.width * detection_box.height
                ):
                    main_box_msg = detection_box

            self.faces_detected_pub.publish(detection_boxes_msg)
            self.main_face_pub.publish(main_box_msg)


def main(args=None):
    rclpy.init(args=args)

    node = FaceDetectNode(node_name="face_detect")

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
