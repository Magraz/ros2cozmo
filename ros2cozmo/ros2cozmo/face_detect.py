#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from ros2cozmo_interfaces.msg import DetectionBox, DetectionBoxes
from cv_bridge import CvBridge

import cv2
import numpy as np

detector = cv2.FaceDetectorYN.create("/home/magraz/cozmo/cozmo_ros2_ws/src/ros2cozmo/ros2cozmo/face_detection_yunet_2022mar.onnx", "", (320, 240))

class FaceDetectSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(DetectionBoxes, '/faces_detected', 10)
        self.subscription = self.create_subscription(
            Image,
            '/cozmo/camera',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """ Handle new images, coming from the robot. """
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        image_array = np.asarray(cv_image)

        detector.setInputSize((image_array.shape[1],image_array.shape[0]))
        results = detector.detect(image_array)
        detection_data = results[1]

        if isinstance(detection_data, np.ndarray):

            detection_boxes_msg = DetectionBoxes()
            
            for detection in results[1]:
                xy_top_l_corner = detection[:2]
                d_box_width_height = detection[2:4]
                confidence = detection[-1]
            
                detection_box = DetectionBox()
                detection_box.confidence = int(confidence*100)
                detection_box.top_left_corner.x = float(xy_top_l_corner[0])
                detection_box.top_left_corner.y = float(xy_top_l_corner[1])
                detection_box.width = int(d_box_width_height[0])
                detection_box.height = int(d_box_width_height[1])

                detection_boxes_msg.detection_boxes.append(detection_box)

            self.publisher_.publish(detection_boxes_msg)

def main(args=None):
    rclpy.init(args=args)

    face_detect_sub = FaceDetectSubscriber()

    rclpy.spin(face_detect_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    face_detect_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
