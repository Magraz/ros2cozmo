import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ros2cozmo_interfaces.msg import DetectionBox, DetectionBoxes
from cv_bridge import CvBridge

import cv2
import numpy as np


class ActionManagerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, namespace=node_name)
        self.subscription = self.create_subscription(
            String, "/action", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # match(msg.data):
        #     case 'track_face':
        #     case 'approach_face':
        #     case 'idle':
        pass


def main(args=None):
    rclpy.init(args=args)

    node = ActionManagerNode(node_name="action_manager")

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
