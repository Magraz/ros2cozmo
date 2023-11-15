import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from ros2cozmo_interfaces.msg import DetectionBox
from geometry_msgs.msg import Twist


class FaceApproachNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, namespace=node_name)

        self.enabled = False

        self.subscription = self.create_subscription(
            DetectionBox, "/face_detect/main_face", self.approach_face, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.enable_srv = self.create_service(Empty, "enable", self.enable_callback)
        self.disable_srv = self.create_service(Empty, "disable", self.disable_callback)

        self.subscription  # prevent unused variable warning

    def enable_callback(self, request, response):
        self.enabled = True
        print("Enable called")
        return response

    def disable_callback(self, request, response):
        self.enabled = False
        print("Disabled called")
        return response

    def approach_face(self, msg):
        twist_msg = Twist()
        # 8925 = 85*105 (place holder)

        if ((msg.width * msg.height) < 8925) and self.enabled:
            # 0.4 = placeholder speed
            twist_msg.linear.x = 0.4
            self.cmd_vel_pub.publish(twist_msg)

        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    node = FaceApproachNode(node_name="face_approach")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
