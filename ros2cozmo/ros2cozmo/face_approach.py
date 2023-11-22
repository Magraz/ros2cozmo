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

        ideal_box_size = 8925
        linear_x_speed = 0.4

        box_area = msg.width * msg.height

        if self.enabled and (box_area > 0):
            if box_area < ideal_box_size * 0.90:
                twist_msg.linear.x = linear_x_speed
            elif ideal_box_size * 1.20 < box_area:
                twist_msg.linear.x = -linear_x_speed

        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    node = FaceApproachNode(node_name="face_approach")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
