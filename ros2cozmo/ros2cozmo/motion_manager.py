import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import String
from ros2cozmo_interfaces.msg import DetectionBox
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


class MotionManagerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, namespace=node_name)
        self.face_tracker_sub = self.create_subscription(
            DetectionBox, "/face_tracker/cmd_vel", self.face_tracker_motion, 10
        )
        self.face_approach_sub = self.create_subscription(
            DetectionBox, "/face_approach/cmd_vel", self.face_approach_motion, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cozmo/cmd_vel", 10)

        self.combined_twist_msg = Twist()
        self.subscription  # prevent unused variable warning

    def face_tracker_motion(self, msg):
        self.combined_twist_msg.angular.x = msg.angular.x
        self.cmd_vel_pub.publish(self.combined_twist_msg)

    def face_approach_motion(self, msg):
        self.combined_twist_msg.linear.x = msg.angular.x
        self.cmd_vel_pub.publish(self.combined_twist_msg)


def main(args=None):
    rclpy.init(args=args)

    node = MotionManagerNode(node_name="motion_manager")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
