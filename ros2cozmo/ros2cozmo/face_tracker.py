import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import String
from ros2cozmo_interfaces.msg import DetectionBox
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


class FaceTrackerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, namespace=node_name)
        self.subscription = self.create_subscription(
            DetectionBox, "/face_detect/main_face", self.track_face, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.enable_srv = self.create_service(Empty, "enable", self.enable_callback)
        self.disable_srv = self.create_service(Empty, "disable", self.disable_callback)

        self.camera_width = 320  # Cozmo camera width
        self.camera_height = 240  # Cozmo camera height
        self.centering_threshold_x = 0.3  # Acceptable error in centering body
        self.centering_threshold_y = 0.2  # Acceptable error in centering head
        self.constant_ang_z_speed = 0.2  # Constant speed for rotation in x
        self.constant_lin_y_speed = 0.5  # Constant speed for rotation in y
        self.enabled = False

    def enable_callback(self, request, response):
        self.enabled = True
        print("Enable called")
        return response

    def disable_callback(self, request, response):
        self.enabled = False
        print("Disabled called")
        return response

    def track_face(self, msg):
        twist_msg = Twist()
        box_area = msg.width * msg.height
        if self.enabled and (box_area > 0):
            # Calculate center of the face
            center_x = msg.top_left_corner.x + msg.width / 2
            center_y = msg.top_left_corner.y + msg.height / 2

            # Calculate error from the center of the camera's view
            error_x = (center_x - self.camera_width / 2) / (self.camera_width / 2)
            error_y = (center_y - self.camera_height / 2) / (self.camera_height / 2)

            # Check if the X error is within the acceptable threshold
            if abs(error_x) < self.centering_threshold_x:
                # Face is centered
                twist_msg.angular.z = 0.0
            else:
                # Determine the direction of rotation
                angular_speed = 0
                if error_x > 0:
                    angular_speed = -self.constant_ang_z_speed
                else:
                    angular_speed = self.constant_ang_z_speed

                # Set the speed
                twist_msg.angular.z = angular_speed

            # Check if the Y error is within the acceptable threshold
            if abs(error_y) < self.centering_threshold_y:
                # Face is centered
                twist_msg.linear.y = 0.0
            else:
                # Determine the direction of rotation
                y_speed = 0
                if error_y > 0:
                    y_speed = -self.constant_lin_y_speed
                else:
                    y_speed = self.constant_lin_y_speed

                # Set the speed for Y
                twist_msg.linear.y = y_speed

        # Publish the movement command
        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    node = FaceTrackerNode(node_name="face_tracker")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
