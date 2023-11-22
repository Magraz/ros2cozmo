import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty


class MotionManagerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, namespace=node_name)

        self.enable_srv = self.create_service(Empty, "enable", self.enable_callback)
        self.disable_srv = self.create_service(Empty, "disable", self.disable_callback)

        self.face_track_en_cli = self.create_client(Empty, "/face_tracker/enable")
        self.face_track_dis_cli = self.create_client(Empty, "/face_tracker/disable")
        self.face_appr_en_cli = self.create_client(Empty, "/face_approach/enable")
        self.face_appr_dis_cli = self.create_client(Empty, "/face_approach/disable")

        self.face_tracker_sub = self.create_subscription(
            Twist, "/face_tracker/cmd_vel", self.face_tracker_motion, 10
        )

        self.face_approach_sub = self.create_subscription(
            Twist, "/face_approach/cmd_vel", self.face_approach_motion, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, "/cozmo/cmd_vel", 10)

        self.combined_twist_msg = Twist()

        self.cliff = False
        self.cliff_sub = self.create_subscription(
            Bool, "/cozmo/cliff", self.cliff_detection, 10
        )

        # Set initial head tilt
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.set_init_head_tilt)
        self.initializing = False
        self.i = 0

    def set_init_head_tilt(self):
        if self.i <= 4:
            initial_head_twist = Twist()
            initial_head_twist.linear.y = 0.4
            self.cmd_vel_pub.publish(initial_head_twist)
            self.initializing = True
            self.i += 1
        else:
            self.initializing = False
            self.timer.cancel()

    def enable_callback(self, request, response):
        _ = self.face_track_en_cli.call_async(request)
        _ = self.face_appr_en_cli.call_async(request)
        return response

    def disable_callback(self, request, response):
        _ = self.face_track_dis_cli.call_async(request)
        _ = self.face_appr_dis_cli.call_async(request)
        return response

    def cliff_detection(self, msg):
        self.cliff = msg.data

    def face_tracker_motion(self, msg):
        if not self.initializing:
            self.combined_twist_msg.angular.z = msg.angular.z
            self.combined_twist_msg.linear.y = msg.linear.y

        self.cmd_vel_pub.publish(self.combined_twist_msg)

    def face_approach_motion(self, msg):
        if not self.initializing:
            if not self.cliff:
                self.combined_twist_msg.linear.x = msg.linear.x
            elif msg.linear.x < 0:
                self.combined_twist_msg.linear.x = msg.linear.x
        self.cmd_vel_pub.publish(self.combined_twist_msg)


def main(args=None):
    rclpy.init(args=args)

    node = MotionManagerNode(node_name="motion_manager")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
