from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2cozmo",
                executable="face_detect",
                name="face_detect",
            ),
            Node(
                package="ros2cozmo",
                executable="face_tracker",
                name="face_tracker",
            ),
            Node(
                package="ros2cozmo",
                executable="face_approach",
                name="face_approach",
            ),
            Node(
                package="ros2cozmo",
                executable="motion_manager",
                name="motion_manager",
            ),
        ]
    )
