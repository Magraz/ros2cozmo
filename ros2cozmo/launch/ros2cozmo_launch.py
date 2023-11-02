from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2cozmo',
            namespace='ros2cozmo_1',
            executable='bringup',
            name='ros2cozmo_bringup'
        ),
        Node(
            package='ros2cozmo',
            namespace='ros2cozmo_1',
            executable='face_detect',
            name='ros2cozmo_face_detect'
        ),
    ])