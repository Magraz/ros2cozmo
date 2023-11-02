from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2cozmo',
            namespace='cozmo',
            executable='bringup',
            name='ros2cozmo_bringup'
        ),
        Node(
            package='ros2cozmo',
            namespace='cozmo',
            executable='face_detect',
            name='ros2cozmo_face_detect'
        ),
    ])