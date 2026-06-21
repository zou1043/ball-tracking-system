from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. Visual detection node
        Node(
            package='ball_detection',
            executable='detect_node',
            name='detect_node',
            output='screen'
        ),
        # 2. Tracking control node
        Node(
            package='ball_detection',
            executable='crtl_node',
            name='crtl_node',
            output='screen'
        ),
        # 3. ESP32 serial bridge node
        Node(
            package='ball_detection',
            executable='esp32_serial_node',
            name='esp32_node',
            output='screen'
        ),
    ])
