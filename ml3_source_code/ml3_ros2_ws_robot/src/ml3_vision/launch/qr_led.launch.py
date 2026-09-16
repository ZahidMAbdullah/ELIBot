from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ml3_camera',
            executable='camera_node_publisher',
            name='camera_publisher_node',
            output='screen',
        ),
        Node(
            package='ml3_vision',
            executable='vision_status_node',
            name='vision_status_node',
            output='screen',
        ),
        Node(
            package='ml3_vision',
            executable='qr_led_indicator',
            name='qr_led_indicator',
            output='screen',
        ),
    ])
