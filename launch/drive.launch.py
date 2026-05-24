from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_car',
            executable='serial_node',
            name='serial_node',
            output='screen'
        ),
        Node(
            package='autonomous_car',
            executable='servo_node',
            name='servo_node',
            output='screen'
        ),
        # camera_node removed — detection_node opens /dev/video0 directly.
        # Run camera_node only for teleop/streaming without detection.
    ])
