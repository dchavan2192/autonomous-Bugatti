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
        Node(
            package='autonomous_car',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
    ])
