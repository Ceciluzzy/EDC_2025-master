from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    detector_node = Node(
            package='detector',
            executable='detector_node',
            name='detector',
            output='screen',
            parameters=[
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        )
    return LaunchDescription([
        detector_node
    ])