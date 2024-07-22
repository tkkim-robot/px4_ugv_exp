from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dasc_ros_utils',
            executable='publish_u.py',
            name='publish_u_node',
            output='screen'
        ),
        Node(
            package='dasc_ros_utils',
            executable='publish_tracking_node.py',
            name='publish_tracking_node',
            output='screen'
        ),
        Node(
            package='dasc_ros_utils',
            executable='obs_map_processing.py',
            name='obs_map_processing_node',
            output='screen'
        ),
    ])
