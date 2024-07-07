from launch import LaunchDescription
from launch_ros.actions import Node


def create_bridge_node(robot_name):

    return Node(
            package='vicon_px4_bridge', executable='bridge', output='screen',
            parameters=[{'px4_name': robot_name, 'vicon_name': robot_name}]
        )

def generate_launch_description():

    robots = ['px4_1']

    bridge_nodes = [create_bridge_node(r) for r in robots]

    return LaunchDescription(bridge_nodes)

