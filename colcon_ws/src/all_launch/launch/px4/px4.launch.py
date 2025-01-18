import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    run_vicon_arg = DeclareLaunchArgument(
            'run_vicon', default_value="True")
    run_uxrce_arg = DeclareLaunchArgument(
            'run_uxrce', default_value="True")
    run_mavlink_arg = DeclareLaunchArgument(
            'run_mavlink', default_value="True")
    robot_name_arg = DeclareLaunchArgument(
            'robot_name', default_value="px4_7")

    microXRCE_bridge = ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyUSB1', '-b', '921600'],
            name='microXRCEAgent',
            condition=IfCondition(LaunchConfiguration('run_uxrce')),
            output='both')

    mavlink_router_conf_file = os.path.join(
            get_package_share_directory('all_launch'), 'config', 'px4', 'mavlink-router.conf')

    mavlink = ExecuteProcess(
            cmd = ["mavlink-routerd", "-c", mavlink_router_conf_file],
            name="mavlink-routerd",
            condition=IfCondition(LaunchConfiguration('run_mavlink')),
            output='log')

    robot_name = LaunchConfiguration('robot_name')
    vicon_px4_bridge_node = Node(
        package='vicon_px4_bridge',
        executable='bridge',
        output='screen',
        parameters=[{'px4_name': robot_name, 'vicon_name': robot_name}],
        condition=IfCondition(LaunchConfiguration('run_vicon')),
    )

    vicon_launch  = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("all_launch"), "/launch", "/sensors", "/vicon.launch.py"]),
            condition=IfCondition(LaunchConfiguration('run_vicon'))
            )

    return LaunchDescription([
        run_vicon_arg,
        run_uxrce_arg,
        run_mavlink_arg,
        robot_name_arg,
        microXRCE_bridge,
        mavlink,
        vicon_px4_bridge_node,
        vicon_launch
        ])
