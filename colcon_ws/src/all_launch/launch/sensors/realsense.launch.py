from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu
import math

NVBLOX_CONTAINER_NAME = 'nvblox_container'


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    args.add_arg('run_splitter', 'False')

    # Config file
    config_file = lu.get_path('all_launch', 'config/sensors/realsense.yaml')

    # Splitter node
    realsense_splitter_node = ComposableNode(
        namespace='camera',
        name='realsense_splitter_node',
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SENSOR_DATA'
        }],
        remappings=[
            ('input/infra_1', '/camera/infra1/image_rect_raw'),
            ('input/infra_1_metadata', '/camera/infra1/metadata'),
            ('input/infra_2', '/camera/infra2/image_rect_raw'),
            ('input/infra_2_metadata', '/camera/infra2/metadata'),
            ('input/depth', '/camera/depth/image_rect_raw'),
            ('input/depth_metadata', '/camera/depth/metadata'),
            ('input/pointcloud', '/camera/depth/color/points'),
            ('input/pointcloud_metadata', '/camera/depth/metadata'),
        ])

    # Driver node
    realsense_node = ComposableNode(
        namespace='camera',
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[config_file],
        extra_arguments=[{'use_intra_process_comms': False}]
        )


    actions = args.get_launch_actions()
    actions.append(
        lu.component_container(
            args.container_name, condition=IfCondition(lu.is_true(args.run_standalone))))
    actions.append(
        lu.load_composable_nodes(
            args.container_name,
            [realsense_splitter_node],
            condition=IfCondition(lu.is_true(args.run_splitter))
        ))
    actions.append(
        lu.load_composable_nodes(
            args.container_name,
            [realsense_node],
        ))


    # static transform to base-link
    actions.append(
            lu.static_transform(
                "base_link",
                "camera_link",
                translation=[0.04, 0.0, 0.04],
                # orientation_rpy=[0.0, 0.0, math.pi]
                orientation_rpy=[0.0, 0.0, 0.0],
                )
            )

    return LaunchDescription(actions)
