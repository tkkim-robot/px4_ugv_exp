from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

# from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera, NvbloxPeopleSegmentation
# from nvblox_ros_python_utils.nvblox_constants import SEMSEGNET_INPUT_IMAGE_WIDTH, \
#     SEMSEGNET_INPUT_IMAGE_HEIGHT

NVBLOX_CONTAINER_NAME = "nvblox_container"


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'rosbag', 'None', description='Path to rosbag (running on sensor if not set).', cli=True)
    args.add_arg('rosbag_args', '', description='Additional args for ros2 bag play.', cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    args.add_arg('run_rviz', False, cli=True)
    args.add_arg('run_realsense', True, cli=True)
    args.add_arg('run_vslam', True, cli=True)
    args.add_arg('run_nvblox', True, cli=True)

    actions = args.get_launch_actions()

    # Globally set use_sim_time if we're running from bag or sim
    actions.append(
        SetParameter('use_sim_time', True, condition=IfCondition(lu.is_valid(args.rosbag))))

    # Realsense
    actions.append(
        lu.include(
            'all_launch',
            'launch/sensors/realsense.launch.py',
            launch_arguments={'container_name': NVBLOX_CONTAINER_NAME},
            condition= UnlessCondition(lu.is_valid(args.rosbag)) and IfCondition(lu.is_valid(args.run_realsense)))
        )

    # Visual SLAM
    actions.append(
        lu.include(
            'all_launch',
            'launch/perception/vslam.launch.py',
            launch_arguments={
                'container_name': NVBLOX_CONTAINER_NAME,
            },
            # Delay for 1 second to make sure that the static topics from the rosbag are published.
            delay=1.0,
            condition=IfCondition(lu.is_valid(args.run_vslam))
            ))

    # Nvblox
    actions.append(
        lu.include(
            'all_launch',
            'launch/perception/nvblox.launch.py',
            launch_arguments={
                'container_name': NVBLOX_CONTAINER_NAME,
            },
            delay=2.0, # take it easy...
            condition=IfCondition(lu.is_valid(args.run_nvblox))
            ))

    # Play ros2bag
    actions.append(
        lu.play_rosbag(
            bag_path=args.rosbag,
            additional_bag_play_args=args.rosbag_args,
            condition=IfCondition(lu.is_valid(args.rosbag))))

    # Visualization
    actions.append(
        lu.include(
            'all_launch',
            'launch/visualization/visualization.launch.py',
            condition=IfCondition(lu.is_valid(args.run_rviz))
            ))

    # Container
    actions.append(lu.component_container(NVBLOX_CONTAINER_NAME, log_level=args.log_level))

    return LaunchDescription(actions)
