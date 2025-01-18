from typing import List, Tuple
from random import randint

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

NVBLOX_CONTAINER_NAME = "nvblox_container"

def get_realsense_remappings() -> List[Tuple[str, str]]:
    remappings = []
    remappings.append(('depth/image', '/camera/depth/image_rect_raw'))
    remappings.append(('depth/camera_info', '/camera/depth/camera_info'))
    remappings.append(('color/image', '/camera/color/image_raw'))
    remappings.append(('color/camera_info', '/camera/color/camera_info'))
    return remappings


def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:

    config = lu.get_path('all_launch', 'config/nvblox/nvblox_base.yaml')
    remappings = get_realsense_remappings()

    remappings.append( ("/pose", "/visual_slam/tracking/vo_pose") )

    parameters = []
    parameters.append(config)
    parameters.append({'num_cameras': 1})
    parameters.append({'use_lidar': False})

    nvblox_node_name = f'nvblox_node'
    nvblox_plugin_name = 'nvblox::NvbloxNode'

    nvblox_node = ComposableNode(
        name=nvblox_node_name,
        package='nvblox_ros',
        plugin=nvblox_plugin_name,
        remappings=remappings,
        parameters=parameters,
    )

    actions = []
    if args.run_standalone:
        actions.append(lu.component_container(args.container_name))

    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))
    actions.append(
        lu.log_info(["Starting nvblox"]))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')

    args.add_opaque_function(add_nvblox)
    return LaunchDescription(args.get_launch_actions())
