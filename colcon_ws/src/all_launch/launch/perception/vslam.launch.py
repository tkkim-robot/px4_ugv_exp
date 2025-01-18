
from typing import List

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

NVBLOX_CONTAINER_NAME = "nvblox_container"

def add_vslam(args: lu.ArgumentContainer) -> List[Action]:

    realsense_remappings = [
        ('visual_slam/camera_info_0', '/camera/infra1/camera_info'),
        ('visual_slam/camera_info_1', '/camera/infra2/camera_info'),
        ('visual_slam/image_0', '/camera/infra1/image_rect_raw'),
        ('visual_slam/image_1', '/camera/infra2/image_rect_raw')
        # ('visual_slam/image_0', '/camera/realsense_splitter_node/output/infra_1'),
        # ('visual_slam/image_1', '/camera/realsense_splitter_node/output/infra_2')
    ]

    base_parameters = {
        'num_cameras': 2,
        'min_num_images': 2,
        'enable_localization_n_mapping': False,
        'enable_imu_fusion': False,
        'gyro_noise_density': 0.000244,
        'gyro_random_walk': 0.000019393,
        'accel_noise_density': 0.001862,
        'accel_random_walk': 0.003,
        'calibration_frequency': 200.0,
        'rig_frame': 'base_link',
        'imu_frame': 'front_stereo_camera_imu',
        'enable_slam_visualization': True,
        'enable_landmarks_view': True,
        'enable_observations_view': True,
        'path_max_size': 200,
        'verbosity': 5,
        'enable_debug_mode': False,
        'debug_dump_path': '/tmp/cuvslam',
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': 'base_link',
    }
    realsense_parameters = {
        'enable_rectified_pose': True,
        'enable_image_denoising': False,
        'rectified_images': True,
        'camera_optical_frames': [
            'camera_infra1_optical_frame',
            'camera_infra2_optical_frame',
        ],
    }

    remappings = realsense_remappings
    camera_parameters = realsense_parameters

    parameters = []
    parameters.append(base_parameters)
    parameters.append(camera_parameters)
    parameters.append(
        {'enable_ground_constraint_in_odometry': args.enable_ground_constraint_in_odometry})

    actions = []
    vslam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=remappings,
        parameters=parameters)
    actions.append(lu.load_composable_nodes(args.container_name, [vslam_node]))

    if args.run_standalone:
        actions.append(lu.component_container(args.container_name))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'enable_ground_constraint_in_odometry',
        'False',
        description='Whether to constraint robot movement to a 2d plane (e.g. for AMRs).',
        cli=True)
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    args.add_opaque_function(add_vslam)
    return LaunchDescription(args.get_launch_actions())
