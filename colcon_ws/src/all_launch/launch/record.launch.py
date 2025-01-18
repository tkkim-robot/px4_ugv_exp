# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('output', '', cli=True)

    actions = args.get_launch_actions()

    recording_started_msg =\
        '''\n\n\n
        -----------------------------------------------------
                    BAG RECORDING IS STARTING NOW

                 (make sure the realsense node is up)
        -----------------------------------------------------
        \n\n\n'''

    # Bag recording
    realsense_topics = [
        '/tf_static',
        '/camera/color/camera_info', '/camera/color/image_raw',
        '/camera/depth/camera_info', '/camera/depth/image_rect_raw',
        '/camera/infra1/camera_info',
        '/camera/infra1/image_rect_raw',
        '/camera/infra2/camera_info', '/camera/infra2/image_rect_raw',
        #'/camera/imu',
    ]

    vslam_topics = [
        '/tf',
        '/tf_static',
        '/visual_slam/status',
        '/visual_slam/tracking/odometry',
        '/visual_slam/tracking/vo_path',
        '/visual_slam/tracking/vo_pose',
        '/visual_slam/tracking/vo_pose_covariance',
        # '/visual_slam/vis/observations_cloud',
    ]

    px4_topics = [
        # '/px4_2/fmu/in/commander_set_state',
        # '/px4_2/fmu/in/parameter_req',
        '/px4_7/fmu/in/trajectory_setpoint',
        #'/px4_2/fmu/in/vehicle_visual_odometry',
        #'/px4_2/fmu/out/commander_status',
        # '/px4_2/fmu/out/parameter_res',
        #'/px4_2/fmu/out/simple_battery_status',
        #'/px4_2/fmu/out/vehicle_local_position',
        # '/px4_2/viz/trajectory_setpoint',
     ]

    topics = []
    topics += realsense_topics
    topics += vslam_topics
    topics += px4_topics

    record_action = lu.record_rosbag(topics=" ".join(topics), bag_path=args.output)
    actions.append(
        TimerAction(period=2.0, actions=[record_action,
                                          lu.log_info(recording_started_msg)]))

    return LaunchDescription(actions)
