#!/bin/bash
#
# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

cd /workspaces/isaac_ros-dev
apt update
rosdep update
rosdep install -i -r --from-paths /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/ --rosdistro humble -y

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to isaac_ros_nvblox
echo "source /workspaces/isaac_ros-dev/install/setup.bash" >> ~/.bashrc
source /workspaces/isaac_ros-dev/install/setup.bash

echo "source /workspaces/px4_ugv_exp/colcon_ws/install/setup.bash" >> ~/.bashrc
source /workspaces/px4_ugv_exp/colcon_ws/install/setup.bash

cd /workspaces
# Restart udev daemon
sudo service udev restart

$@
