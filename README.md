# px4_rover_exp

Environmental setup for rovers using PX4, ros2 humble, and VICON MoCap

### Hardware and Basic Setups

1. Boot the Nvidia Orin Board (if bootable, then go ahead)
2. Install Nvidia SDK Manager app in your favorite, host computer
3. Boot the orin as recovery mode, you need a thin connect

    [JETSON-ORIN-NX-16G-DEV-KIT - Waveshare Wiki](https://www.waveshare.com/wiki/JETSON-ORIN-NX-16G-DEV-KIT)

4. Through Nvidia SDK, set up the correct device, jetpack version (6) and everything. I suggest to check to install everything they have (including cuda)

    [Install Jetson Software with SDK Manager — SDK Manager 2.1.0 documentation](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html)

    - It seems also can install via command line, but need to update the BSP (the nvidia driver) in advance, so not recommend to do so
        [How to Install and Configure JetPack SDK — JetPack 6.0 documentation](https://docs.nvidia.com/jetson/archives/jetpack-archived/jetpack-60/install-setup/index.html#package-management-tool)

5. After flashing is complete, Nvidia SDK will try to install the selected SDK packages afterwards. Here, you need to connect the Orin and your host PC either via USB or ethernet. In my case, USB wasn’t work, so I connected them via ethernet, and manually set up the IP address to 192.168.56.x (just not 55 which is the default one), and proceed.
    1. Orin board needs to be connected through internet.
    2. Install the wifi/bluetooth module. In my case it was Intel 8xxxx module. But it was not detecting the wifi.

     <p align="center">
         <img width="300" alt="wifi" src="https://github.com/tkkim-robot/px4_rover_exp/assets/40379815/e5ff6214-55b1-4524-b86c-d9411fa52a2c">
     </p>

    3. So connect the internet through any sources, and install the driver
        ```bash
        sudo apt install iwlwifi-modules
        ```

### Development Envrionment Setup

1. Follow the basic installations

    - (This part in DASC Lab member can be fully ignored)
    - (assume SSD is already set up, and booted with that SSD, so need to skip the SSD part)
    - Therefore, the `/mnt/nova_ssd/` should be ignored, and instead just use the default home directory, such as `/home/ubuntu/`

    [Developer Environment Setup — isaac_ros_docs documentation](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)

2. Check JetPack version. The version should be R36 (June 27th, 2024)
    - `cat /etc/nv_tegra_release` if the output is not R36, then need to upgrade jetpack.
3. Clone `isaac_ros_common` under `${ISAAC_ROS_WS}/src`.

    ```bash
    cd ${ISAAC_ROS_WS}/src && \
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
    ```

4. (Deprecated)
    - Jetpack 6 will automatically install docker (nvidia-docker via nvidia-toolkit), so you can skip this part.
    - I know you already might have it, but there has been an major update for nvidia docker (now `nvidia-docker2` is deprecated, and need to install `nvidia-container-toolkit` that we have installed in step 3). And in this update, you don’t need to install CUDA, but need to install the new nvidia driver.
    1. Follow the step in 3.9 Ubuntu installation (network repo).

        - For instsance, the jetson Xavier that we are using, you should put
            - `ubuntu2004/arm64` as `$distro` and `$arch`

        [1. Introduction — Installation Guide for Linux 12.5 documentation](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/#ubuntu)
5. Then, go to this page, follow the instructions to install Realsense related packages (make realsense camera works in the Docker file)

    [Isaac ROS RealSense Setup — isaac_ros_docs documentation](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/sensors/realsense_setup.html)

    - add docker to the group (you will see the red message says to do so)
    - set docker to use without sudo
        - `sudo usermod -aG docker $USER`
    - the `run-dev.sh` takes a lot of time
    - Then, check whether the Isaac-ros docker can use realsense packages (see the instructions)
    - need to connect with a valid USB cable (no the power charging cable)

6. If above step succeed, go to this page to finish nvblox setting
    1. download assets

        [isaac_ros_nvblox — isaac_ros_docs documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#set-up-development-environment)

    2. `run-dev.sh` will now run the same docker container
    3. Then, install nvblox inside of your docker container **(install from source)**
    4. If you find from the colcon build that `--allow-overriding (something)` , which means you have installed it via Debian and now tries to install it from source, so add that flag.
    5. try a demo example
7. If above step succeed, go to this page to try a real camera demo:

    [RealSense Camera Examples — isaac_ros_docs documentation](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_realsense.html)

    - note that the installation (in case from srouce - install packages via rosdep) is done within the docker container, so if you run docker again, you need to install them again.
    - you need to unplug/plug the realsense camera whenever you re-run docker container

8. Let’s add our own docker setup into the isaac-ros docker container. The series of commands setting and installing docker environments are all organized in `run_dev.sh` , and it will call `build_image_layers.sh` to build dockerfile one by one.

    1. So, we will change a few things, to insert our dockerfile, and `run_dev.sh` will automatically build our dockerfile during it’s own instruction.
    2. First, configure [`.isaac_ros_common-config`](https://github.com/tkkim-robot/px4_rover_exp/blob/main/docker/.isaac_ros_common-config). During the realsene setup, we already created and configured this file. You can find this file in `isaac_ros_common/scripts` directory. It was previously `ros2_humble.realsense` . Now, change it to:

        ```bash
        CONFIG_IMAGE_KEY=ros2_humble.realsense.dasc_isaac
        ```

        It means `build_image_layers.sh` will run in an order of: 1. `Dockerfile.aarch64` 2. `Dockerfile.ros2_humble` 3. `Dockerfile.realsense` 4. `Dockerfile.dasc_isaac` 5. `Dockerfile.user`.

        Except for 4, they are already provided by Nvidia.

    3. Second, place the `Dockerfile.dasc_isaac` in the directory `isaac_ros_common/docker`. It includes building PX4 and VICON related libraies.

        [Dockerfile.dasc_isaac](https://github.com/tkkim-robot/px4_rover_exp/blob/main/docker/Dockerfile.dasc_isaac)

    4. Third, replace the `workspace-entrypoint.sh` in directory `isaac_ros_common/docker/scripts` . It includes install rosdep and source ros setup.bash files. It prevents the docker container from installing rosdep for NvBlox every time.

        [workspace-entrypoint.sh](https://github.com/tkkim-robot/px4_rover_exp/blob/main/docker/workspace-entrypoint.sh)

    5. Lastly, remove the `--rm` argument in `run_dev.sh` ’s last line (docker run arguments). Otherwise, the container will be removed whenever we exit the container. Then, add this line as an extra arguments in the last CLI in `run_Dev.sh`. Also change the workdir for convenience:

        ```bash
        docker run - it \
        (...)
        -v /home/ubuntu/workspaces/px4_ugv_exp/colcon_ws:/workspaces/colcon_ws \
        (...)
        --workdir /workspaces \
        ```

9. Before running `run_dev.sh` again, we need to clone our px4-related ros2 source packages from git.

    1. In `workspaces/` directory:

        ```bash
        git clone https://github.com/tkkim-robot/px4_ugv_exp
        cd px4_ugv_exp
        git submodule update --init --recursive
        ```

    2. Run `run_dev.sh`, and build the ros2 packages:

        ```bash
            colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
        ```

    3. Try running a demo NvBlox demo again

        ```bash
        ROS_DOMAIN_ID={YOUR_ID} ros2 launch nvblox_examples_bringup realsense_example.launch.py
        ```

    4. If the rviz doesn't pop up (or 'xcb' related issue), then:
        ```bash
        export DISPLAY=":1"
        ```
        It also might be `":0"`, but in our setting it was `":1"` (read from another article it's related to auto login).

10. In the docker, give r/w access to USB1 (which is px4 board)
    - `sudo chmod 777 /dev/ttyUSB1`


### How to Run Experiments

1. Trun on the VICON.
    - Turn two VICON computers and one switch on.
    - On system tab, reboot the red cameras 
    - On object tab, de-select "auto enable" and click "track".
    - Press 'alt' and drag markers of interset, and type the name (default: px4_1) and done. 
    - Right click 'px4_1' on object tab, click "save object" and make it "shared". 
2. Turn on the rover with Orin and charged battery, put on markers on the plate (to be asymmetric).
3. Place the indicator to the battery (left sided), and change the beep threshold to 3.70. (11.1v is the lowest voltage to stop, it's charged up to 12.x v.)
4. SSH into the Orin with VSCode.
5. Setup ground station with random laptop.
    - ``` cd hardik/rover_groundstation_ros2_jumpstart```
    - ``` xhost + ```
    - ``` docker compose up -d ```
    - ``` docker exec -it rover_groundstation_ros2_jumpstart-gs-1 bash```
    - ``` (in the docker) ROS_DOMAIN_ID=4 ros2 launch ground_station_launch  gs.launch.py ```

6. Setup Orin with SSH in VSCODE
    - ``` docker start isaac-(...) ```
    - ``` docker exec -it isaac-(...) bash```
    - ``` (in the docker) cd colcon_ws && source install/setup.bash```
    - ``` sudo chmod 777 /dev/ttyUSB1 ```
    - ``` ROS_DOMAIN_ID=4 ros2 launch all_launch px4.launch.py ```
    - Then, the RVIZ in groundstation turns green to "VALID"

7. Now, run scripts
    - The extra repos and folders are placed in 'px4_ugv_exp/colcon_ws/src/dasc_ros/dasc_ros_utils/scripts/', and it is mounted to the docker.
    - Now, running your own scripts that publish px4 topic, it will run the rover.
    - ``` ROS_DOMAIN_ID=4 ros2 run dasc_lab_utils publish_u.py```
        - it will convert the ros2 ctrl_vel to left/right wheel velocity into px4.
    - ```ROS_DOMAIN_ID=4 ros2 run dasc_lab_utils publish_tracking_node.py```
        - customize your file and run like this.

8. To set (0,0) velocity to both wheels:
    - (very beginning) In RVIZ, click "raw mode", make sure both motors are 0.0, click "publish", then click "arm". 
    - When you "disarm", click "publish", "arm", deselect "publish", then run my program again. 