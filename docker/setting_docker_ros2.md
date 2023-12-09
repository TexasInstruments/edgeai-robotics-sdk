# Docker Setup for ROS 2

In the ROS 2 Docker images both for the target SK board and Ubuntu PC, two popular DDS implementations are installed by default:

- eProsima Fast DDS (default DDS for ROS 2 {{ROS2_DISTRO}})
- Eclipse Cyclone DDS (additionally installed)

To ensure smooth running of the out-of-box demos, the following DDS selections are made in the ROS 2 Docker containers by default on each platform.

| Platform          | DDS Choice           | Note                                            |
|-------------------|----------------------|-------------------------------------------------|
| Target SK board   | Eclipse Cyclone DDS  | Provides better performance, especially with "rosbag play" |
| Visualization PC  | eProsima Fast DDS    | Provides better `Ctrl+C` response                 |

You can switch the DDS implementation in each launch session by setting the environment variable `RMW_IMPLEMENTATION`. For example,

To use the eProsima Fast DDS,
```
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 ...
```

To use the Eclipse Cyclone DDS,
```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 ...
```

## Set Up Docker Environment on the Target

In the ROS 2 Docker container environment, ROS {{ROS2_DISTRO}} and necessary libraries and tools are installed.

1. To generate the scripts for building and running a Docker image for ROS 2 {{ROS2_DISTRO}}:
    ```
    root@am6x-sk:~/j7ros_home$ make scripts ROS_VER=2
    ```
    Make sure that two bash scripts, `docker_build_ros2.sh` and `docker_run_ros2.sh`, are generated.

2. To build the Docker image:
    ```
    root@am6x-sk:~/j7ros_home$ ./docker_build_ros2.sh
    ```
    This step may take some time depending on the network speed. Once "docker build" is completed, you can check the resulting Docker image with `docker images`.

3. To start/run the Docker container:
    ```
    root@am6x-sk:~/j7ros_home$ ./docker_run_ros2.sh
    ```
    It is important to use `docker_run_ros2.sh` script to start a Docker container since the script includes all the necessary settings to leverage all the cores and hardware accelerators of the TI Processor.

4. To build the ROS applications, inside the Docker container:
    ````{only} tag_j7x
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ colcon build --base-paths /opt/robotics_sdk/ros2 --cmake-force-configure

    # TDA4VM: In case "colcon build" fails (e.g., due to limited memory), "--executor sequential" option can be added as follows
    root@j7-docker:~/j7ros_home/ros_ws$ colcon build --base-paths /opt/robotics_sdk/ros2 --executor sequential --cmake-force-configure

    root@j7-docker:~/j7ros_home/ros_ws$ source install/setup.bash
    ```
    ````
    ````{only} tag_am62a
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ colcon build --base-paths /opt/robotics_sdk/ros2 --executor sequential --cmake-force-configure
    root@j7-docker:~/j7ros_home/ros_ws$ source install/setup.bash
    ```
    ````

## Set Up Docker Environment on the Remote PC for Visualization

```{note}
If your Ubuntu PC (for visualization) uses an Nvidia GPU driver, please ensure that the Nvidia Container Toolkit ("nvidia-docker" or "nvidia-docker2" depending on Ubuntu distro) is installed before running `docker_run_ros2.sh` script. Additionally, when generating scripts, make sure to add the argument `GPUS=y` (see Step 1 below).
```

You can choose any folder, but `init_setup.sh` script sets up `${HOME}/j7ros_home` as the working directory.

1. To generate bash scripts for building and running a Docker image for ROS 2 {{ROS2_DISTRO}}:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=2
    ```
    If the Ubuntu PC uses an Nvidia GPU driver, please add one more argument `GPUS=y`:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=2 GPUS=y
    ```
    Make sure that two bash scripts, `docker_build_ros2.sh` and `docker_run_ros2.sh`, are generated.

2. To build the ROS 2 Docker image:
    ```
    user@pc:~/j7ros_home$ ./docker_build_ros2.sh
    ```
    It may take some time building the Docker image. The Docker image built can be listed with `docker images`.

3. Run the ROS 2 Docker container:
    ```
    user@pc:~/j7ros_home$ ./docker_run_ros2.sh
    ```

4. Build the ROS nodes for visualization:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ colcon build --base-paths src/robotics_sdk/ros2
    root@pc-docker:~/j7ros_home/ros_ws$ source install/setup.bash
    ```

## Run Demo Applications

The table below summarizes the launch commands that you can use in the Docker container for each demo, on the target SK board, and on the remote visualization PC. For more details, see the following subsections.

Launch arguments can be passed to the following launch commands.

```{note}
Camera ID and subdev ID for cameras: You can check the camera ID (`cam_id`) and subdev ID (`subdev_id`) for the camera
attached to the SK board by running `/opt/edgeai-gst-apps/scripts/setup_cameras.sh`. You can use the alias `setup_cameras` inside the SDK container.

When a new camera is connected to the SK board while you’re already inside the SDK container, it is important to run the camera setup command.
```

```{only} tag_j7x
- To specify a camera recognized as `/dev/video-{usb,rpi,imx390}-camX` and `/dev/v4l-{rpi,imx390}-subdevY`, use the arguments `cam_id:=X` and `subdev_id:=Y`.
- To specify the serial number of the ZED camera (found in the original box), use the argument `zed_sn`, which should start with 'SN' followed by the serial number.
```
```{only} tag_am62a
- To specify a camera recognized as `/dev/video-{usb,rpi,imx390}-camX` and `/dev/v4l-{rpi,imx390}-subdevY`, use the arguments `cam_id:=X` and `subdev_id:=Y`.
```

```{note}
For your convenience, the SDK Docker container comes pre-configured with a variety of handy aliases. These aliases are specifically designed for building and launching demo applications, as well as for visualization tasks. Details about these aliases can be found in the `~/set_aliases.sh` file within the container. Alternatively, you can also refer to the `$SDK_DIR/docker/set_aliases.sh` file.
```

```{only} tag_j7x
| Demo (Input Source) | Launch command on Target        | Launch command on Remote Visualization PC  |
|---------------------|---------------------------------|--------------------------------------------|
| Stereo Vision (ZED camera) | ros2 launch ti_sde zed_sde_launch.py cam_id:=X zed_sn:=SNxxxxx | ros2 launch ti_viz_nodes rviz_sde_launch.py |
| Stereo Vision with point-cloud (ZED camera) | ros2 launch ti_sde zed_sde_pcl_launch.py cam_id:=X  zed_sn:=SNxxxxx | ros2 launch ti_viz_nodes rviz_sde_pcl_launch.py |
| Semantic Segmentation CNN (ZED camera)    | ros2 launch ti_vision_cnn zed_semseg_cnn_launch.py cam_id:=X  zed_sn:=SNxxxxx | ros2 launch ti_viz_nodes rviz_semseg_cnn_launch.py |
| Semantic Segmentation CNN (Mono camera)   | ros2 launch ti_vision_cnn gscam_semseg_cnn_launch.py cam_id:=X | same as above |
| Semantic Segmentation CNN (IMX219 camera) | ros2 launch ti_vision_cnn gscam_semseg_cnn_imx219_launch.py cam_id:=X subdev_id:=Y | same as above |
| Semantic Segmentation CNN (IMX390 camera) | ros2 launch ti_vision_cnn gscam_semseg_cnn_imx390_launch.py cam_id:=X subdev_id:=Y | same as above |
| Object Detection CNN (ZED camera)    | ros2 launch ti_vision_cnn zed_objdet_cnn_launch.py cam_id:=X zed_sn:=SNxxxxx | ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py |
| Object Detection CNN (Mono camera)   | ros2 launch ti_vision_cnn gscam_objdet_cnn_launch.py cam_id:=X | same as above |
| Object Detection CNN (IMX219 camera) | ros2 launch ti_vision_cnn gscam_objdet_cnn_imx219_launch.py cam_id:=X subdev_id:=Y | same as above |
| Object Detection CNN (IMX390 camera) | ros2 launch ti_vision_cnn gscam_objdet_cnn_imx390_launch.py cam_id:=X subdev_id:=Y | same as above |
| 3D Obstacle Detection (ZED camera) | ros2 launch ti_estop zed_estop_launch.py cam_id:=X zed_sn:=SNxxxxx | ros2 launch ti_viz_nodes rviz_estop_launch.py |
| Object Detection with 3D Spatial Information (ZED camera)  | ros2 launch ti_objdet_range zed_objdet_range_launch.py cam_id:=X zed_sn:=SNxxxxx | ros2 launch ti_viz_nodes rviz_objdet_range_launch.py |
```
```{only} tag_am62a
| Demo (Input Source) | Launch command on Target        | Launch command on Remote Visualization PC  |
|---------------------|---------------------------------|--------------------------------------------|
| Semantic Segmentation CNN (Mono camera) | ros2 launch ti_vision_cnn gscam_semseg_cnn_launch.py cam_id:=X framerate:=15 | ros2 launch ti_viz_nodes rviz_semseg_cnn_launch.py |
| Semantic Segmentation CNN (IMX219 camera) | ros2 launch ti_vision_cnn gscam_semseg_cnn_imx219_launch.py cam_id:=X subdev_id:=Y | same as above |
| Semantic Segmentation CNN (IMX390 camera) | ros2 launch ti_vision_cnn gscam_semseg_cnn_imx390_launch.py cam_id:=X subdev_id:=Y | same as above |
| Object Detection CNN (Mono camera) | ros2 launch ti_vision_cnn gscam_objdet_cnn_launch.py cam_id:=X framerate:=15 | ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py |
| Object Detection CNN (IMX219 camera) | ros2 launch ti_vision_cnn gscam_objdet_cnn_imx219_launch.py cam_id:=X subdev_id:=Y | same as above |
| Object Detection CNN (IMX390 camera) | ros2 launch ti_vision_cnn gscam_objdet_cnn_imx390_launch.py cam_id:=X subdev_id:=Y | same as above |
```

**Running Demos with ROSBAG**

It is recommended to launch the demos in two terminals on the target SK board, specifically, launch `ros2 bag play` in a separate terminal as shown in the following table.

```{only} tag_j7x
| Demo                           | ROSBAG launch command on Target (Terminal 1) | Demo launch command on Target (Terminal 2) | Launch command on Remote Visualization PC |
|--------------------------------|--------------------------------------------|------------------------------------------|-------------------------------------------|
| Stereo Vision                  | ros2 launch ti_sde rosbag_launch.py  | ros2 launch ti_sde sde_launch.py     | ros2 launch ti_viz_nodes rviz_sde_launch.py |
| Stereo Vision with point-cloud | ros2 launch ti_sde rosbag_launch.py  | ros2 launch ti_sde sde_pcl_launch.py | ros2 launch ti_viz_nodes rviz_sde_pcl_launch.py |
| Semantic Segmentation CNN | ros2 launch ti_sde rosbag_remap_launch.py | ros2 launch ti_vision_cnn semseg_cnn_launch.py | ros2 launch ti_viz_nodes rviz_semseg_cnn_launch.py |
| Object Detection CNN      | ros2 launch ti_sde rosbag_remap_launch.py | ros2 launch ti_vision_cnn objdet_cnn_launch.py | ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py |
| 6D Pose Estimation CNN    | -                                         | ros2 launch ti_vision_cnn bag_6dpose_cnn_launch.py | ros2 launch ti_viz_nodes rviz_6dpose_cnn_launch.py width:=1280 height:=960 |
| 3D Obstacle Detection     | ros2 launch ti_sde rosbag_launch.py  | ros2 launch ti_estop estop_launch.py                | ros2 launch ti_viz_nodes rviz_estop_launch.py |
| Object Detection with 3D Spatial Information  | ros2 launch ti_sde rosbag_launch.py  | ros2 launch ti_objdet_range objdet_range_launch.py  | ros2 launch ti_viz_nodes rviz_objdet_range_launch.py |
| Visual Localization       | -                                    | ros2 launch ti_vl bag_visloc_launch.py              | ros2 launch ti_viz_nodes rviz_visloc_launch.py |
```
```{only} tag_am62a
| Demo                      | ROSBAG launch command on Target (Terminal 1) | Demo launch command on Target (Terminal 2)  | Launch command on Remote Visualization PC |
|---------------------------|-------------------------------------------|------------------------------------------------|-------------------------------------------|
| Semantic Segmentation CNN | ros2 launch /opt/robotics_sdk/ros2/nodes/ti_sde/launch/rosbag_remap_launch.py | ros2 launch ti_vision_cnn semseg_cnn_launch.py | ros2 launch ti_viz_nodes rviz_semseg_cnn_launch.py |
| Object Detection CNN      | ros2 launch /opt/robotics_sdk/ros2/nodes/ti_sde/launch/rosbag_remap_launch.py | ros2 launch ti_vision_cnn objdet_cnn_launch.py | ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py |
```

```{tip}
You can use TMUX inside the ROS Docker container to split the current terminal window into multiple panes. Below are some useful basic keys for using TMUX. You can find a full list of keys [here](https://tmuxcheatsheet.com/).

- `tmux`: Start a tmux session.
- `Ctrl + b`, followed by `"`: Split pane vertically.
- `Ctrl + b`, followed by `↑` or `↓`: Switch to the pane in the respective direction.
- `Ctrl + b`, followed by `x`: Close the current pane.
```
