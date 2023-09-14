
# Docker Setup for ROS 1

## Set Up Docker Environment on the Target

In the ROS 1 Docker container environment, ROS {{ROS1_DISTRO}}, necessary libraries and tools are installed.

1. To generate the scripts for building and running a Docker image for ROS 1 {{ROS1_DISTRO}}:
    ```
    root@am6x-sk:~/j7ros_home$ make scripts ROS_VER=1
    ```
    Make sure that two bash scripts, `docker_build_ros1.sh` and `docker_run_ros1.sh`, are generated.

2. To build the Docker image:
    ```
    root@am6x-sk:~/j7ros_home$ ./docker_build_ros1.sh
    ```
    This step may take some time depending on the network speed. Once the "docker build" is completed, you can check the resulting Docker image with `docker images`.

3. To start/run the Docker container:
    ```
    root@am6x-sk:~/j7ros_home$ ./docker_run_ros1.sh
    ```
    It is important to use `docker_run_ros1.sh` script to start a Docker container since the script includes all the necessary settings to leverage all the cores and hardware accelerators of the TI Processor.

4. To build the ROS applications, inside the Docker container:
    ````{only} tag_j7x
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ catkin_make --source /opt/robotics_sdk/ros1 --force-cmake
    # TDA4VM: In case catkin_make fails due to limited memory, "-j1" option can be added as follows
    root@j7-docker:~/j7ros_home/ros_ws$ catkin_make -j1 --source /opt/robotics_sdk/ros1 --force-cmake
    root@j7-docker:~/j7ros_home/ros_ws$ source devel/setup.bash
    ```
    ````
    ````{only} tag_am62a
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ catkin_make -j1 --source /opt/robotics_sdk/ros1 --force-cmake
    root@j7-docker:~/j7ros_home/ros_ws$ source devel/setup.bash
    ```
    ````

## Set Up Docker Environment on the Remote PC for Visualization

```{note}
If your Ubuntu PC (for visualization) uses an Nvidia GPU driver, please ensure that the Nvidia Container Toolkit ("nvidia-docker" or "nvidia-docker2" depending on Ubuntu distro) is installed before running `docker_run_ros1.sh` script. Additionally, when generating scripts, make sure to add the argument `GPUS=y` (see Step 1 below).
```

You can choose any folder, but `init_setup.sh` script sets up `${HOME}/j7ros_home` as the working directory.

1. To generate the scripts for building and running a Docker image for ROS 1 {{ROS1_DISTRO}}:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=1
    ```
    If the Ubuntu PC uses an Nvidia GPU driver, please add one more argument `GPUS=y` as follows:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=1 GPUS=y
    ```
    Make sure that two bash scripts, `docker_build_ros1.sh` and `docker_run_ros1.sh`, are generated.

2. To build the ROS 1 Docker image:
    ```
    user@pc:~/j7ros_home$ ./docker_build_ros1.sh
    ```
    It may take some time building the Docker image. Once the "docker run" completes, the resulting Docker image can be listed with `docker images`.

3. ROS network setting: We need to set up two environment variables that will be passed to the Docker container and used to configure ROS network settings. Please update the following two lines in `setup_env_pc.sh`:
    ```
    export J7_IP_ADDR=<target_IP_address>
    export PC_IP_ADDR=<PC_IP_address>
    ```
    `<target_IP_address>` can be found by running `make ip_show` on a target terminal.

    Then, source the updated script before executing `docker_run_ros1.sh` (in the next step):
    ```
    user@pc:~/j7ros_home$ source ./setup_env_pc.sh
    ```

4. Start/Run the ROS 1 Docker container:
    ```
    user@pc:~/j7ros_home$ ./docker_run_ros1.sh
    ```

5. Build the ROS nodes for visualization:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ catkin_make --source src/robotics_sdk/ros1
    root@pc-docker:~/j7ros_home/ros_ws$ source devel/setup.bash
    ```

## Run Demo Applications

The table below summarizes the launch commands that you can use in the Docker container for each demo, on the target SK board, and on the remote visualization PC. For more details, see the following subsections.

Launch arguments can be passed to the following launch commands.
- To specify a camera recognized as `/dev/videoX`, use the argument `video_id:=X`.
- To specify the serial number of the ZED camera (found in the original box), use the argument `zed_sn`, which should start with 'SN' followed by the serial number.

```{only} tag_j7x
| Demo (Input Source) | Launch command on Target        | Launch command on Remote Visualization PC  |
|---------------------|---------------------------------|--------------------------------------------|
| Stereo Vision (ROSBAG)     | roslaunch ti_sde bag_sde.launch  | roslaunch ti_viz_nodes rviz_sde.launch |
| Stereo Vision (ZED camera) | roslaunch ti_sde zed_sde.launch video_id:=x zed_sn:=SNxxxxx | same as above |
| Stereo Vision with point-cloud (ROSBAG)     | roslaunch ti_sde bag_sde_pcl.launch video_id:=x zed_sn:=SNxxxxx | roslaunch ti_viz_nodes rviz_sde_pcl.launch |
| Stereo Vision with point-cloud (ZED camera) | roslaunch ti_sde zed_sde_pcl.launch video_id:=x zed_sn:=SNxxxxx | same as above |
| Semantic Segmentation CNN (ROSBAG)        | roslaunch ti_vision_cnn bag_semseg_cnn.launch   | roslaunch ti_viz_nodes rviz_semseg_cnn.launch |
| Semantic Segmentation CNN (ZED camera)    | roslaunch ti_vision_cnn zed_semseg_cnn.launch video_id:=x zed_sn:=SNxxxxx | same as above |
| Semantic Segmentation CNN (Mono camera)   | roslaunch ti_vision_cnn gscam_semseg_cnn.launch video_id:=x | same as above |
| Semantic Segmentation CNN (IMX219 camera) | roslaunch ti_vision_cnn gscam_semseg_cnn_imx219.launch video_id:=x subdev_id:=y | same as above |
| Semantic Segmentation CNN (IMX390 camera) | roslaunch ti_vision_cnn gscam_semseg_cnn_imx390.launch video_id:=x subdev_id:=y | same as above |
| Object Detection CNN (ROSBAG)        | roslaunch ti_vision_cnn bag_objdet_cnn.launch   | roslaunch ti_viz_nodes rviz_objdet_cnn.launch |
| Object Detection CNN (ZED camera)    | roslaunch ti_vision_cnn zed_objdet_cnn.launch video_id:=x zed_sn:=SNxxxxx | same as above |
| Object Detection CNN (Mono camera)   | roslaunch ti_vision_cnn gscam_objdet_cnn.launch video_id:=x | same as above |
| Object Detection CNN (IMX219 camera) | roslaunch ti_vision_cnn gscam_objdet_cnn_imx219.launch video_id:=x subdev_id:=y | same as above |
| Object Detection CNN (IMX390 camera) | roslaunch ti_vision_cnn gscam_objdet_cnn_imx390.launch video_id:=x subdev_id:=y | same as above |
| 3D Obstacle Detection (ROSBAG)     | roslaunch ti_estop bag_estop.launch  | roslaunch ti_viz_nodes rviz_estop.launch |
| 3D Obstacle Detection (ZED camera) | roslaunch ti_estop zed_estop.launch video_id:=x zed_sn:=SNxxxxx | same as above |
| Object Detection with 3D Spatial Information (ROSBAG)     | roslaunch ti_objdet_range bag_objdet_range.launch  | roslaunch ti_viz_nodes rviz_objdet_range.launch |
| Object Detection with 3D Spatial Information (ZED camera) | roslaunch ti_objdet_range zed_objdet_range.launch video_id:=x zed_sn:=SNxxxxx | same as above |
| Visual Localization (ROSBAG)       | roslaunch ti_vl bag_visloc.launch    | roslaunch ti_viz_nodes rviz_visloc.launch |
```
```{only} tag_am62a
| Demo (Input Source) | Launch command on Target        | Launch command on Remote Visualization PC  |
|---------------------|---------------------------------|--------------------------------------------|
| Semantic Segmentation CNN (ROSBAG)      | roslaunch ti_vision_cnn bag_semseg_cnn.launch   | roslaunch ti_viz_nodes rviz_semseg_cnn.launch |
| Semantic Segmentation CNN (Mono camera) | roslaunch ti_vision_cnn gscam_semseg_cnn.launch video_id:=x | same as above |
| Semantic Segmentation CNN (IMX219 camera) | roslaunch ti_vision_cnn gscam_semseg_cnn_imx219.launch video_id:=x subdev_id:=y | same as above |
| Semantic Segmentation CNN (IMX390 camera) | roslaunch ti_vision_cnn gscam_semseg_cnn_imx390.launch video_id:=x subdev_id:=y | same as above |
| Object Detection CNN (ROSBAG)        | roslaunch ti_vision_cnn bag_objdet_cnn.launch   | roslaunch ti_viz_nodes rviz_objdet_cnn.launch |
| Object Detection CNN (Mono camera)   | roslaunch ti_vision_cnn gscam_objdet_cnn.launch video_id:=x | same as above |
| Object Detection CNN (IMX219 camera) | roslaunch ti_vision_cnn gscam_objdet_cnn_imx219.launch video_id:=x subdev_id:=y | same as above |
| Object Detection CNN (IMX390 camera) | roslaunch ti_vision_cnn gscam_objdet_cnn_imx390.launch video_id:=x subdev_id:=y | same as above |
```

In the following, **[SK]** and **[PC]** indicate the steps that should be launched, either on the target SK board or on the PC.

````{only} tag_j7x
**Run Stereo Vision Application**

1. **[SK]** To launch `ti_sde` node with playing back a ROSBAG file, run the following `roslaunch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_sde bag_sde.launch
    ```
    Alternatively, you can run the following directly on the target host Linux:
    ```
    root@am6x-sk:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_sde bag_sde.launch
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_sde.launch` in the above.

2. **[PC]** For visualization, in the ROS 1 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_sde.launch
    ```
````

````{only} tag_j7x
**Run Stereo Vision Application with Point-Cloud Enabled**

1. **[SK]** To launch `ti_sde` node with point-cloud enabled on a ROSBAG file, run the following `roslaunch` command **inside** the Docker:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_sde bag_sde_pcl.launch
    ```
    Alternatively, you can run the following directly on the target host Linux:
    ```
    root@am6x-sk:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_sde bag_sde_pcl.launch
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_sde_pcl.launch` in the above.

2. **[PC]** For visualization, in the ROS 1 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_sde_pcl.launch
    ```
````

**Run Semantic Segmentation CNN Application**

1. **[SK]** To launch semantic segmentation demo with playing back a ROSBAG file, run the following `roslaunch` command **inside** the Docker:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_vision_cnn bag_semseg_cnn.launch
    ```
    Alternatively, you can run the following directly on the target host Linux:
    ```
    root@am6x-sk:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_vision_cnn bag_semseg_cnn.launch
    ```
    ```{only} tag_j7x
    To process the image stream from a ZED stereo camera or a USB mono camera, replace the launch file with `zed_semseg_cnn.launch` or `gscam_semseg_cnn.launch` in the above.
    ```
    ```{only} tag_am62a
    To process the image stream from a USB mono camera, replace the launch file with `gscam_semseg_cnn.launch` in the above.
    ```

2. **[PC]** For visualization, in the ROS 1 container on the PC::
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_semseg_cnn.launch
    ```

**Run Object Detection CNN Application**

1. **[SK]** To launch object detection demo with playing back a ROSBAG file, run the following `roslaunch` command **inside** the Docker:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_vision_cnn bag_objdet_cnn.launch
    ```
    Alternatively, you can run the following directly on the target host Linux:
    ```
    root@am6x-sk:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_vision_cnn bag_objdet_cnn.launch
    ```
    ```{only} tag_j7x
    To process the image stream from a ZED stereo camera or a USB mono camera, replace the launch file with `zed_semseg_cnn.launch` or `gscam_semseg_cnn.launch` in the above.
    ```
    ```{only} tag_am62a
    To process the image stream from a USB mono camera, replace the launch file with `gscam_semseg_cnn.launch` in the above.
    ```

2. **[PC]** For visualization, in the ROS 1 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_objdet_cnn.launch
    ```

````{only} tag_j7x
**Run 3D Obstacle Detection Application**

1. **[SK]** To launch `ti_estop` node with playing back a ROSBAG file, run the following `roslaunch` command **inside** the Docker:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_estop bag_estop.launch
    ```
    Alternatively, you can run the following directly on the target host Linux:
    ```
    root@am6x-sk:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_estop bag_estop.launch
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_estop.launch` in the above.

2. **[PC]** For visualization, in the ROS 1 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_estop.launch
    ```
````

````{only} tag_j7x
**Run Object Detection with 3D Spatial Information**

1. **[SK]** To launch `ti_objdet_range` node with playing back a ROSBAG file, run the following `roslaunch` command **inside** the Docker:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_objdet_range bag_objdet_range.launch
    ```
    Alternatively, you can run the following directly on the target host Linux:
    ```
    root@am6x-sk:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_objdet_range bag_objdet_range.launch
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_objdet_range.launch` in the above.

2. **[PC]** For visualization, in the ROS 1 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_objdet_range.launch
    ```
````

````{only} tag_j7x
**Run Visual Localization Application**

1. **[SK]** To launch `ti_vl` node with playing back a ROSBAG file, run the following `roslaunch` command **inside** the Docker:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_vl bag_visloc.launch
    ```
    Alternatively, you can run the following directly on the target host Linux:
    ```
    root@am6x-sk:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_vl bag_visloc.launch
    ```
2. **[PC]** For visualization, in the ROS 1 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_visloc.launch
    ```
````

**Build and Run Hector SLAM Application**

Many open-source SLAM algorithms can run on the target SK board. Among them, it is demonstrated how to setup and run Hector SLAM on 2D Lidar data. For details, please refer to [Hector SLAM Application](../ros1/slam/README.md).
