
# Setting Up Robotics SDK

This section describes how to set up the Robotics SDK on the Processor SDK Linux for Edge AI.

## Requirements & Dependencies

### Supported Hardware Platforms

```{only} tag_j7x
| Platform                                    | EVM                                            |
|---------------------------------------------|------------------------------------------------|
| [TDA4VM](https://www.ti.com/product/TDA4VM) | [SK-TDA4VM](https://www.ti.com/tool/SK-TDA4VM) |
| [AM68A](https://www.ti.com/product/AM68A)   | [SK-AM68](https://www.ti.com/tool/SK-AM68)     |
| [AM69A](https://www.ti.com/product/AM69A)   | [SK-AM69](https://www.ti.com/tool/SK-AM69)     |
```
```{only} tag_am62a
| Platform                                               | EVM                                                |
|--------------------------------------------------------|----------------------------------------------------|
| [AM62Ax](https://www.ti.com/lit/ds/symlink/am62a3.pdf) | [SK-AM62A-LP](https://www.ti.com/tool/SK-AM62A-LP) |
```

### Processor SDK Linux for Edge AI

The Robotics SDK requires the **SD card image** from Processor SDK Linux for Edge AI:

```{only} tag_j7x
| Platform | Link to Processor SDK Linux                                                                                                |
| -------- | ---------------------------------------------------------------------------------------------------------------------------|
| TDA4VM   | [Processor SDK Linux for TDA4VM](https://www.ti.com/tool/download/PROCESSOR-SDK-LINUX-SK-TDA4VM) (Version **{{SDK_VER}}.XX**) |
| AM68A    | [Processor SDK Linux for AM68A](https://www.ti.com/tool/download/PROCESSOR-SDK-LINUX-AM68A) (Version **{{SDK_VER}}.XX**)      |
| AM69A    | [Processor SDK Linux for AM69A](https://www.ti.com/tool/download/PROCESSOR-SDK-LINUX-AM69A) (Version **{{SDK_VER}}.XX**)      |
```
```{only} tag_am62a
| Platform | Link to Processor SDK Linux                                                                                                |
| -------- | ---------------------------------------------------------------------------------------------------------------------------|
| AM62A    | [Processor SDK Linux for AM62A](https://www.ti.com/tool/download/PROCESSOR-SDK-LINUX-AM62A) (Version **{{SDK_VER}}.XX**)      |
```

The SD card image contains Processor SDK Linux and libraries that are necessary for setting up the Robotics SDK environment.

### Ubuntu PC

A Ubuntu PC is required for visualization of ROS topics published from the TI EdgeAI Starter Kit (SK) board. We have tested only with native x86_64 Ubuntu PCs and have **not** tested with any other Ubuntu systems: including Ubuntu virtual machines and Docker Desktop on Mac or Windows.

```{only} tag_ros1n2
It is assumed that a matching ROS distro (either ROS 1 {{ROS1_DISTRO}} or ROS 2 {{ROS2_DISTRO}}) is installed on the remote Ubuntu PC, either in the host Ubuntu filesystem natively or in a Docker container. In case you want to install ROS natively on the remote Ubuntu filesystem:

- For ROS {{ROS1_DISTRO}} installation steps, please refer to [this ROS wiki page](http://wiki.ros.org/noetic/Installation/Ubuntu).
- For ROS 2 {{ROS2_DISTRO}} installation steps, please refer to [this ROS 2 documentation](https://docs.ros.org/en/foxy/Installation.html).
```

```{only} tag_ros2only
It is assumed that a matching ROS distro (ROS 2 {{ROS2_DISTRO}}) is installed on the remote Ubuntu PC, either in the host Ubuntu filesystem natively or in a Docker container. In case you want to install ROS natively on the remote Ubuntu filesystem:

- For ROS 2 {{ROS2_DISTRO}} installation steps, please refer to [this ROS 2 documentation](https://docs.ros.org/en/foxy/Installation.html).
```

```{only} tag_ros1n2
We also provide Dockerfiles (`docker/Dockerfile.x86_64.{noetic,humble}`) that can build and run on the remote Ubuntu PC for both ROS {{ROS1_DISTRO}} and ROS 2 {{ROS2_DISTRO}}, with detailed steps for setting up the Docker environment on the remote PC. For installation of Docker on the Ubuntu PC, the following links may be useful:
```

```{only} tag_ros2only
We also provide Dockerfiles (`docker/Dockerfile.x86_64.humble`) that can build and run on the remote Ubuntu PC for ROS 2 {{ROS2_DISTRO}}, with detailed steps for setting up the Docker environment on the remote PC. For installation of Docker on the Ubuntu PC, the following links may be useful:
```

- [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
- [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

### Cameras

```{only} tag_j7x
The Robotics SDK provides camera ROS nodes for [ZED stereo camera](https://www.stereolabs.com/zed/), USB mono cameras (Logitech C270, C920, C922, and others), and CSI cameras including IMX219 and IMX390. All the demo applications can be tried out with a live camera as well as with a ROSBAG file that is provided with the SDK.

- For configuration of a stereo camera, please see [ros1/drivers/zed_capture/README](../ros1/drivers/zed_capture/README.md).
- For configuration of a USB mono camera, please see [ros1/drivers/mono_capture/README](../ros1/drivers/mono_capture/README.md).
- For configuration of CSI cameras, please see [this section of Processor SDK Linux documentation](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/TDA4VM/latest/exports/edgeai_docs/devices/TDA4VM/linux/getting_started.html#hardware-setup).
- For GStreamer camera ROS node (USB cameras are also supported), please see [ros2/drivers/gscam2/README_TI](../ros2/drivers/gscam2/README_TI.md)
```
```{only} tag_am62a
The Robotics SDK provides camera ROS nodes for USB mono cameras (Logitech C270, C920, C922) and CSI cameras including IMX219 and IMX390. All the demo applications can be tried out with a live camera as well as with a ROSBAG file that is provided with the SDK.

- For configuration of a USB mono camera, please see [ros1/drivers/mono_capture/README](../ros1/drivers/mono_capture/README.md).
- For configuration of CSI cameras, please see [this section of Processor SDK Linux documentation](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/TDA4VM/latest/exports/edgeai_docs/devices/TDA4VM/linux/getting_started.html#hardware-setup).
- For GStreamer camera ROS node (USB cameras are also supported), please see [ros2/drivers/gscam2/README_TI](../ros2/drivers/gscam2/README_TI.md)
```

## Set Up Development Environment on the Target

![](docs/tiovx_ros_setup.svg)
<figcaption>Figure 1. Robotics SDK Setup and Installation Steps</figcaption>
<br />

Figure 1 shows the hardware setup and high-level installation steps on the target SK board and the Ubuntu PC. The target SK board and the remote Ubuntu PC are assumed to be connected on the same network through Ethernet or WiFi connectivity. For details on how to set up the WiFi on the SK board, please refer to {{'[this section of Edge AI documentation](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/TDA4VM/{}/exports/edgeai_docs/devices/TDA4VM/linux/wifi_oob_demo.html)'.format(SDK_VER)}}.

### Build SD Card

1. From the Ubuntu PC, download the **SD card image (`tisdk-edgeai-image-<platform>-evm.wic.xz`)** from Processor SDK Linux (using the links provided in Section 1.2).

2. Flash the downloaded image to an SD card (minimum 32GB, high-performance) using the Balena Etcher tool. For detailed instructions, please refer to {{'[this section](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/TDA4VM/{}/exports/edgeai_docs/devices/TDA4VM/linux/getting_started.html#software-setup)'.format(SDK_VER)}}.

````{note}
The etcher image is created for 16 GB SD cards. If you are using a larger SD card, it is highly recommended to expand the root filesystem to use the full SD card capacity using the following steps on the Ubuntu PC.

```
# find the SD card device entry using lsblk (Eg: /dev/sdb)
# use the following commands to expand the filesystem

umount /dev/sdX1
umount /dev/sdX2
sudo parted -s /dev/sdX resizepart 2 '100%'
sudo e2fsck -f /dev/sdX2
sudo resize2fs /dev/sdX2

# replace /dev/sdX in above commands with SD card device entry
```
````

### Connect Remotely to the Target

1. To find the target IP address assigned to the EVM, use a serial port communications program (for example, `sudo minicom -D /dev/ttyUSBx` where `/dev/ttyUSBx` is the device for the UART serial port), log in with `root` account, and run `ifconfig`.

2. From a terminal on the PC, open an SSH session to connect remotely to the target:
    ```
    user@pc:~$ ssh root@<target_IP_address>
    ```
    ```{tip}
    It is recommended to use a *static* IP for the SK board to make ROS network setting easy.
    ```
    <!-- You can consider using VS Code with "remote development extension pack" for a better experience, in a similar way as described in {{'[this section of Edge AI documentation](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/TDA4VM/{}/exports/edgeai_docs/devices/TDA4VM/linux/getting_started.html#connect-remotely)'.format(SDK_VER)}}. -->

### Initial Setup for the Robotics SDK

#### On the SK Board
You can run the installation script on the target as follows:
```
root@am6x-sk:~$ cd /opt/edgeai-gst-apps/scripts
root@am6x-sk:~$ source ./install_robotics_sdk.sh
```
This script takes care of:
- Cloning the main GIT repository for Robotics SDK under `/opt/robotics_sdk`
- Setting up the folders for evaluating the Robotics SDK under `$HOME/j7ros_home`
- Downloading ROSBAG and other data files
- Downloading several deep-learning models from the edge AI model zoo. You can also use the model downloader tool (please refer to {{'[this section of Edge AI documentation](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/TDA4VM/{}/exports/edgeai_docs/common/inference_models.html)'.format(SDK_VER)}}).

#### On the Remote Ubuntu PC
In a similar way, you can use a script to set up on the remote Ubuntu PC for visualization:
```
user@pc:~$ wget -O init_setup.sh https://git.ti.com/cgit/processor-sdk-vision/jacinto_ros_perception/plain/init_setup.sh
user@pc:~$ source ./init_setup.sh REL.09.00.00
```

````{note}
In a proxy network, in case the `wget` command above does not work, you can try again with adding `--proxy off` argument:
```
user@pc:~$ wget --proxy off -O init_setup.sh https://git.ti.com/cgit/processor-sdk-vision/jacinto_ros_perception/plain/init_setup.sh
```
````

### Set Up Docker Environment on the Target

The Robotics SDK runs in a Docker container environment on the Processor SDK Linux. In the Robotics SDK Docker environment, ROS and necessary libraries and tools are installed.

First, following [this link](https://docs.docker.com/get-started/#test-docker-installation), please check that Docker and network work correctly on the target SK board.

```{only} tag_ros1n2
The following two sections describe the Docker environment setup, details of building, and running apps under Docker, for ROS 1 and ROS 2, respectively. Please note that it will take several minutes to build the Docker image. The Docker image built can be listed with `docker images`.

- [Docker Setup for ROS 1 {{ROS1_DISTRO}}](./setting_docker_ros1.md)
- [Docker Setup for ROS 2 {{ROS2_DISTRO}}](./setting_docker_ros2.md)
```
```{only} tag_ros2only
The following section describes the Docker environment setup, details of building, and running apps under Docker. Please note that it will take several minutes to build the Docker image. The Docker image built can be listed with `docker images`.

- [Docker Setup for ROS 2 {{ROS2_DISTRO}}](./setting_docker_ros2.md)
```

```{note}
**Proxy Network**: If the board running the Docker container is behind a proxy server, the default settings for downloading files and installing packages via `apt-get` may not work, and proper settings for proxy should be established for building and running the SDK Docker image.
```

```{tip}
**Docker Run**: After "docker build" is completed, it is important to use `docker_run_rosX.sh` script to start a Docker container, since the script includes all the necessary settings to leverage all the cores and hardware accelerators of the processor. Please note that `docker_run_rosX.sh` includes `--rm` argument by default. Just remove the `--rm` argument in `docker_run_rosX.sh` in case you want to do "docker commit" after exiting a Docker container. A short information about several useful Docker commands is provided in {{'[this link](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/TDA4VM/{}/exports/edgeai_docs/devices/TDA4VM/linux/docker_environment.html#additional-docker-commands)'.format(SDK_VER)}}.
```

`````{only} tag_ros1n2
````{tip}
**Switching between ROS 1 and ROS 2 Docker Containers**: When the applications are built under the ROS 1 container, two directories, `{build, devel}` are created under **ros_ws** directory under target host and similarly `{build, install, log}` directories are created when applications are built under ROS 2 container. Since the containers share the common space `ros_ws` on target host Linux filesystem, these directories need to be removed if switching between ROS 1 and ROS 2 containers. Alternatively, we can create different folders for each of ROS distro and apply soft-links as shown below as example before running "docker run" script.

```
# Before launching ROS 1 container, establish the following soft-links:
$ROS_WS/build -> $ROS_WS/build_noetic
$ROS_WS/devel -> $ROS_WS/devel_noetic

# Before launching ROS 2 container, establish the following soft-links:
$ROS_WS/build -> $ROS_WS/build_foxy
$ROS_WS/install -> $ROS_WS/devel_foxy
$ROS_WS/log -> $ROS_WS/log_foxy
```
````
`````