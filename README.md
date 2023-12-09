# Robotics SDK Introduction

## Git Repository

[Robotics SDK Git Repository](https://git.ti.com/cgit/processor-sdk-vision/jacinto_ros_perception/about/)

## User Guide Documentation

```{only} tag_j7x
- {{'[Robotics SDK for TDA4VM](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/{}/TDA4VM/docs/index.html)'.format(SDK_VER)}}
- {{'[Robotics SDK for AM68A](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/{}/AM68A/docs/index.html)'.format(SDK_VER)}}
- {{'[Robotics SDK for AM69A](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/{}/AM69A/docs/index.html)'.format(SDK_VER)}}
```
```{only} tag_am62a
- {{'[Robotics SDK for AM62A](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/{}/AM62A/docs/index.html)'.format(SDK_VER)}}
```

## Overview

```{only} tag_j7x
The Robotics SDK provides a robotics software development environment for TI Jacinto Processors. It also offers software building blocks and example demos that can be leveraged in robotics software development. The SDK runs in Docker container environments on:

- [Processor SDK Linux for TDA4VM {{RELEASE}}](https://www.ti.com/tool/download/PROCESSOR-SDK-LINUX-SK-TDA4VM)
- [Processor SDK Linux for AM68A {{RELEASE}}](https://www.ti.com/tool/download/PROCESSOR-SDK-LINUX-AM68A)
- [Processor SDK Linux for AM69A {{RELEASE}}](https://www.ti.com/tool/download/PROCESSOR-SDK-LINUX-AM69A)
```

```{only} tag_am62a
The Robotics SDK provides a robotics software development environment for TI Sitara Processors. It also offers software building blocks and example demos that can be leveraged in robotics software development. The SDK runs in Docker container environments on:

- [Processor SDK Linux for AM62A {{RELEASE}}](https://www.ti.com/tool/download/PROCESSOR-SDK-LINUX-AM62A)
```

```{only} tag_ros1n2
In the next section, you will find detailed steps for setting up Docker container environments for ROS {{ROS1_DISTRO}} and ROS 2 {{ROS2_DISTRO}} on the Processor SDK Linux for Edge AI. The Robotics SDK allows:
```
```{only} tag_ros2only
In the next section, you will find detailed steps for setting up Docker container environments for ROS 2 {{ROS2_DISTRO}} on the Processor SDK Linux for Edge AI. The Robotics SDK allows:
```

- Optimized software implementation of computation-intensive software building blocks (including deep-learning, vision, perception, mapping and localization) on deep-learning core (C7x/MMA), DSP cores, hardware accelerators built-in on the processors.
- Application software development on the target using libraries optimized on the TI Processor along with many open-source libraries and packages including, for example, OpenCV, Point-Cloud Library (PCL), and more.

Figure 1 shows the software libraries and components provided by the Robotics SDK.

![](docs/tiovx_ros_sw_stack.png)
 <figcaption>Figure 1. Robotics SDK: Software Stack (Note: hardware diagram varies depending on the platform.) </figcaption>
 <br />

### TI Vision Apps Library

The TI Vision Apps Library is included in the pre-built base image for the Processor SDK Linux for Edge AI. The library provides a set of APIs, including:

- TI OpenVX kernels and software infrastructure
- Imaging and vision applications
- Perception applications
- Advanced driver-assistance systems (ADAS) applications

### Open-Source Deep-Learning Runtime

The Processor SDK Linux for Edge AI also supports the following open-source deep-learning runtime libraries:

- TVM/Neo-AI-DLR
- TFLite Runtime
- ONNX Runtime

For more details on open-source deep-learning runtime on TI Processors, please check [TI Edge AI Cloud](https://dev.ti.com/edgeai/). The Robotics SDK provides a versatile vision CNN node optimized for TI Processors that supports many deep-learning models for object detection, semantic segmentation, and 6D pose estimation operations.

Figure 2 shows a representative deep-learning and compute-intensive demo application developed with the Robotics SDK.

![](docs/tiovx_ros_demo_diagram.svg)
<figcaption>Figure 2. Example Demo Application with Robotics SDK </figcaption>
<br />

## Setting Up Robotics SDK Docker Container Environment

This section describes how to set up the Robotics SDK on top of the Processor SDK Linux for Edge AI. To get started, please refer to our [Setting Up Robotics SDK](docker/README.md) guide.

## Sensor Driver Nodes

The following ROS nodes for cameras have been tested and are supported by the SDK:

```{only} tag_j7x
- [USB Stereo Camera Capture Node for ZED Cameras](ros1/drivers/zed_capture/README.md)
- [GStreamer-based Camera Capture Node](ros1/drivers/gscam/README_TI.md)
- [USB Mono Camera Capture Node](ros1/drivers/mono_capture/README.md)
- [mmWave Radar Driver Node](docs/radar_driver_node_ros2.md)
```
```{only} tag_am62a
- [GStreamer-based Camera Capture Node](ros1/drivers/gscam/README_TI.md)
- [USB Mono Camera Capture Node](ros1/drivers/mono_capture/README.md)
- [mmWave Radar Driver Node](docs/radar_driver_node_ros2.md)
```

## Demo Applications

The SDK supports the following out-of-box demo applications:

```{only} tag_j7x
- [Stereo Vision Processing Accelerated on LDC and SDE](ros1/nodes/ti_sde/README.md)
- [Semantic Segmentation CNN Accelerated on C7x/MMA](ros1/nodes/ti_vision_cnn/README.md)
- [Object Detection CNN Accelerated on C7x/MMA](ros1/nodes/ti_vision_cnn/README_objdet.md)
- [6D Pose Estimation CNN Accelerated on C7x/MMA](ros1/nodes/ti_vision_cnn/README_6D_pose.md)
- [3D Obstacle Detection Accelerated on SDE and C7x/MMA](ros1/nodes/ti_estop/README.md)
- [Vision Object Detection with 3D Spatial Information](ros1/nodes/ti_objdet_range/README.md)
- [Visual Localization Accelerated on C7x/MMA](ros1/nodes/ti_vl/README.md)
<!-- - [2D Lidar SLAM (open-source)](ros1/slam/README.md)
- [AprilTag Detection (open-source)](docs/april_tag.md) -->
```
```{only} tag_am62a
- [Semantic Segmentation CNN Accelerated on C7x/MMA](ros1/nodes/ti_vision_cnn/README.md)
- [Object Detection CNN Accelerated on C7x/MMA](ros1/nodes/ti_vision_cnn/README_objdet.md)
<!-- - [2D Lidar SLAM (open-source)](ros1/slam/README.md)
- [AprilTag Detection (open-source)](docs/april_tag.md) -->
```
![](ros1/nodes/ti_vision_cnn/docs/objdet_rviz.png)
![](ros1/nodes/ti_estop/docs/estop_rviz.png)

<figcaption>Figure 3. Demo Application Examples </figcaption>
<br />

## Scope of Robotics SDK

```{only} tag_j7x
![](docs/sdk_scope_j7x_09_01_00.png)
```
```{only} tag_am62a
![](docs/sdk_scope_am62a_09_01_00.png)
```

## Limitations and Known Issues

See [known_issues.md](docs/known_issues.md) for a list of limitations and known issues.

## Change Log

See [CHANGELOG.md](CHANGELOG.md) for the change log.

## Questions & Feedback

If you have any questions or feedback, please visit [TI E2E](https://e2e.ti.com/support/processors).
