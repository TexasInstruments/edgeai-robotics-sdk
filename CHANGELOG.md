Change Log
==========

## Release 9.2.0 (2024-04-05)

- Added support for AM67A. For details, please check the [documentation](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/09_02_00/AM67A/docs/). The Robotics SDK now supports five TI Processor platforms: AM62A, TDA4VM, AM67A, AM68A, and AM69A.
- Added a new camera and radar fusion demo application (`ti_objdet_radar`). This application consists consists of CNN object detection, mmWave radar processing chain (on IWR6843), and object-level fusion algorithm.
- Added new ROS-GST plugins (`ti_ros_gst_plugins`). These GStreamer plugins provide an optimized bridge between ROS 2 and the GStreamer framework.
- The vision CNN chain (`ti_vision_cnn`) now includes support for human pose estimation.

## Release 9.1.0 (2023-12-08)

- The vision CNN chain (`ti_vision_cnn`) now includes support for 6D pose estimation. This enhancement is particularly beneficial for applications such as automated bin picking.
- The 3D perception demo application (`ti_objdet_range`), which utilizes CNN object detection and stereo vision, has been successfully migrated to ROS 2.
- The ROS node for the radar driver, which is compatible with TI mmWave radar devices (including the IWR6843), has been successfully migrated to ROS 2.
- Added support for RPi headers and GPIO libraries for SK-AM62A (as part of the Processor SDK Linux for Edge AI).
- For convenience, the SDK Docker container comes pre-configured with a variety of handy aliases. Details about these aliases can be found in `$SDK_DIR/docker/set_aliases.sh`.
- We have begun using ros:humble-perception-jammy as the base image. Consequently, this has led to an increase in the size of the SDK Docker image.

## Release 9.0.0 (2023-08-11)

- Added support for SK-AM62A. For details, please check the [documentation](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/09_00_00/AM62A/docs/). The Robotics SDK now supports four TI Processor platforms: SK-AM62A, SK-TDA4VM, SK-AM68A, and SK-AM69A.
- Migrated ROS 2 to Humble and it is operating in Ubuntu 22.04 Docker container.
- ROS 1 support is not included in 9.0.0 release. However, ROS 1 Noetic is still supported on the four TI platforms with Robotics SDK 8.6.1.
- Support for IMX390 FPD-link CSI camera is added in `gscam2` and `ti_vision_cnn` processing chain in ROS 2.
- The default 2D object detection model has been changed to TFL-OD-2020-ssdLite-mobDet-DSP-coco-320x320.

## Release 8.6.1 (2023-05-31)

- The Robotics SDK now supports two additional platforms: SK-AM68 and SK-AM69, as well as SK-TDA4VM.
- Added support for RPi headers and GPIO libraries for SK-AM68 and SK-AM69 (as part of the Processor SDK Linux for Edge AI).
- Reorganized and updated the Robotics SDK documentation to cover TDA4VM, AM68A, and AM69A.

## Release 8.6.0 (2023-03-03)

- Added a new parameter for controlling the specification of the camera port.
- Added fisheye camera support to the LDC lookup-table generation tool (tools/mono_camera/generate_rect_map_mono.py).
- The GStreamer pipeline for IMX390 camera in `gscam` node now includes `tiovxldc` in mesh image mode. Example LDC lookup-table files are provided that does resizing as well as rectification.
- The default 2D object detection model has been changed to ONR-OD-8020-ssd-lite-mobv2-mmdet-coco-512x512.
- Replaced the OD model TFL-OD-2020-ssdLite-mobDet-DSP-coco-320x320 with TFL-OD-2010-ssd-mobV2-coco-mlperf-300x300

## Release 8.5.0 (2022-12-22)

- Relocated the stereo-processing openVX sub-graphs from the processor SDK vision_apps component into the Robotics SDK.
- Added a new parameter for controlling the number of bounding boxes overlaid, for vision_cnn object detection ROS node.
- Updated Open-source runtime libraries (ONNX, TFLite, TVM-DLR) pre-built for the Ubuntu 20.04 Docker container.
- The default 2D object detection model has been changed to ONR-OD-8050-ssd-lite-regNetX-800mf-fpn-bgr-mmdet-coco-512x512.
- Added the documents on how to run RealSense camera and AprilTag ROS package.

## Release 8.4.0 (2022-09-19)

- ROS development environment in a Docker container on top of Processor Linux SDK for edge AI 8.4.0 pre-built image.
- Open-source runtime libraries (ONNX, TFLite, TVM-DLR) pre-built for the Ubuntu 20.04 Docker container are installed.
- Support for IMX390 FPD-link camera is added in `gscam` and `ti_vision_cnn` processing chain.
- New coarse synchronization between multiple mmWave devices in the radar driver ROS node for TI mmWave radar devices
- Code changes in `ti_vision_cnn` node to publish the bounding box data after properly scaling it to output resolution.

## Release 8.2.0 (2022-04-22)

- ROS development environment in a Docker container on top of Processor Linux SDK for edge AI 8.2.0 pre-built image.
- A new 3D perception demo application (`ti_objdet_range`) that is based on CNN object detection and stereo vision is added.
- OpenCV based stereo camera calibration tool is added. Using checkerboard chart images, the Python tool calibrates stereo cameras to update camera parameters and stereo rectification tables.
- Moved the mono and stereo camera tools under `tools` folder.
- Deep-learning runtime Python packages for ONNX, TFLite, TVM-DLR are added in the Robotics SDK ROS Docker containers.
- Radar driver ROS node for TI mmWave radar devices (including IWR6843ISK) is added. The ROS node configures the mmWave device and publishes a ROS PointCloud2 message for the objects detected.
- The default semantic segmentation models are now part of Edge AI model zoo: TVM-SS-5818-deeplabv3lite-mobv2-qat-robokit-768x432, ONR-SS-8818-deeplabv3lite-mobv2-qat-robokit-768x432.
- The default 2D object detection model (ONR-OD-8080-yolov3-lite-regNetX-1.6gf-bgr-mmdet-coco-512x512) is improved. Initial loading time is significantly reduced.
- Added support for WiFi station mode with Intel Wireless-AC 9260 M.2 WiFi module (as part of Processor SDK Linux for Edge AI).
- Hardware PWM is enabled (as part of Processor SDK Linux for Edge AI).

## Release 8.1.0 (2022-02-04)

- Version labeling for the Robotics SDK starts following the version numbers of the Processor SDK Linux releases.
- ROS development environment in a Docker container on top of Processor Linux SDK for edge AI 8.1.0 pre-built image.
- Migrated to ROS Noetic: Most of demo applications run in ROS Noetic as well as in ROS 2 Foxy.
- Added GStreamer-based camera ROS nodes: Open-source 'gscam' (for ROS 1 Noetic) and 'gscam2' (in ROS 2 Foxy) packages are customized to better integrate with TDA4-optimized processing chains, also leveraging the Edge AI GStreamer plugins optimized on TDA4.
- Added CSI camera support: OV5640 CSI camera is supported with the GStreamer-based camera ROS nodes.
- Performance benchmarking report is provided in the documentation.
- Added GPIO libraries for Python and C++ (as part of Processor SDK Linux for Edge AI).
- SDK installation path is moved to /opt/robotics_sdk on the TDA4 root filesystem.

## Version 0.5.0 (2021-09-28)

- ROS development environment in a Docker container on top of Processor Linux SDK for edge AI 8.0.1 pre-built image.
- A new versatile vision CNN inference ROS node is added. This new CNN inference node supports many 2D object detection, and semantic segmentation models from the TI edge AI model zoo, that run in any of ONNX-RT, TFL-RT, and Neo-AI-DLR runtime frameworks.
- Added ROS 2 support: Docker environment for ROS 2 Foxy is provided. Most of demo applications run in ROS 2 Foxy as well as in ROS 1 Melodic.
- A visual localization demo application where DKAZE feature extraction is accelerated on C7/MMA.
- USB mono camera support is added: we provide a OpenCV based ROS driver for capturing raw images from a USB camera (e.g., Logitech C920, C922, C270).
- Camera calibration tool and LDC remap look-up-table generation tool for USB mono cameras are also provided.
- Open-source 2D Lidar SLAM: we provide a script and instruction for evaluating open-source Hector SLAM on TDA4 device.
- Dockerfiles for the remote visualization PC are provided for each of ROS 1 Melodic and ROS 2 Foxy.

## Version 0.4.0 (2021-07-14)

- ROS development environment in a Docker container on top of TI Edge AI 0.5 base image
- Proxy settings are provided for building and running the Docker image behind a proxy network (TI network as an example)

## Version 0.3.0 (2021-04-15)

- ROS development environment in a Docker container on top of Processor SDK 7.3.0 pre-built image
- Enhanced the stereo vision demo application: added point-cloud generation
- Updated the semantic segmentation demo application: migrated to open-source deep-learning runtime (TVM + Neo-AI-DLR)
- Added a new demo application: 3D obstacle detection accelerated on deep-learning core (C7/MMA) and hardware accelerators (SDE, LDC, MSC)
- USB stereo camera ROS driver node for ZED cameras
- Stereo rectification LDC lookup-table generation tool for ZED cameras
- A live USB stereo camera support for all three demo applications (stereo vision, semantic segmentation, and 3D obstacle detection)

## Version 0.1.0 (2020-12-15)

- Released with Processor SDK RTOS 7.1.0
- TI OpenVX (TIOVX) with ROS development framework
- TI Vision Apps Library deployed on the J721e target that enables building applications directly on the target
- Docker container environment on J721e for TIOVX + ROS development framework
- Demo application: stereo vision processing node accelerated on LDC and SDE
- Demo application: CNN semantic segmentation node with TIDL running on C7x/MMA
