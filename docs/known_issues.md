Limitations and Known Issues
============================

````{only} tag_j7x
1. RViz Visualization: RViz visualization is displayed on a remote Ubuntu PC.

1. Ctrl+C Termination: In some cases, terminating a ROS node or a ROS launch using Ctrl+C can be slow and may escalate to SIGTERM. This behavior is more frequent in the ROS 2 container.

1. Stereo Vision Demo:
    - The output disparity map may contain artifacts that are common to block-based stereo algorithms. e.g., noise in the sky, texture-less area, repeated patterns, etc.
    - While the confidence map from SDE has 8 values between 0 (least confident) to 7 (most confident), the confidence map from the multi-layer SDE refinement has only 2 values: 0 and 7. Therefore, the confidence map from the refinement may not appear as fine as the SDE's confidence map.

1. Default Semantic Segmentation Model: The default semantic segmentation model used in `ti_vision_cnn` and `ti_estop` nodes was initially trained with the Cityscapes dataset and subsequently re-trained with a small dataset collected from a stereo camera (ZED camera, HD mode). This re-training was done for a limited set of scenarios with coarse annotation. As a result, the model's accuracy performance may be limited when used with a different camera model or in different environmental scenes.

1. ROS 2 Demo Launch: Launching a demo in the ROS 2 environment with "ros2 bag play" in a single launch script currently not stable. It is recommended to launch “ros2 bag play” in a separate terminal. The demos in ROS 2 container currently run more stable with live cameras (ZED stereo camera or USB mono camera).


1. ZED Camera Recognition: The ZED camera is sometimes recognized as USB 2.1 device. The ZED camera should be connected to USB 3.0 for the demos. You can verify the USB version using the following command:
    ```text
    # For ZED camera
    root@am6x-sk:~# lsusb -v -d 2b03:f582 | grep bcdUSB
    bcdUSB               3.00
    # For ZED 2 camera
    root@am6x-sk:~# lsusb -v -d 2b03:f780 | grep bcdUSB
    bcdUSB               3.00
    ```
````

```{only} tag_am62a
1. RViz Visualization: RViz visualization is displayed on a remote Ubuntu PC.

1. Ctrl+C Termination: In some cases, terminating a ROS node or a ROS launch using Ctrl+C can be slow and may escalate to SIGTERM. This behavior is more frequent in the ROS 2 container.

1. Default Semantic Segmentation Model: The default semantic segmentation model used in `ti_vision_cnn` and `ti_estop` nodes was initially trained with the Cityscapes dataset and subsequently re-trained with a small dataset collected from a stereo camera (ZED camera, HD mode). This re-training was done for a limited set of scenarios with coarse annotation. As a result, the model's accuracy performance may be limited when used with a different camera model or in different environmental scenes.

1. ROS 2 Demo Launch: Launching a demo in the ROS 2 environment with "ros2 bag play" in a single launch script currently not stable. It is recommended to launch “ros2 bag play” in a separate terminal. The demos in ROS 2 container currently run more stable with live cameras (ZED stereo camera or USB mono camera).
```