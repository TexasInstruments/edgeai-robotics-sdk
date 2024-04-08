ZED Stereo Camera ROS Node
==========================
This is a stereo camera ROS node for the ZED camera, based on the OpenCV VideoCapture API. The ROS node publishes left and right raw images and their camera_info.

## Usage

1. Obtain the factory camera calibration data file for your ZED camera with a script provided.

    ```sh
    cd $SDK_DIR/tools/stereo_camera
    ./download_calib_file.sh <serial_number>
    ```
    Replace `<serial_number>` with the serial number of the ZED camera (found on the original box).
    Place the downloaded calibration data file (`SNxxxx.conf`) under the `$SDK_DIR/ros1/drivers/zed_capture/config` folder.

2. Generate `camera_info` YAML files and undistortion/rectification look-up-table (LUT) files, which are required in offloading the undistortion/rectification on the VPAC/LDC hardware accelerator. It is recommended to perform the following steps in the Robotics SDK ROS 1 container on the Ubuntu PC, and then "scp" the output artifacts to the target Linux filesystem, under `/opt/robotics_sdk/ros1/drivers/zed_capture/config`.

    Run the following script:
    ```sh
    cd $SDK_DIR/tools/stereo_camera
    python3 generate_rect_map.py -i SNxxxx.conf -m <camera_mode> -p <output_folder_path>
    ```
    - Replace `SNxxxx.conf` with the factory calibration data file obtained from Step 1.
    - `<camera_mode>` is the camera mode. Valid camera mode are: 2K, FHD, FHD2, HD, HD2, VGA (see ``Launch File Parameters`` section for description). If the `-m` argument is not provided, the tool will iterate by default for the following three camera modes: HD, HD2, FHD, and FHD2.

    This script parses the calibration data and generates the following files under `<output_folder_path>`:

    - `<SN_string>_<camera_mode>_camera_info_{left,right}.yaml`: `camera_info` for left and right raw image,
    - `<SN_string>_<camera_mode>_remap_LUT_{left,right}.bin`: undistortion/rectification remap LUT for left and right raw images.

3. Build the ZED camera ROS node
    ```sh
    cd $ROS_WS
    # ROS 1
    catkin_make --source /opt/robotics_sdk/ros1 --force-cmake
    source devel/setup.bash
    # ROS 2
    colcon build --base-paths /opt/robotics_sdk/ros2 --cmake-force-configure
    source install/setup.bash
    ```

4. Launch the ZED camera node: Update `zed_sn_str` in `launch/zed_capture.launch` (`launch/zed_capture_launch.py` in ROS 2. Remember to run `colcon build` after any modification in ROS 2), or pass as a lauch argument as follows:
    ```sh
    # ROS 1
    roslaunch zed_capture zed_capture.launch zed_sn_str:=SNxxxxx
    # ROS 2
    ros2 launch zed_capture zed_capture_launch.py zed_sn_str:=SNxxxxx
    ```
## Stereo Camera Calibration

For cases where the ZED cameras needs to be recalibrated for some reason, we provides a OpenCV based calibration tool. For more information, refer to [Stereo Camera Calibration](../../../tools/stereo_camera/calibration/README.md).

## Launch File Parameters

| Parameter     | Description                                                               | Value                |
|---------------|---------------------------------------------------------------------------|----------------------|
| zed_sn_str    | ZED camera serial number string, which should start with 'SN' followed by the serial number | string       |
| cam_id        | Camera device number. Use `cam_id = X` if the device shows up as `/dev/video-usb-camX` on the target | string   |
| camera_mode   | ZED camera mode                                                           | '2K' (2208x1242)     |
| _             | _                                                                         | 'FHD' (1920x1080)    |
| _             | _                                                                         | 'FHD2' (1920x1024)   |
| _             | _                                                                         | 'HD' (1280x720)      |
| _             | _                                                                         | 'HD2' (1280x720)     |
| _             | _                                                                         | 'VGA' (672x376)      |
| frame_rate    | Frame rate at which raw images are published                              | int                  |
| encoding      | Image encoding                                                            | 'yuv422' (default) or 'bgr8'   |

'FHD2' is a newly added mode that provides a resolution of 1920x1024 resolution that is supported by the stereo depth engine (SDE) hardware accelerator. The images are obtained by cropping top and bottom evenly the original 1080p images captured from the 'FHD' mode.

'HD2' is a newly added mode that provides a resolution of 720p for experiments that need a longer focal length than the native 'HD'. The images are obtained by center-cropping the original 1080p images captured from the 'FHD' mode.

When `encoding` is set to 'yuv422', the pixel format YUV422::YUYV from the ZED camera is converted to YUV422::UYVY format, considering the compatibility with LDC hardware accelerator.

## Data Collection Steps

1. Connect the ZED camera to UBS 3 port (it is recommended to use a USB type-A to type-C adaptor on TDA4VM). Check if the camera is recognized with the command `ls /dev/video*`. If necessary, update `launch/zed_capture.launch` to change video device, camera mode, frame rate, and etc.

2. On the first terminal, launch the ZED capture node with following and keep it running:
    ```sh
    roslaunch zed_capture zed_capture.launch zed_sn_str:=SNxxxxx
    ```

3. On the second terminal, to capture into ROS bag files, run one of two examples below

    ```sh
    # Collect 15 seconds of data and stop itself
    roslaunch zed_capture recordbag.launch
    # Save into a series of bag files, each keeping 15 seconds of data, until terminated with Ctrl+C
    roslaunch zed_capture recordbag_split.launch
    ```

4. (Optional: to check the ROS topics) On 3rd terminal, run the following commands:
    ```sh
    rqt_image_view /camera/left/image_raw
    ```

By default, ROS bag files are stored under `${HOME}/.ros` folder.

```{note}
It is not recommended to combine the `zed_capture` node and ROSBAG capture into a single launch file, as it takes several seconds for the ZED camera to settle down its ISP tuning after the ZED camera node is started.
```
