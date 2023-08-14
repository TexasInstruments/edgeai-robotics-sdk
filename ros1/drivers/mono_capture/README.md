Mono Camera Tools and Capture Node
==================================

## Mono Camera Tools

1. **Camera Calibration**: A Python tool for camera calibration is provided: `$SDK_DIR/tools/mono_camera/camera_calibration.py`. This tool calibrates the camera using a set of checkerboard images captured with the target USB camera and generates a `camera_info.yaml` file. Example calibration images are provided. While setting up the Docker environment, the calibration images are downloaded under `$HOME/j7ros_home/data/calib_images`. Below is the usage of the camera calibration tool. 
    ```
    usage: camera_calibration.py [-h] [-p PATH] [-e EXT] [-r PTS_ROWS] [-c PTS_COLUMNS] [-s SQUARE_SIZE] [-v VISUALIZE] [-f SCALING_FACTOR] [-o OUT_FNAME] [-n CAMERA_NAME]

    optional arguments:
    -h, --help            show this help message and exit
    -p PATH, --path PATH  Path to the directory storing images for calibration
    -e EXT, --ext EXT     Extension of the files
    -r PTS_ROWS, --pts_rows PTS_ROWS
                            Number of inner corners per row
    -c PTS_COLUMNS, --pts_columns PTS_COLUMNS
                            Number of inner corners per column
    -s SQUARE_SIZE, --square_size SQUARE_SIZE
                            Size of a square (the edge, by mm) of the checker board
    -v VISUALIZE, --visualize VISUALIZE
                            Visualize the checker board or not
    -f SCALING_FACTOR, --scaling_factor SCALING_FACTOR
                            scaling factor after reading images
    -o OUT_FNAME, --out_fname OUT_FNAME
                            Path to the output file to store result (in YAML format)
    -n CAMERA_NAME, --camera_name CAMERA_NAME
                            Camera name
    ```
    For examples of how to use the tool, please refer to `$SDK_DIR/tools/mono_camera/calib_C920.sh`.

2. **Rectification Map Generatio**n: A Python tool is provided for generating rectification map: `$SDK_DIR/tools/mono_camera/generate_rect_map_mono.py`. This tool takes the `camera_info` YAML file as input and generates a binary file for undistortion/rectification look-up-table (LUT). The generated LUT is required for offloading the undistortion/rectification processes to the VPAC/LDC hardware accelerator. Below is the usage of the tool.

    ```
    usage: generate_rect_map_mono.py [-h] [--input INPUT] [--name NAME]

    optional arguments:
    -h, --help            show this help message and exit
    --input INPUT, -i INPUT
                            YAML file name for camera_info. The file should be stored under ../config folder
    --name NAME, -n NAME  Camera name
    ```
    For examples of how to use the tool, please see `$SDK_DIR/tools/mono_camera/calib_C920.sh`.
## USB Mono Camera ROS Node

This is a ROS node for USB mono cameras (Webcam) based on the OpenCV VideoCapture API. The node publishes raw images and camera information.

```{note}
This ROS node has only been tested with Logitech C920 and C270 webcams in 'YUYV' (YUYV 4:2:2) mode. 'MJPG' (Motion-JPEG) mode is currently not enabled or tested. If you want to use the camera in 'MJPG' mode, it is recommended to use the **gscam** / **gscam2** ROS package instead.
```

### Usage
1. Build the USB mono camera ROS node
    ```sh
    cd $ROS_WS
    # ROS 1
    catkin_make --source /opt/robotics_sdk/ros1 --force-cmake
    source devel/setup.bash
    # ROS 2
    colcon build --base-paths /opt/robotics_sdk/ros2 --cmake-force-configure
    source install/setup.bash
    ```

2. Launch the mono camera node
    ```sh
    # ROS 1
    roslaunch mono_capture mono_capture.launch
    # ROS 2
    ros2 launch mono_capture mono_capture_launch.py
    ```

### Launch File Parameters

| Parameter   | Description                                                                | Value                  |
|-------------|----------------------------------------------------------------------------|------------------------|
| model_str   | Camera model string                                                        | string                 |
| cam_id      | Camera ID number. Set `cam_id = X` if the device shows up as `/dev/video-usb-camX` on the target with TI Edge AI | string |
| camera_mode | Camera mode                                                                | 'FHD' (1920x1080)      |
|             |                                                                            | 'HD' (1280x720)        |
|             |                                                                            | 'VGA' (640x480)        |
| frame_rate  | Frame rate at which raw images are published                               | int                    |
| encoding    | Image encoding                                                             | 'yuv422' (default) or 'bgr8' |
| topic_ns    | Topic namespace                                                            | 'camera' (default)     |

```{note}
When `encoding` is set to 'yuv422', the pixel format YUV422::YUYV from the webcam is converted to YUV422::UYVY format to ensure compatibility with the LDC hardware accelerator.
```