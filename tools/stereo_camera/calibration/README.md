Stereo Camera Calibration
==========================

The relative positions of left and right cameras can slightly change over time. In such cases, the factory calibration data for a ZED camera cannot be used, and it is necessary to recalibrate the ZED camera to update camera parameters and stereo rectification tables. For this purpose, we provide three OpenCV-based Python scripts. Note that these scripts are designed to run on Ubuntu PC. Although these scripts are tested with ZED cameras, they can be adapted for calibrating other stereo cameras with minor changes if needed.

## Stereo Camera Capture

Multiple stereo images should be captured with a checkerboard using `stereo_capture.py` script.

```
usage: python3 stereo_capture.py [-h] [-p PATH] [-m CAM_MODE] [-d DEV_NUM]

optional arguments:
    -h, --help                    Show this help message and exit.
    -p PATH, --path PATH          Path to the directory where snapshots will be stored for calibration.
                                  Left and right images are saved in PATH/imageL and PATH/imageR, respectively.
    -m CAMMODE, --mode CAMMODE    Camera mode. FHD, FHD2, HD and VGA are supported. FHD2 is 1920x1024, which is cropped from FHD.
    -d DEVNUM, --device DEVNUM    Device number, e.g., 0 for /dev/video0, 2 for /dev/video2.
```

It is recommended to use a 9x7 checkerboard chart. The checker board chart can be downloaded from [calib.io](https://calib.io). To ensure good calibration, capture multiple snapshots, ideally more than 20. Present the checkerboard chart in various poses, covering different locations within the field of view and at different distances from the camera. Keep the camera in the same position while moving the checkerboard around. Avoid poses where the checkerboard angle is too close to 90 degrees.

The following figures depict examples of good and bad poses.

<!-- <p align="center">
    <img src="docs/checkerboard_good.png" width="150">
    <img src="docs/checkerboard_good2.png" width="180">
    <img src="docs/checkerboard_fair.png" width="180">
    <img src="docs/checkerboard_bad.png" width="140">
    <img src="docs/checkerboard_bad2.png" width="120">
</p> -->
![](docs/stereo_calib_fig1.png)
<figcaption>Figure 1. Good or Bad checkerboard poses: From left to right, good, good, fair, bad and bad.</figcaption>
<br />

Once `stereo_capture.py` is launched, both left and right image sequences will be displayed in the same window. To save each image snapshot, position the mouse over the image window and press the 's' key. The left and right images will be saved in `PATH/imageL` and `PATH/imageR`, respectively, where `PATH` is the directory specified when launched. After capturing all the images, press 'q' to exit. The following figures show the examples of checkerboard snapshots.

<!-- <p align="center">
    <img src="docs/leftImage01.png" width="350"> <img src="docs/rightImage01.png" width="350">
    <img src="docs/leftImage02.png" width="350"> <img src="docs/rightImage02.png" width="350">
    <img src="docs/leftImage03.png" width="350"> <img src="docs/rightImage03.png" width="350">
    <img src="docs/leftImage04.png" width="350"> <img src="docs/rightImage04.png" width="350">
    <img src="docs/leftImage05.png" width="350"> <img src="docs/rightImage05.png" width="350">
</p> -->
![](docs/stereo_calib_fig2.png)
<figcaption>Figure 2. Examples of checkerboard snapshots.</figcaption>
<br />

## Stereo Camera Calibration

With multiple stereo images with the checkerboard chart, we can now ready to run stereo camera calibration.

```
Usage: python3 stereo_calibration.py [-h] [-p PATH] [-m CAM_MODE] [-w WIDTH] [-h HEIGHT] [-s SIZE]

Optional Arguments:
    -h, --help                    Show this help message and exit.
    -p PATH, --path PATH          Path to the directory where snapshots are stored for calibration.
                                  Left and right images are saved in PATH/imageL and PATH/imageR, respectively.
    -m CAMMODE, --mode CAMMODE    Camera mode. FHD, FHD2, HD and VGA are supported. FHD2 is 1920x1024, which is cropped from FHD.
    -c COLUMN, --column COLUMN    Checkerboard width in terms of the number of control points
    -r ROW, --row ROW             Checkerboard height in terms of the number of control points
    -s SIZE, --size SIZE          Checkerboard grid size in mm

Usage Example: See stereo_calibration.sh
```

It first detects the corner points from all the saved left and right images. The figures below show the detected corner points from the left and right pairs. It is not always successful in detecting the corner points from all images; it may fail sometimes for certain images. Additionally, even though all the control points are detected from a left and right pair, the order of the corner points may not be correct. To account for potential errors in corner point detection, it is recommended to save 3 or 4 additional stereo image pairs when capturing snapshots with `stereo_capture.py`.

<!-- <p align="center">
    <img src="docs/leftImage_check01.png" width="350"> <img src="docs/rightImage_check01.png" width="350">
    <img src="docs/leftImage_check02.png" width="350"> <img src="docs/rightImage_check02.png" width="350">
    <img src="docs/leftImage_check03.png" width="350"> <img src="docs/rightImage_check03.png" width="350">
    <img src="docs/leftImage_check04.png" width="350"> <img src="docs/rightImage_check04.png" width="350">
    <img src="docs/leftImage_check05.png" width="350"> <img src="docs/rightImage_check05.png" width="350">
</p> -->
![](docs/stereo_calib_fig3.png)
<figcaption>Figure 3. Detected corner points.</figcaption>
<br />

After detecting the corner points from all images, the camera calibration for left and right images, and the stereo rectification are performed. After the calibration, the tool performs the following steps:

1. Prints the calibration errors, e.g.,
    ```text
    ============================
    Calibration RMS error
    Left   = 0.42999868487598675
    Right  = 0.45839071430768813
    Stereo = 0.39342797821643921
    ============================
    ```
    If one of errors is larger than 1.0, it is recommended to redo calibration. There are a couple of main reasons that cause high RMS error. First of all, ensure that the checkerboard were placed at various distances from camera and various positions. Otherwise, it is recommended to capture stereo images again. Second, verify that the order of detected corner points is correct. Such images are detected and discarded after the corner point detection, but there may still be some remaining. For the stereo depth engine (SDE), the RMS error less than 0.5 is recommended.

2. Creates camera info YAML files for left and right cameras, e.g., `custom_CAMMODE_camera_info_left.yaml` and `custom_CAMMODE_camera_info_right.yaml`, and saves them under `zed_capture/config` folder.

3. Creates LDC look-up-tables for left adn right cameras, e.g., `custom_CAMMODE_LUT_left.bin` and `custom_CAMMODE_LUT_right.bin`, and saves them under `zed_capture/config` folder. These tables can be used for the SDE application on the target.

4. Creates a stereo remap table, `stereoMap.xml` in the format that the OpenCV rectification can read, and saves it under `PATH` which is one specified when the tool is launched.

## Stereo Camera Calibration Test

The stereo calibration can be tested on the Ubuntu PC before testing it on J7.

```
usage: python3 stereo_test.py [-h] [-p PATH] [-m CAM_MODE] [-d DEV_NUM]

optional arguments:
    -h, --help                    Show this help message and exit.
    -p PATH, --path PATH          Path to the directory where snapshots will be stored for calibration.
                                  Left and right images are saved in PATH/imageL and PATH/imageR, respectively.
    -m CAMMODE, --mode CAMMODE    Camera mode. FHD, FHD2, HD and VGA are supported. FHD2 is 1920x1024, which is cropped from FHD.
    -d DEVNUM, --device DEVNUM    Device number, e.g., 0 for /dev/video0, 2 for /dev/video2.
```

The script reads `PATH/stereoMap.xml`, where `PATH` is the directory specified during the launch, to rectify raw inputs from the left and right cameras. As shown in Figure 4, it displays the rectified left and right sequences with overlaid horizontal lines, aiding in visual evaluation. Based on this, you can determine whether the stereo camera requires recalibration.

<!-- <p align="center">
    <img src="docs/leftRectified.png" width="350"> <img src="docs/rightRectified.png" width="350">
</p> -->
![](docs/stereo_calib_fig4.png)
<figcaption>Figure 4. Rectified left and right images</figcaption>
