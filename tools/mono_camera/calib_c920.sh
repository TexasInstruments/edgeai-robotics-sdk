#!/bin/bash
# Example script for camera calibration
# Please run this script after downloading example calibration images.
#   At Ubuntu PC command line (not in Docker container):
#   ~/j7ros_home$ make calib_download

# Python venv is set up in the SDK Docker container
. /opt/venv/bin/activate

PYTHON=python3
J7ROS_DIR=${HOME}/j7ros_home
ARCH=$(arch)
if [[ "$ARCH" == "x86_64" ]]; then
    VIZ_FLAG=True
else
    VIZ_FLAG=False
fi

if [[ -z "$SDK_DIR" ]]; then
    if [[ "$ARCH" == "aarch64" ]]; then
        export SDK_DIR=/opt/robotics_sdk
    elif [[ "$ARCH" == "x86_64" ]]; then
        export SDK_DIR=$ROS_WS/src/robotics_sdk
    else
        echo "$ARCH is not supported"
        exit 1
    fi
fi
CAMINFO_FILE="${SDK_DIR}/ros1/drivers/mono_capture/config/C920_HD_camera_info.yaml"
CAMERA_NAME=C920

# camera calibration
${PYTHON} camera_calibration.py \
    -p ${J7ROS_DIR}/data/calib_images_example \
    -e jpg \
    -o ${CAMINFO_FILE} \
    -r 9 -c 7 -s 25 \
    -v ${VIZ_FLAG} \
    -n ${CAMERA_NAME}

# generate rectification map LUT
${PYTHON} generate_rect_map_mono.py \
    -i ${CAMINFO_FILE} \
    -n ${CAMERA_NAME}

deactivate