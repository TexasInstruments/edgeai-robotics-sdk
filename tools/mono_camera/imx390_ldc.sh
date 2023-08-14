#!/bin/bash
# Run this script in Robotics SDK Docker container on the target.
# bash /opt/robotics_sdk/tools/mono_camera/imx390_ldc.sh

# Python venv is set up in the SDK Docker container
. /opt/venv/bin/activate

PYTHON=python3
CAMERA_INFO_FILE=imx390_35244_equidistant_camera_info.yaml
CAMERA_NAME='imx390_35244'

ARCH=`arch`
if [[ "$ARCH" == "aarch64" ]]; then
    SDK_DIR=/opt/robotics_sdk
elif [[ "$ARCH" == "x86_64" ]]; then
    SDK_DIR=${HOME}/j7ros_home/ros_ws/src/robotics_sdk
    echo "Please make the resulting files *.bin and *.yaml copied on the target under /opt/imaging/imx390."
fi
SRC_DIR=$SDK_DIR/tools/mono_camera
CAMERA_INFO_PATH=$SRC_DIR/$CAMERA_INFO_FILE
if [[ "$ARCH" == "aarch64" ]]; then
    cp $CAMERA_INFO_PATH /opt/imaging/imx390
    CAMERA_INFO_PATH=/opt/imaging/imx390/$CAMERA_INFO_FILE
fi

SCRIPT_PATH=$SDK_DIR/tools/mono_camera/generate_rect_map_mono.py
${PYTHON} $SCRIPT_PATH -i $CAMERA_INFO_PATH -n $CAMERA_NAME --width 1920 --height 1080
${PYTHON} $SCRIPT_PATH -i $CAMERA_INFO_PATH -n $CAMERA_NAME --width 1280 --height 720
${PYTHON} $SCRIPT_PATH -i $CAMERA_INFO_PATH -n $CAMERA_NAME --width 960 --height 540

deactivate