#!/bin/bash
ROS_WS=$HOME/j7ros_home/ros_ws
ARCH=`arch`
if [[ "$ARCH" == "aarch64" ]]; then
    SDK_DIR=/opt/robotics_sdk
elif [[ "$ARCH" == "x86_64" ]]; then
    SDK_DIR=$ROS_WS/src/robotics_sdk
else
    echo "$ARCH is not supported"
    exit 1
fi

CURRENT_DIR=$(pwd)

# install gscam (for ROS1) and apply a patch
WORK_PATH=$SDK_DIR/ros1/drivers
cd $WORK_PATH
if [[ ! -d "gscam" ]]; then
    git clone --single-branch --branch master https://github.com/ros-drivers/gscam.git
    cd gscam
    git checkout -b ti_nv12 tags/1.0.1
    git apply $WORK_PATH/patches/gscam_ti_nv12.patch
    git add -u
    git add config/* launch/*
    git commit -m "Customized for TI Jacinto Robotics SDK: Added pipleline that uses TI GStreamer plugins, and added NV12 encoding mode."
fi

# install gscam2 (for ROS2) and apply a patch
WORK_PATH=$SDK_DIR/ros2/drivers
cd $WORK_PATH
if [[ ! -d "gscam2" ]]; then
    git clone --single-branch --branch main https://github.com/clydemcqueen/gscam2.git
    cd gscam2
    git checkout -b ti_nv12 2c02495167f4fc0afdc88062735b410582a354ce
    git apply $WORK_PATH/patches/gscam2_ti_nv12.patch
    git add -u
    git add config/* launch/*
    git commit -m "Customized for TI Jacinto Robotics SDK: Added pipleline that uses TI GStreamer plugins', and added NV12 encoding mode."
fi

# install dependency of gscam2
cd $WORK_PATH
if [[ ! -d "ros2_shared" ]]; then
    git clone --single-branch --branch master https://github.com/ptrmu/ros2_shared.git
fi

cd $CURRENT_DIR
