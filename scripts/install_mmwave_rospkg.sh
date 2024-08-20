#!/bin/bash
ROS_VER=${1:-"2"}
ROS_WS=$HOME/j7ros_home/ros_ws
ARCH=$(arch)
if [[ -z "$SDK_DIR" ]]; then
    if [[ "$ARCH" == "aarch64" ]]; then
        SDK_DIR=/opt/robotics_sdk
    elif [[ "$ARCH" == "x86_64" ]]; then
        SDK_DIR=$ROS_WS/src/robotics_sdk
    else
        echo "$ARCH is not supported"
        exit 1
    fi
fi

# install mmwave_rospkg
REPO_URL="https://git.ti.com/git/mmwave_radar/mmwave_ti_ros.git"
COMMIT="3cfb96e29db1a5728a9c45f968ae2739d370f58f"
BRANCH="master"
if [ "$ROS_VER" == "1" ]; then
    WORK_PATH=$SDK_DIR/ros1/drivers
elif [ "$ROS_VER" == "2" ]; then
    WORK_PATH=$SDK_DIR/ros2/drivers
else
    echo "ROS_VER=$ROS_VER not supported"
    exit 1
fi

CURRENT_DIR=$(pwd)
cd $WORK_PATH
if [[ ! -d "ti_mmwave_rospkg" ]]; then
    echo "[ti_mmwave_rospkg] Installing..."
    git clone --single-branch --branch $BRANCH $REPO_URL > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "Failed to clone the repository."
        exit 1
    fi
    cd mmwave_ti_ros
    git checkout $COMMIT > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "Failed to switch to commit $COMMIT."
        exit 1
    fi
    cd ..
    cp -r mmwave_ti_ros/ros${ROS_VER}_driver/src/* .
    rm -rf mmwave_ti_ros
    echo "[ti_mmwave_rospkg] Done"
fi
cd $CURRENT_DIR
