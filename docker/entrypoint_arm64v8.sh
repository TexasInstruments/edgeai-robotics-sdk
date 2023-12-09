#!/bin/bash
set -e

# setup proxy as required
source /root/setup_proxy.sh

# Ubuntu version and ROS distro
UBUNTU_VER=$(lsb_release -r | cut -f2)
echo "Ubuntu $UBUNTU_VER. ROS-$ROS_DISTRO"

# set up ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# setup TI Processor SDK environment
SRC_PATH=/opt/robotics_sdk/docker
FILE="$SRC_PATH/setup_ti_processor_sdk.sh"
if [ -f $FILE ]; then
    source $FILE
else
    echo "$FILE does not exist"
fi

if [ "$ROS_VERSION" == "1" ]; then
    # ROS network settings
    if [ -z "$J7_IP_ADDR" ]; then
        echo "env variable J7_IP_ADDR is not set"
    fi
    export ROS_MASTER_URI=http://$J7_IP_ADDR:11311
    export ROS_IP=$J7_IP_ADDR

    # source ros_ws setup.bash if exists
    SETUP_FILE=$ROS_WS/devel/setup.bash
    if [ -f $SETUP_FILE ]; then
        source $SETUP_FILE
    fi

elif [ "$ROS_VERSION" == "2" ]; then
    # source ros_ws setup.bash if exists
    SETUP_FILE=$ROS_WS/install/setup.bash
    if [ -f $SETUP_FILE ]; then
        source $SETUP_FILE
    fi

    # exprimental: https://docs.ros.org/en/foxy/Guides/DDS-tuning.html
    sysctl net.ipv4.ipfrag_high_thresh=134217728

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

else
    echo "Invalid ROS_VERSION"
fi

# ROS workspace setup
cd $WORK_DIR
mkdir -p ${ROS_DISTRO}_ws
ln -snf ${ROS_DISTRO}_ws ros_ws
cd $ROS_WS

exec "$@"
