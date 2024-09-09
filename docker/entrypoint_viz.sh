#!/bin/bash
set -e

# set up proxy as needed
source /root/setup_proxy.sh

# Ubuntu version
UBUNTU_VER=$(lsb_release -r | cut -f2)
echo "Ubuntu $UBUNTU_VER. ROS-$ROS_DISTRO"

# set up the ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# set up the SDK ROS packages pre-installed in the container
SETUP_FILE="/opt/ros/robotics_sdk_install/setup.bash"
if [ -f "$SETUP_FILE" ]; then
    source "$SETUP_FILE"
fi

# set up the ROS packages in the ROS workspace
if [ "$ROS_VERSION" == "1" ]; then
    # ROS network settings
    if [ -z "$J7_IP_ADDR" ]; then
        echo "env variable J7_IP_ADDR is not set"
    fi
    if [ -z "$PC_IP_ADDR" ]; then
        echo "env variable PC_IP_ADDR is not set"
    fi
    export ROS_MASTER_URI=http://$J7_IP_ADDR:11311
    export ROS_IP=$PC_IP_ADDR

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

else
    echo "Invalid ROS_VERSION"
fi

exec "$@"
