#!/bin/bash
set -e

# setup proxy as required
/root/setup_proxy.sh

# set up ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# setup TI Processor SDK environment
SRC_PATH=$ROS_WS/src/jacinto_ros_perception/docker
FILE="$SRC_PATH/setup_ti_processor_sdk.sh"
if [ -f $FILE ]; then
    source $FILE
else
    echo "$FILE does not exist"
fi

if [ "$ROS_VERSION" == "1" ]; then
    # ROS network settings
    export ROS_MASTER_URI=http://$J7_IP_ADDR:11311
    export ROS_IP=$J7_IP_ADDR

    # source catkin_ws setup.bash if exists
    SETUP_FILE=$ROS_WS/devel/setup.bash
    if [ -f $SETUP_FILE ]; then
        source $SETUP_FILE
    fi

elif [ "$ROS_VERSION" == "2" ]; then
    # source catkin_ws setup.bash if exists
    SETUP_FILE=$ROS_WS/install/setup.bash
    if [ -f $SETUP_FILE ]; then
        source $SETUP_FILE
    fi
else
    echo "Invalid ROS_VERSION"
fi

exec "$@"
