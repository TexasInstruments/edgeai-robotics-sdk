#!/bin/bash

#  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set -e

# set up ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [ "$ROS_VERSION" == "1" ]; then
    # ROS network settings
    if [ -z "$J7_IP_ADDR" ]; then
        echo "env variable J7_IP_ADDR is not set"
    fi
    export ROS_MASTER_URI=http://$J7_IP_ADDR:11311

    if [[ $(arch) == "aarch64" ]]; then
        export ROS_IP=$J7_IP_ADDR
    else
        if [ -z "$PC_IP_ADDR" ]; then
            echo "env variable PC_IP_ADDR is not set"
        fi
        export ROS_IP=$PC_IP_ADDR
    fi

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
