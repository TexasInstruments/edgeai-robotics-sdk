#!/bin/bash

#  Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
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

TIME_OUT=5
ROSBAG_TIME_OUT=10
DELAY=3

#Run ROSBAG Files at 1 FPS
RATE_DIV_FACTOR=0.025
POLLING_INTERVAL=1000

ROSBAG_DIR=/root/j7ros_home/data/ros_bag

TEST_COMMAND="ros2 launch ti_vision_cnn bag_semseg_cnn_launch.py"

cd $ROS_WS
source $ROS_WS/install/setup.bash

timeout -s INT -k $(($ROSBAG_TIME_OUT + 4)) $ROSBAG_TIME_OUT ros2 bag record -o $ROSBAG_DIR/sample_SS_test \
        -p $POLLING_INTERVAL /vision_cnn/tensor /camera/image_raw &

sleep $DELAY

timeout -s INT -k $(($TIME_OUT + 4)) $TIME_OUT $TEST_COMMAND