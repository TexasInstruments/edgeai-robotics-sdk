#!/usr/bin/python3

#  Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
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


from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
import math
from pathlib import Path


class Pose:
    def __init__(self) -> None:
        self.x = None
        self.y = None
        self.z = None

    def extractPose(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z


ERROR_THRESHOLD = 0.5

# create reader instance and open for reading
test_bag = Reader('/root/j7ros_home/data/ros_bag/sample_vl_test')
golden_ref_bag = Reader('/root/j7ros_home/data/ros_bag/sample_vl_golden')

with test_bag as test, golden_ref_bag as golden:
    # topic and msgtype information is available on .connections list
    for connection_test, connection_golden in zip(test.connections, golden.connections):
        print(connection_test.topic, connection_test.msgtype)
        print(connection_golden.topic, connection_golden.msgtype)

    frame_count = 1

    test_topic = [x for x in test.connections if x.topic ==
                  '/vision_cnn/tensor']
    golden_topic = [x for x in golden.connections if x.topic ==
                    '/vision_cnn/golden/tensor']

    for test_messages, golden_messages in zip(test.messages(connections=test_topic), golden.messages(connections=golden_topic)):
        connection_test, timestamp_test, rawdata_test = test_messages
        connection_golden, timestamp_golden, rawdata_golden = golden_messages

        msg_test = deserialize_cdr(rawdata_test, connection_test.msgtype)
        msg_golden = deserialize_cdr(rawdata_golden, connection_golden.msgtype)

        targetPose = Pose()
        currentPose = Pose()

        currentPose.extractPose(msg_test)
        targetPose.extractPose(msg_golden)

        currentLoc = [currentPose.x, currentPose.y, currentPose.z]
        targetLoc = [targetPose.x, targetPose.y, targetPose.z]

        error = math.dist(targetLoc, currentLoc)

        if (error < ERROR_THRESHOLD):
            print(f"[VL TEST-CASE] passed for Frame {frame_count}")
        else:
            print(f"[VL TEST-CASE] failed for Frame {frame_count}")

        frame_count += 1
