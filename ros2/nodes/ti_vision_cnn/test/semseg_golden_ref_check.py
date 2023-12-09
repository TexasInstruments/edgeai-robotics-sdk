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
from cv_bridge import CvBridge
import numpy as np
import sys
from pathlib import Path

IOU_THRES = 0.40
NUM_CLASSES = 20

class_id = np.arange(NUM_CLASSES)

def calculate_iou(pred_mask, gt_mask, class_id):
    # Create binary maps for the given class
    pred_mask = [[int(i==class_id) for i in x] for x in pred_mask]
    gt_mask   = [[int(i==class_id) for i in x] for x in gt_mask]

    # Calculate overlap (Logical AND)
    overlap = np.logical_and(pred_mask,gt_mask).astype(int)
    # print(np.sum(overlap))

    # Calculate union (Logical OR)
    union   = np.logical_or(pred_mask,gt_mask).astype(int)
    # print(np.sum(union))

    # Calculate IoU
    if np.sum(union) == 0:
        return 0

    iou = np.sum(overlap) / np.sum(union)
    return iou

# create reader instance and open for reading
test_bag = Reader('/root/j7ros_home/data/ros_bag/sample_SS_test')
golden_ref_bag = Reader('/root/j7ros_home/data/ros_bag/sample_SS_golden')

with test_bag as test, golden_ref_bag as golden:
    # topic and msgtype information is available on .connections list
    for connection_test, connection_golden in zip(test.connections, golden.connections):
        print(connection_test.topic, connection_test.msgtype)
        print(connection_golden.topic, connection_golden.msgtype)

    frame_count = 0

    bridge = CvBridge()
    test_topic = [x for x in test.connections if x.topic ==
                  '/vision_cnn/tensor']
    golden_topic = [x for x in golden.connections if x.topic ==
                    '/vision_cnn/golden/tensor']

    for test_messages, golden_messages in zip(test.messages(connections=test_topic), golden.messages(connections=golden_topic)):

        iou = 0
        connection_test, timestamp_test, rawdata_test = test_messages
        connection_golden, timestamp_golden, rawdata_golden = golden_messages

        msg_test = deserialize_cdr(rawdata_test, connection_test.msgtype)
        msg_golden = deserialize_cdr(rawdata_golden, connection_golden.msgtype)

        test_frame = bridge.imgmsg_to_cv2(msg_test)
        golden_frame = bridge.imgmsg_to_cv2(msg_golden)

        for i in range(class_id.size):
            iou += calculate_iou(test_frame,golden_frame,class_id[i])

        avg_frame_iou = iou/class_id.size
        print(avg_frame_iou)

        if (avg_frame_iou >= IOU_THRES):
            print(f"[SS TEST-CASE] passed for Frame {frame_count}")
        else:
            print(f"[SS TEST-CASE] failed for Frame {frame_count}")

        frame_count += 1
