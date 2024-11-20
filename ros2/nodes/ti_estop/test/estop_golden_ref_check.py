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

import os
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rosbags.typesys import get_types_from_msg, register_types
from pathlib import Path


class Box:

    def __init__(self) -> None:
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0
        self.x3 = 0
        self.y3 = 0
        self.x4 = 0
        self.y4 = 0

    def extractFrontBoxPoints(self, msg, itr):
        self.x1 = msg.obj_pos3d[itr].pf1x
        self.y1 = msg.obj_pos3d[itr].pf1y

        self.x2 = msg.obj_pos3d[itr].pf2x
        self.y2 = msg.obj_pos3d[itr].pf2y

        self.x3 = msg.obj_pos3d[itr].pf3x
        self.y3 = msg.obj_pos3d[itr].pf3y

        self.x4 = msg.obj_pos3d[itr].pf4x
        self.y4 = msg.obj_pos3d[itr].pf4y

    def extractBackBoxPoints(self, msg, itr):
        self.x1 = msg.obj_pos3d[itr].pr1x
        self.y1 = msg.obj_pos3d[itr].pr1y

        self.x2 = msg.obj_pos3d[itr].pr2x
        self.y2 = msg.obj_pos3d[itr].pr2y

        self.x3 = msg.obj_pos3d[itr].pr3x
        self.y3 = msg.obj_pos3d[itr].pr3y

        self.x4 = msg.obj_pos3d[itr].pr4x
        self.y4 = msg.obj_pos3d[itr].pr4y


# create reader instance and open for reading
test_bag = Reader('/root/j7ros_home/data/ros_bag/sample_ESTOP_test')
golden_ref_bag = Reader('/root/j7ros_home/data/ros_bag/sample_ESTOP_golden')
detection2D_text = Path(
    '/opt/robotics_sdk/ros2/msgs/common_msgs/msg/Detection3D.msg').read_text()
ObjectPos3D_text = Path(
    '/opt/robotics_sdk/ros2/msgs/common_msgs/msg/ObjectPos3D.msg').read_text()
add_types = {}
add_types.update(get_types_from_msg(
    detection2D_text, 'common_msgs/msg/Detection3D'))
add_types.update(get_types_from_msg(
    ObjectPos3D_text, 'common_msgs/msg/ObjectPos3D'))
register_types(add_types)

TEST_PASS_THRES = 0.45
IOU_THRES = 0.55

def IOU(ref_data, inf_data):
    """
    Calculate the Intersection over Union (IoU) of two bounding boxes.

    Parameters
    ----------
    ref_data : list
        Values: [xmin, ymin, xmax, ymax]
        The (xmin, ymin) position is at the top left corner,
        the (xmax, ymax) position is at the bottom right corner
    inf_data : list
        Values: [xmin, ymin, xmax, ymax}
        The (xmin, ymin) position is at the top left corner,
        the (xmax, ymax) position is at the bottom right corner

    Returns
    -------
    float
        in [0, 1]
    """
    # print(ref_data,inf_data)
    assert ref_data[0] < ref_data[2]
    assert ref_data[1] > ref_data[3]
    assert inf_data[0] < inf_data[2]
    assert inf_data[1] > inf_data[3]

    # determine the coordinates of the intersection rectangle
    x_left = max(ref_data[0], inf_data[0])
    y_top = min(ref_data[1], inf_data[1])
    x_right = min(ref_data[2], inf_data[2])
    y_bottom = max(ref_data[3], inf_data[3])

    if x_right < x_left or y_bottom > y_top:
        return 0.0

    # Area of the intersection rectangle
    intersection_area = (x_right - x_left) * (y_bottom - y_top)
    # computing the area of both reference frame and inference frame
    ref_frame_area = (ref_data[2] - ref_data[0]) * (ref_data[3] - ref_data[1])
    inf_frame_area = (inf_data[2] - inf_data[0]) * (inf_data[3] - inf_data[1])

    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = intersection_area / \
        float(ref_frame_area + inf_frame_area - intersection_area)

    assert iou >= 0.0
    assert iou <= 1.0
    return iou


with test_bag as test, golden_ref_bag as golden:
    # topic and msgtype information is available on .connections list
    for connection_test, connection_golden in zip(test.connections, golden.connections):
        print(connection_test.topic, connection_test.msgtype)
        print(connection_golden.topic, connection_golden.msgtype)

    IOU_val = None
    case_pass = False
    frame_no = 1

    for test_messages, golden_messages in zip(test.messages(), golden.messages()):
        connection_test, timestamp_test, rawdata_test = test_messages
        connection_golden, timestamp_golden, rawdata_golden = golden_messages
        msg_test = deserialize_cdr(rawdata_test, connection_test.msgtype)
        msg_golden = deserialize_cdr(rawdata_golden, connection_golden.msgtype)
        no_object_pass = 0

        num_objects_test = msg_test.num_objects

        if num_objects_test == 0:
            continue
        else:
            for i in range(num_objects_test):

                case_pass = False
                test_front_box = Box()
                test_back_box = Box()

                test_front_box.extractFrontBoxPoints(msg_test, i)
                test_back_box.extractBackBoxPoints(msg_test, i)

                test_front_box_points = [
                    test_front_box.x1, test_front_box.y1, test_front_box.x3, test_front_box.y3]
                test_back_box_points = [
                    test_back_box.x1, test_back_box.y1, test_back_box.x3, test_back_box.y3]

                # for golden_messages in golden.messages():

                num_objects_golden = msg_golden.num_objects

                for i in range(num_objects_golden):

                    golden_front_box = Box()
                    golden_back_box = Box()

                    golden_front_box.extractFrontBoxPoints(msg_golden, i)
                    golden_back_box.extractBackBoxPoints(msg_golden, i)

                    golden_front_box_points = [
                        golden_front_box.x1, golden_front_box.y1, golden_front_box.x3, golden_front_box.y3]
                    golden_back_box_points = [
                        golden_back_box.x1, golden_back_box.y1, golden_back_box.x3, golden_back_box.y3]

                    IOU_front_box_val = IOU(
                        golden_front_box_points, test_front_box_points)
                    IOU_back_box_val = IOU(
                        golden_back_box_points, test_back_box_points)

                    if ((IOU_front_box_val >= IOU_THRES) and (IOU_back_box_val >= IOU_THRES)):
                        case_pass = True
                        break

                # Counting the no of objects that are passing
                if (case_pass):
                    no_object_pass += 1

        # Deciding Pass/Fail based on no of objects that passed and the total no of objects that are detected
        if ((no_object_pass)/num_objects_test < TEST_PASS_THRES):
            print(f"[ESTOP TEST-CASE] failed for Frame {frame_no}")
        else:
            print(f"[ESTOP TEST-CASE] passed for Frame {frame_no}")

        frame_no += 1
