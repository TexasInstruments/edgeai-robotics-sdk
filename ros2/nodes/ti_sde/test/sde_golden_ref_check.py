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
from rosbags.typesys import get_types_from_msg, register_types
import numpy as np
import sys
from pathlib import Path

ERROR_THRESHOLD = 0.2
NUM_FRAC_BITS = 4
SDE_DISPARITY_OFFSET = 3

SDE_falseColorLUT_RGB = [
    [128, 64, 0, 24, 33, 28, 24, 21, 18, 14, 10, 6, 3, 1, 2, 2, 2, 3, 2, 3, 2, 2, 2, 2, 3, 3, 3, 2, 2, 2, 3, 3, 2, 3, 1, 3, 3, 2, 2, 3, 2, 3, 3, 2, 2, 3, 2, 2, 3, 3, 3, 3, 2, 2, 4, 2, 3, 3, 2, 3, 3, 2, 2, 3, 3, 3, 2, 2, 3, 2, 2, 3, 1, 3, 2, 3, 2, 3, 3, 3, 2, 2, 2, 2, 3, 2, 3, 2, 3, 3, 3, 3, 2, 2, 2, 3, 2, 3, 2, 4, 2, 1, 3, 2, 2, 2, 3, 3, 3, 2, 2, 2, 1, 8, 13, 20, 26, 31, 38, 44, 50, 56, 63, 67, 74, 81, 86, 93, 99, 104, 110, 117, 123, 129, 136, 140, 147, 155, 159, 166, 172, 177, 183, 191, 196, 202, 209, 214, 219, 225, 231, 238, 244,
        249, 255, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 255, 255, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 255, 255, 254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 255, 254, 255, 255, 255, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 255, 255, 255, 254, 255, 255, 255, 254, 255, 254, 255, 255, 255, 255, 255, 254, 255, 255, 255, 255, 255, 255, 254, 255],
    [128, 64, 0, 4, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 12, 19, 25, 32, 37, 43, 51, 57, 63, 70, 76, 82, 89, 95, 101, 109, 115, 120, 127, 133, 140, 146, 152, 158, 165, 172, 178, 184, 190, 196, 204, 209, 216, 222, 229, 236, 241, 247, 254, 254, 254, 255, 254, 254, 255, 254, 254, 255, 254, 255, 254, 254, 254, 254, 253, 254, 253, 254, 254, 253, 255, 253, 253, 254, 254, 254, 254, 254, 254, 254, 253, 254, 253, 254, 254, 253, 254, 254, 254, 254, 253, 254, 253, 254, 254, 253, 254, 254, 254, 253, 254, 254, 254, 254, 253, 254, 253, 254,
        255, 254, 254, 254, 254, 254, 254, 255, 254, 255, 254, 254, 255, 254, 254, 254, 255, 254, 255, 255, 255, 255, 255, 252, 249, 247, 244, 241, 239, 237, 234, 231, 230, 227, 225, 222, 219, 217, 215, 211, 209, 207, 205, 201, 200, 198, 195, 192, 189, 187, 184, 181, 179, 177, 174, 171, 169, 168, 164, 162, 160, 157, 154, 152, 150, 147, 144, 142, 139, 138, 135, 132, 130, 126, 124, 122, 120, 116, 114, 112, 109, 107, 105, 100, 97, 94, 90, 87, 83, 81, 76, 73, 70, 67, 63, 59, 57, 52, 49, 45, 43, 39, 35, 31, 29, 25, 21, 18, 15, 11, 7, 4, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 255],
    [128, 64, 0, 74, 96, 101, 104, 108, 113, 116, 120, 125, 129, 135, 142, 148, 153, 160, 166, 174, 179, 185, 192, 198, 205, 211, 217, 224, 230, 235, 242, 248, 255, 255, 255, 255, 255, 254, 255, 255, 255, 255, 255, 254, 253, 255, 255, 255, 254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 255, 255, 255, 255, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 249, 242, 236, 231, 224, 217, 210, 205, 199, 192, 186, 179, 173, 169, 162, 155, 149, 144, 138, 130, 123, 117, 112, 105, 99, 91, 87, 80, 73,
        67, 60, 54, 48, 41, 35, 28, 23, 17, 9, 2, 5, 4, 4, 3, 3, 4, 3, 3, 2, 3, 4, 4, 4, 4, 4, 3, 3, 2, 3, 3, 2, 5, 4, 4, 4, 3, 4, 3, 3, 2, 3, 3, 4, 4, 4, 4, 3, 3, 4, 3, 3, 2, 2, 3, 4, 5, 2, 3, 4, 5, 2, 3, 4, 3, 3, 4, 4, 3, 3, 4, 3, 3, 3, 4, 3, 4, 3, 4, 3, 3, 4, 2, 3, 3, 4, 3, 4, 3, 2, 3, 4, 3, 2, 3, 4, 4, 3, 3, 4, 2, 3, 4, 3, 2, 3, 4, 2, 2, 3, 4, 2, 3, 2, 2, 3, 3, 2, 2, 3, 2, 2, 3, 3, 2, 2, 3, 3, 2, 2, 3, 3, 2, 2, 3, 2, 2, 2, 3, 2, 2, 2, 3, 9, 16, 23, 27, 34, 40, 48, 53, 59, 66, 73, 77, 85, 89, 255]
]


class Disparity:
    def __init__(self) -> None:
        self.width = None
        self.height = None
        self.min_disp = None
        self.max_disp = None
        self.step = None
        self.data = None
        self.scale_factor = None

    def initData(self, msg):
        self.width = msg.width
        self.height = msg.height
        self.min_disp = msg.min_disp
        self.max_disp = msg.max_disp
        self.step = msg.step
        self.data = msg.data
        self.scale_factor = float(
            ((1 << NUM_FRAC_BITS) * (self.max_disp - self.min_disp)) / 255)
        self.scale_factor = 1.0 / self.scale_factor


def exractNumpyarray(disparity_data):

    np_array = np.zeros(
        (disparity_data.height, disparity_data.width, 3), dtype="uint8")

    row = []

    for j in range(disparity_data.height):
        for i in range(disparity_data.width):

            pixDisparity = disparity_data.data[i]
            outDisparity = (pixDisparity >> 3) & 0xFFF
            valid = ((pixDisparity & 0x3) > 0)

            value = int(
                (outDisparity * disparity_data.scale_factor * valid) + SDE_DISPARITY_OFFSET)

            R = (SDE_falseColorLUT_RGB[0][value])
            G = (SDE_falseColorLUT_RGB[1][value])
            B = (SDE_falseColorLUT_RGB[2][value])

            row.append([R,G,B])

        np_array = np.append(np_array, row)
        row.clear()

    return np_array


def mse(disp_test_data, disp_golden_data):
    # the 'Mean Squared Error' between the two images is the
    # sum of the squared difference between the two images;
    # NOTE: the two images must have the same dimension
    err = np.sum((disp_test_data.astype("float") -
                 disp_golden_data.astype("float")) ** 2)
    err /= float(disp_test_data.shape[0])

    # return the MSE, the lower the error, the more "similar"
    # the two images are
    return err


disparity_text = Path(
    '/opt/robotics_sdk/ros2/msgs/common_msgs/msg/Disparity.msg').read_text()
add_types = {}
add_types.update(get_types_from_msg(
    disparity_text, 'common_msgs/msg/Disparity'))
register_types(add_types)

# create reader instance and open for reading
test_bag = Reader('/root/j7ros_home/data/ros_bag/sample_SDE_test')
golden_ref_bag = Reader('/root/j7ros_home/data/ros_bag/sample_SDE_golden')

with test_bag as test, golden_ref_bag as golden:
    # topic and msgtype information is available on .connections list
    for connection_test, connection_golden in zip(test.connections, golden.connections):
        print(connection_test.topic, connection_test.msgtype)
        print(connection_golden.topic, connection_golden.msgtype)

    disp_frame_count = 1
    frame_error = sys.maxsize

    test_topic = [x for x in test.connections if x.topic ==
                  '/camera/disparity/raw']
    golden_topic = [x for x in golden.connections if x.topic ==
                    '/camera/disparity/golden/raw']

    for test_messages, golden_messages in zip(test.messages(connections=test_topic), golden.messages(connections=golden_topic)):
        connection_test, timestamp_test, rawdata_test = test_messages
        connection_golden, timestamp_golden, rawdata_golden = golden_messages

        msg_test = deserialize_cdr(rawdata_test, connection_test.msgtype)
        msg_golden = deserialize_cdr(rawdata_golden, connection_golden.msgtype)

        testData = Disparity()
        goldenData = Disparity()

        testData.initData(msg_test)
        goldenData.initData(msg_golden)

        test_frame = exractNumpyarray(testData)
        golden_frame = exractNumpyarray(goldenData)

        frame_error = mse(test_frame, golden_frame)

        if (frame_error < ERROR_THRESHOLD):
            print(f"[SDE TEST-CASE] passed for Frame {disp_frame_count}")
        else:
            f"[SDE TEST-CASE] failed for Frame {disp_frame_count}"

        disp_frame_count += 1
