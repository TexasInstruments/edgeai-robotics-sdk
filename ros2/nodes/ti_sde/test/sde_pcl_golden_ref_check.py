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
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import numpy.lib.recfunctions as recfuncs
import open3d as o3d
import sys

from pathlib import Path

DUMMY_FIELD_PREFIX = '__'
AVG_ERROR_THRES = 2.0

type_mappings = [(PointField.INT8, np.dtype('int8')),
                 (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')),
                 (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')),
                 (PointField.FLOAT64, np.dtype('float64'))]

pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}


def pointcloud2_to_dtype(cloud_msg):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in cloud_msg.fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(
                ('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1
        np_dtype_list.append((f.name, pftype_to_nptype[f.datatype]))
        offset += pftype_sizes[f.datatype]

    # might be extra padding between points
    while offset < cloud_msg.point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def split_rgb_field(cloud_arr):
    '''Takes an array with a named 'rgb' float32 field, and returns an array in which
    this has been split into 3 uint 8 fields: 'r', 'g', and 'b'.

    (pcl stores rgb in packed 32 bit floats)
    '''
    rgb_arr = cloud_arr['rgb'].copy()
    rgb_arr.dtype = np.uint32
    r = np.asarray((rgb_arr >> 16) & 255, dtype=np.uint8)
    g = np.asarray((rgb_arr >> 8) & 255, dtype=np.uint8)
    b = np.asarray(rgb_arr & 255, dtype=np.uint8)

    # create a new array, without rgb, but with r, g, and b fields
    new_dtype = []
    for field_name in cloud_arr.dtype.names:
        field_type, field_offset = cloud_arr.dtype.fields[field_name]
        if not field_name == 'rgb':
            new_dtype.append((field_name, field_type))
    new_dtype.append(('r', np.uint8))
    new_dtype.append(('g', np.uint8))
    new_dtype.append(('b', np.uint8))
    new_cloud_arr = np.zeros(cloud_arr.shape, new_dtype)

    # fill in the new array
    for field_name in new_cloud_arr.dtype.names:
        if field_name == 'r':
            new_cloud_arr[field_name] = r
        elif field_name == 'g':
            new_cloud_arr[field_name] = g
        elif field_name == 'b':
            new_cloud_arr[field_name] = b
        else:
            new_cloud_arr[field_name] = cloud_arr[field_name]
    return new_cloud_arr


def pointcloud2_to_array(cloud_msg, split_rgb=False, remove_padding=True):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray

    Reshapes the returned array to have shape (height, width), even if the height is 1.

    The reason for using np.fromstring rather than struct.unpack is speed... especially
    for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = pointcloud2_to_dtype(cloud_msg)

    # parse the cloud into an array
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    if remove_padding:
        cloud_arr = recfuncs.repack_fields(cloud_arr[
            [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]])

    if split_rgb:
        cloud_arr = split_rgb_field(cloud_arr)

    return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))


def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float64):
    '''Pulls out x, y, and z columns from the cloud recordarray, and returns
        a 3xN matrix.
    '''
    # remove crap points
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(
            cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]

    # pull out x, y, and z values
    points = np.zeros(list(cloud_array.shape) + [3], dtype=dtype)
    points[..., 0] = cloud_array['x']
    points[..., 1] = cloud_array['y']
    points[..., 2] = cloud_array['z']

    return points


def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True):
    return get_xyz_points(pointcloud2_to_array(cloud_msg))


# create reader instance and open for reading
test_bag = Reader('/root/j7ros_home/data/ros_bag/sample_SDE_PCL_test')
golden_ref_bag = Reader('/root/j7ros_home/data/ros_bag/sample_SDE_PCL_golden')

with test_bag as test, golden_ref_bag as golden:
    # topic and msgtype information is available on .connections list
    for connection_test, connection_golden in zip(test.connections, golden.connections):
        print(connection_test.topic, connection_test.msgtype)
        print(connection_golden.topic, connection_golden.msgtype)

    pcd_count = 1

    test_topic = [x for x in test.connections if x.topic == '/point_cloud']
    golden_topic = [x for x in golden.connections if x.topic ==
                    '/point_cloud/golden']

    for test_messages, golden_messages in zip(test.messages(connections=test_topic), golden.messages(connections=golden_topic)):
        connection_test, timestamp_test, rawdata_test = test_messages
        connection_golden, timestamp_golden, rawdata_golden = golden_messages

        msg_test = deserialize_cdr(rawdata_test, connection_test.msgtype)
        msg_golden = deserialize_cdr(rawdata_golden, connection_golden.msgtype)

        pcl_test_data = pointcloud2_to_xyz_array(msg_test)
        pcl_golden_data = pointcloud2_to_xyz_array(msg_golden)

        pcd_test = o3d.geometry.PointCloud()
        pcd_golden = o3d.geometry.PointCloud()

        pcd_test.points = o3d.utility.Vector3dVector(pcl_test_data)
        pcd_golden.points = o3d.utility.Vector3dVector(pcl_golden_data)

        dist = pcd_golden.compute_point_cloud_distance(pcd_test)
        dist = np.asarray(dist)

        avg_error = np.average(dist)

        if avg_error < AVG_ERROR_THRES:
            print(f"[SDE PCL TEST-CASE] passed for Point Cloud {pcd_count}")
        else:
            print(f"[SDE PCL TEST-CASE] passed for Point Cloud {pcd_count}")
        pcd_count += 1
