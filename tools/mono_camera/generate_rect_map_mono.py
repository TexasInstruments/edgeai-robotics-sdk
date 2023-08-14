#!/usr/bin/env python3

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

import sys
import os
import numpy as np
import cv2
import argparse

def save_camera_info(image_size, camera_name, dist_model, K, D, R, P, file_path):
    """ save the camera info to given file_path
    """
    cv_file = cv2.FileStorage(file_path, cv2.FILE_STORAGE_WRITE)
    # work-around. write() has a bug in writing integers (adding .)
    cv_file.write("image_width",             str(image_size[0]))
    cv_file.write("image_height",            str(image_size[1]))
    cv_file.write("camera_name",             camera_name)
    cv_file.write("camera_matrix",           K)
    cv_file.write("distortion_model",        dist_model)
    cv_file.write("distortion_coefficients", D)
    cv_file.write("rectification_matrix",    R)
    cv_file.write("projection_matrix",       P)
    cv_file.release()
    print("camera_info saved into {}".format(file_path))

def save_remap_lut(map_x, map_y, file_path):
    """ save remap LUT to given file_path
    """
    col0 = np.floor((map_x.flatten().reshape(-1,1)*16.)).astype(int)
    col1 = np.floor((map_y.flatten().reshape(-1,1)*16.)).astype(int)
    lut  = np.concatenate((col0, col1), axis=1)
    with open(file_path, "w") as f:
        np.savetxt(f, lut, fmt='%i')
        print("remap LUT saved into {}".format(file_path))

def save_LDC_lut(map_x, map_y, file_path):
    """ save LDC LUT to given file_path
    """
    LDC_DS_FACTOR = 4

    width   = map_x.shape[1]
    height  = map_x.shape[0]
    sWidth  = int(width / LDC_DS_FACTOR + 1)
    sHeight = int(height / LDC_DS_FACTOR + 1)
    lineOffset = ((sWidth + 15) & (~15))

    ldcLUT_x  = np.zeros((sHeight,lineOffset), dtype=np.int16)
    ldcLUT_y  = np.zeros((sHeight,lineOffset), dtype=np.int16)

    for y in range(0, sHeight):
        j = y * LDC_DS_FACTOR
        if j > height - 1:
            j = height - 1

        for x in range(0, sWidth):
            i = x * LDC_DS_FACTOR
            if i > width - 1:
                i = width - 1

            dx = np.floor(map_x[j, i]*8. + 0.5).astype(int) - i*8
            dy = np.floor(map_y[j, i]*8. + 0.5).astype(int) - j*8

            ldcLUT_x[y, x] = dx & 0xFFFF
            ldcLUT_y[y, x] = dy & 0xFFFF

        remain = ((sWidth + 15) & (~15)) - sWidth
        while (remain > 0):
            ldcLUT_x[y, sWidth - 1 + remain] = 0
            ldcLUT_y[y, sWidth - 1 + remain] = 0
            remain = remain - 1

    #  y offset comes first
    col0   = np.floor(ldcLUT_y.flatten().reshape(-1,1)).astype(np.int16)
    col1   = np.floor(ldcLUT_x.flatten().reshape(-1,1)).astype(np.int16)
    ldcLUT = np.concatenate((col0, col1), axis=1)
    ldcLUT.tofile(file_path)
    print("LDC LUT saved into {}".format(file_path))

def save_camera_info(image_size, camera_name, dist_model, K, D, R, P, file_path):
    """ save the camera info to given file_path
    """
    cv_file = cv2.FileStorage(file_path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("image_width",             image_size[0])
    cv_file.write("image_height",            image_size[1])
    cv_file.write("camera_name",             camera_name)
    cv_file.write("camera_matrix",           K)
    cv_file.write("distortion_model",        dist_model)
    cv_file.write("distortion_coefficients", D)
    cv_file.write("rectification_matrix",    R)
    cv_file.write("projection_matrix",       P)
    cv_file.release()
    print("camera_info saved into {}".format(file_path))

def main(camera_info_file, camera_mode, camera_name, new_image_size=None):
    """ Parse camera_info YAML file, and generate undistortion and
        rectification look-up-table (aks remap LUT).

        Following two distortion models are supported:
        1. `plumb_bob`
        2. `equidistant` for fisheye cameras
            In this model, this tool also generates camera_info for the rectified images
    """

    if not os.path.exists(camera_info_file):
        sys.exit('Error: {} does not exist'.format(camera_info_file))

    # parse camera_info_file
    cv_file = cv2.FileStorage(camera_info_file, cv2.FILE_STORAGE_READ)
    width       = int(cv_file.getNode("image_width").real())
    height      = int(cv_file.getNode("image_height").real())
    # camera_name = cv_file.getNode("camera_name").string()
    K           = cv_file.getNode("camera_matrix").mat()
    dist_model  = cv_file.getNode("distortion_model").string()
    D           = cv_file.getNode("distortion_coefficients").mat()
    R           = cv_file.getNode("rectification_matrix").mat()
    P           = cv_file.getNode("projection_matrix").mat()
    print("camera_info parsed from {}".format(camera_info_file))
    cv_file.release()

    image_size = (width, height)
    if dist_model == 'plumb_bob':
        # create remap table for left
        map_x, map_y = cv2.initUndistortRectifyMap(K, D, R, P, size=image_size, m1type=cv2.CV_32FC1)

        # write LDC undist/rect LUT
        lut_file_path = os.path.join(os.path.dirname(camera_info_file), camera_name+"_"+camera_mode+"_LUT.bin")
        save_LDC_lut(map_x, map_y, lut_file_path)

    elif dist_model == 'equidistant':
        # fisheye distortion model: we also generate camera_info for the rectified images
        if args.new_height == None:
            new_image_size = (width, height)
        alpha = 1.0
        fov_scale = np.float64(1.0)
        # opencv-python<4.3 required (specified in requirements.txt).
        # default version 4.5.4 in Ubuntu 22.04 is not compatible and
        # fisheye.estimateNewCameraMatrixForUndistortRectify() outputs
        # a wrong camera matrix for the example IMX390 camera_info.
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, image_size, R=np.eye(3), balance=alpha, new_size=new_image_size, fov_scale=fov_scale)
        map_x, map_y = cv2.fisheye.initUndistortRectifyMap(K, D, R=np.eye(3), P=new_K, size=new_image_size, m1type=cv2.CV_32FC1)

        # write LDC undist/rect LUT
        lut_file_name = "%s_%s_%ix%i_LUT.bin" % (camera_name, dist_model, new_image_size[0], new_image_size[1])
        lut_file_path = os.path.join(os.path.dirname(camera_info_file), lut_file_name)
        save_LDC_lut(map_x, map_y, lut_file_path)

        # save camera_info for the rectified images
        new_D = np.zeros((1,4))
        rect_matrix = np.eye(3)
        proj_matrix = np.hstack( (new_K, np.zeros((3,1))) )
        out_yaml_file_path = os.path.join(os.path.dirname(camera_info_file), lut_file_path.replace('_LUT.bin', '_rect.yaml'))
        save_camera_info(new_image_size, camera_name, dist_model, new_K,
                    new_D, rect_matrix, proj_matrix, out_yaml_file_path)

    else:
        sys.exit('Error: {} is not supported'.format(dist_model))


if __name__ == "__main__":
    """
    Example for C920 webcam:
      ./generate_rect_map_mono.py -i c920_camera_info.yaml -n c920
    Example for IMX390 fisheye camera:
      ./generate_rect_map_mono.py -i imx390_35244_equidistant_camera_info.yaml -n imx390_35244 --width 1920 --height 1080
    """
    # input argument parser
    parser = argparse.ArgumentParser(description='LCD LUT generation tool for mono camera')
    parser.add_argument('--input', '-i', type=str, default='',
        help='YAML file name for camera_info.')
    parser.add_argument('--name', '-n', type=str, default='C920',
        help='Camera name')
    parser.add_argument('--width', dest='new_width',
        help='LDC output image width (only for IMX390)', type=int)
    parser.add_argument('--height', dest='new_height',
        help='LDC output image height (only for IMX390)', type=int)
    args = parser.parse_args()

    # main routine
    camera_info_file = args.input
    camera_name = args.name
    camera_mode = 'HD'
    new_image_size = (args.new_width, args.new_height)
    main(camera_info_file, camera_mode, camera_name, new_image_size)
