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

usage() {
    echo "usage: prepare_docker_build.sh <DLRT_DIR> <DST_DIR>"
    echo "  <DLRT_DIR> DL runtime lib path"
    echo "  <DST_DIR>  destination path"
    exit 1
}

if [ "$#" -eq 2 ]; then
    DLRT_DIR=$1
    DST_DIR=$2
else
    usage
fi

# Temporary folder to keep the files to be added while building the docker image
rm -rf $DST_DIR
mkdir -p ${DST_DIR}

# Copy files to add
ARCH=`arch`

if [[ "$ARCH" == "aarch64" ]]; then
    SDK_DIR=/opt/robotics_sdk
elif [[ "$ARCH" == "x86_64" ]]; then
    ROS_WS=${HOME}/j7ros_home/ros_ws
    SDK_DIR=$ROS_WS/src/robotics_sdk
else
    echo "Error: $ARCH is not supported"
    exit 1
fi
cp -p ${SDK_DIR}/docker/setup_proxy.sh ${DST_DIR}
cp -p ${SDK_DIR}/docker/ros_setup.sh ${DST_DIR}
cp -p ${SDK_DIR}/docker/entrypoint_*.sh ${DST_DIR}
cp -rp ${SDK_DIR}/docker/proxy ${DST_DIR}

# Copy library files to the temporary folder
if [[ "$ARCH" == "aarch64" ]]; then
    mkdir -p ${DST_DIR}/lib
    Lib_files=(
        # Processor SDK libraries
        /usr/lib/libtivision_apps.so.8.1.0
        /usr/lib/libvx_tidl_rt.so.1.0
        /usr/lib/libion.so
        /usr/lib/libti_rpmsg_char.so.0.3.1
        # DLR lib
        /usr/lib/python3.8/site-packages/dlr/libdlr.so
        # TFLite libs
        ${DLRT_DIR}/libtensorflow-lite.a
        /usr/lib/libtidl_tfl_delegate.so
        # ONNX-RT lib
        ${DLRT_DIR}/libonnxruntime.so.1.7.0
        /usr/lib/libtidl_onnxrt_EP.so.1.0
    )
    for Lib_file in ${Lib_files[@]}; do
        cp $Lib_file ${DST_DIR}/lib
    done

    # Copy header files
    mkdir -p ${DST_DIR}/include
    cp -rp /usr/include/processor_sdk ${DST_DIR}/include
    cp /usr/include/dlr.h ${DST_DIR}/include
fi