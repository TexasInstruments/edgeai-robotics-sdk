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
    echo "usage: prepare_docker_build.sh <DST_DIR> <TIVA_LIB_VER> <RPMSG_LIB_VER>"
    echo "  <DST_DIR>  destination path"
    exit 1
}

if [ "$#" -eq 3 ]; then
    DST_DIR=$1
    TIVA_LIB_VER=$2
    RPMSG_LIB_VER=$3
else
    usage
fi

# Temporary folder to keep the files to be added while building the docker image
rm -rf $DST_DIR
mkdir -p ${DST_DIR}/proxy

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
cp -p ${SDK_DIR}/docker/set_aliases.sh ${DST_DIR}
cp -p ${SDK_DIR}/docker/entrypoint_*.sh ${DST_DIR}
cp -p ${SDK_DIR}/tools/mono_camera/requirements.txt ${DST_DIR}

if [[ "$ARCH" == "aarch64" ]]; then
    if [ -d "/opt/proxy" ]; then
        cp -rp /opt/proxy/* ${DST_DIR}/proxy
    fi
fi

if [[ "$ARCH" == "x86_64" ]]; then
    if [ -d "$HOME/proxy" ]; then
        cp -rp $HOME/proxy/* ${DST_DIR}/proxy
    fi
fi

if [[ "$ARCH" == "aarch64" ]]; then
    source ${SDK_DIR}/docker/scripts/detect_soc.sh
    # Installation of OSRT libs is now part of edgeai-gst-app/scripts/install_dl_inferer.sh
    cp -p /opt/edgeai-gst-apps/scripts/install_apps_utils.sh ${DST_DIR}
    cp -p /opt/edgeai-gst-apps/scripts/install_tiovx_modules.sh ${DST_DIR}
    cp -p /opt/edgeai-gst-apps/scripts/install_tiovx_kernels.sh ${DST_DIR}
    cp -p /opt/edgeai-gst-apps/scripts/install_gst_plugins.sh ${DST_DIR}
    cp -p /opt/edgeai-gst-apps/scripts/install_ti_gpio_libs.sh ${DST_DIR}
fi

# Copy library files to the temporary folder
if [[ "$ARCH" == "aarch64" ]]; then
    mkdir -p ${DST_DIR}/lib
    Lib_files=(
        # Processor SDK libraries
        /usr/lib/libtivision_apps.so.${TIVA_LIB_VER}
        /usr/lib/libti_rpmsg_char.so.${RPMSG_LIB_VER}
        /usr/lib/libtidl_tfl_delegate.so
        /usr/lib/libtidl_onnxrt_EP.so.1.0
        /usr/lib/libvx_tidl_rt.so.1.0
    )
    for Lib_file in ${Lib_files[@]}; do
        cp $Lib_file ${DST_DIR}/lib
    done

    # Copy a GST lib that was updated from PSDK
    mkdir -p ${DST_DIR}/lib_gstreamer-1.0
    cp /usr/lib/gstreamer-1.0/libgstvideo4linux2.so ${DST_DIR}/lib_gstreamer-1.0

    # Copy header files
    mkdir -p ${DST_DIR}/include
    cp -rp /usr/include/processor_sdk ${DST_DIR}/include
fi
