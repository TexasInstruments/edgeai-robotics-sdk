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

usage() {
    echo "usage: prepare_docker_build.sh <DST_DIR>"
    echo "  <DST_DIR>  destination path"
    exit 1
}

if [ "$#" -eq 1 ]; then
    DST_DIR=$1
else
    usage
fi

if [ -z "$ARCH" ]; then
    echo "Error: ARCH is not defined."
    exit 1
fi

if [ -z "$SDK_DIR" ]; then
    echo "Error: SDK_DIR is not defined."
    exit 1
fi

# Temporary folder to keep the files to be added while building the docker image
rm -rf $DST_DIR
mkdir -p ${DST_DIR}/proxy

# Copy files to add

# function to check and copy files
cp_fnc() {
    if [ -f "$1" ]; then
        cp "$1" "$2"
    else
        echo "File $1 does not exist."
        exit 1
    fi
}

cp_fnc ${SDK_DIR}/docker/setup_proxy.sh ${DST_DIR}
cp_fnc ${SDK_DIR}/docker/ros_setup.sh ${DST_DIR}
cp_fnc ${SDK_DIR}/docker/set_aliases.sh ${DST_DIR}
cp_fnc ${SDK_DIR}/docker/entrypoint_arm64.sh ${DST_DIR}
cp_fnc ${SDK_DIR}/docker/entrypoint_viz.sh ${DST_DIR}
cp_fnc ${SDK_DIR}/tools/mono_camera/requirements.txt ${DST_DIR}
if [[ "$ARCH" == "arm64" ]]; then
    cp_fnc ${SDK_DIR}/docker/install_gst_v4l2_lib.sh ${DST_DIR}
    cp_fnc ${SDK_DIR}/docker/install_vision_apps_lib.sh ${DST_DIR}
    cp_fnc ${SDK_DIR}/docker/install_osrt_libs.sh ${DST_DIR}
    cp_fnc ${SDK_DIR}/docker/install_tidl_libs.sh ${DST_DIR}
fi

# check if PROXY_DIR is already set, if not, set it based on conditions
if [[ -z "$PROXY_DIR" ]]; then
    if [[ "$ARCH" == "arm64" && "$(whoami)" == "root" ]]; then
        PROXY_DIR="/opt/proxy"
    else
        PROXY_DIR="$HOME/proxy"
    fi
fi

if [ -d "$PROXY_DIR" ]; then
    cp -rp $PROXY_DIR/* ${DST_DIR}/proxy
fi

if [[ "$ARCH" == "arm64" ]]; then
    if [[ -z "$SOC" ]]; then
        echo "SOC is not defined. Sourcing detect_soc.sh."
        source ${SDK_DIR}/docker/scripts/detect_soc.sh
    else
        echo "SOC=$SOC already defined."
    fi
fi

# for testing using a local libs folder
if [[ "$ARCH" == "arm64" ]]; then
    if [ -f "$HOME/ubuntu22-deps.tar.gz" ]; then
        cp "$HOME/ubuntu22-deps.tar.gz" ${DST_DIR}
    fi
fi