#!/bin/bash

#  Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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

echo "$(basename "$0"): Processing..."

# Example BASE_URL="https://software-dl.ti.com/jacinto7/esd/robotics-sdk/10_00_00/deps"
if [ -z "$BASE_URL" ]; then
    echo "Error: BASE_URL is not defined."
    exit 1
else
    echo "BASE_URL=$BASE_URL"
fi

if [ -z "$SOC" ]; then
    echo "SOC=$SOC: SOC should be defined."
    exit 1
else
    echo "SOC=$SOC"
fi

# TIDL runtime modules
LIB_TIDL_RT=libvx_tidl_rt.so
LIB_ONNX_RT_EP=libtidl_onnxrt_EP.so
LIB_TFLITE_RT_DELEGATE=libtidl_tfl_delegate.so
TIDL_SO_VER=1.0

# list of files to download
FILES=(
    "arm-tidl/${SOC}/${LIB_TIDL_RT}.${TIDL_SO_VER}"
    "arm-tidl/${SOC}/${LIB_ONNX_RT_EP}.${TIDL_SO_VER}"
    "arm-tidl/${SOC}/${LIB_TFLITE_RT_DELEGATE}.${TIDL_SO_VER}"
)

LINK_FILES=(
    "${LIB_TIDL_RT}"
    "${LIB_ONNX_RT_EP}"
    "${LIB_TFLITE_RT_DELEGATE}"
)

# source folder for lib files
LIB_DIR="$HOME/ubuntu22-deps"
DOWNLOAD_LIBS=true
if [ -d "$LIB_DIR" ]; then
    DOWNLOAD_LIBS=false
    echo "$(basename "$0"): to use the library files under $LIB_DIR"
fi

# destination folder for lib files
LIB_DST="/usr/lib"

# function to download the lib files
download_files() {
    if [ "$DOWNLOAD_LIBS" = true ]; then
        mkdir -p "$LIB_DIR"
        for file in "${FILES[@]}"; do
            mkdir -p "$LIB_DIR/$(dirname "$file")"
            wget -q --no-proxy "$BASE_URL/$file" -O "$LIB_DIR/$file"
            if [ $? -ne 0 ]; then
                echo "Error: Failed to download $file"
                exit 1
            fi
        done
    fi
}

# function to copy the lib files and establish soft links
copy_and_link_files() {
    for i in "${!FILES[@]}"; do
        file="${FILES[$i]}"
        link_file="${LINK_FILES[$i]}"

        # copy the file to /usr/lib
        cp "$LIB_DIR/$file" "${LIB_DST}/$(basename "$file")"

        # create the symbolic link
        ln -snf "${LIB_DST}/$(basename "$file")" "${LIB_DST}/${link_file}"
    done
}

# clean up
clean_up() {
    if [ "$DOWNLOAD_LIBS" = true ]; then
        rm -rf "${LIB_DIR}"
    fi
}

download_files
copy_and_link_files
clean_up

echo "$(basename "$0"): Completed!"
