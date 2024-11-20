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

echo "$(basename "$0"): Processing..."

# Example BASE_URL_RT="https://github.com/TexasInstruments-Sandbox/edgeai-osrt-libs-build/releases/download/rel.10.01.00.01-ubuntu22.04"
if [ -z "$BASE_URL_RT" ]; then
    echo "Error: BASE_URL_RT is not defined."
    exit 1
else
    echo "BASE_URL_RT=$BASE_URL_RT"
fi

if [ -z "$SDK_VER_STR" ]; then
    echo "Error: BASE_URL_RT is not defined."
    exit 1
else
    echo "SDK_VER_STR=$SDK_VER_STR"
fi

# package names
ONNX_RT_WHL=onnxruntime_tidl-1.14.0+${SDK_VER_STR}-cp310-cp310-linux_aarch64.whl
ONNX_PKG=onnx-1.14.0+${SDK_VER_STR}-ubuntu22.04_aarch64
TFLITE_RT_WHL=tflite_runtime-2.12.0-cp310-cp310-linux_aarch64.whl
TFLITE_PKG=tflite-2.12-ubuntu22.04_aarch64
DLR_WHL=dlr-1.13.0-py3-none-any.whl

# python dist-packages path
PYTHON_DIST=/usr/local/lib/python3.10/dist-packages

# list of files to download
FILES=(
    "$DLR_WHL"
    "$ONNX_RT_WHL"
    "$TFLITE_RT_WHL"
    "$TFLITE_PKG.tar.gz"
    "$ONNX_PKG.tar.gz"
)

# source folder for lib files
LIB_DIR="$HOME/ubuntu22-deps"
DOWNLOAD_LIBS=true
if [ -d "$LIB_DIR" ]; then
    DOWNLOAD_LIBS=false
    echo "$(basename "$0"): to use the library files under $LIB_DIR"
fi

# function to download the lib files
download_files() {
    if [ "$DOWNLOAD_LIBS" = true ]; then
        mkdir -p "$LIB_DIR"
        for file in "${FILES[@]}"; do
            wget -q --no-proxy "$BASE_URL_RT/$file" -O "$LIB_DIR/$file"
            if [ $? -ne 0 ]; then
                echo "Error: Failed to download $file"
                exit 1
            fi
        done
    fi
}

pip_install_whl()
{
    local whl_file=$1
    local options=$2
    if [ -f "$whl_file" ]; then
        pip3 install $whl_file -t $PYTHON_DIST $options
    else
        echo "Error: $whl_file does not exist"
        exit 1
    fi
}

extract_pkg() {
    local pkg=$1
    if [ -f "$pkg" ]; then
        tar xzf "$pkg"
    else
        echo "$pkg does not exist"
        exit 1
    fi
}

# install libs
install_libs() {
    cd $LIB_DIR
    if [ ! -d /usr/include/tensorflow ]; then
        extract_pkg $TFLITE_PKG.tar.gz
        mv "$TFLITE_PKG/tensorflow" /usr/include
        mv "$TFLITE_PKG/tflite_2.12" /usr/lib/
        cp "$TFLITE_PKG/libtensorflow-lite.a" /usr/lib/
    else
        echo "skipping tensorflow setup: found /usr/include/tensorflow"
    fi

    if [ ! -d /usr/include/onnxruntime ]; then
        extract_pkg $ONNX_PKG.tar.gz
        cp -r $ONNX_PKG/libonnxruntime.so* /usr/lib/
        ln -s "/usr/lib/libonnxruntime.so.1.14.0+${SDK_VER_STR}" /usr/lib/libonnxruntime.so
        mv "$ONNX_PKG/onnxruntime" /usr/include/
    else
        echo "skipping onnxruntime setup: found /usr/include/onnxruntime"
    fi

    if [ ! -f /usr/include/itidl_rt.h ]; then
        git clone -b master git://git.ti.com/processor-sdk-vision/arm-tidl.git arm-tidl-repo
        cp arm-tidl-repo/rt/inc/itidl_rt.h /usr/include/
        cp arm-tidl-repo/rt/inc/itvm_rt.h /usr/include/
        rm -rf arm-tidl-repo
    else
        echo "skipping itidl_rt.h setup: found /usr/include/itidl_rt.h"
    fi

    if [ ! -f /usr/dlr/libdlr.so ]; then
        mkdir /usr/dlr/
        ln -sf "$PYTHON_DIST/dlr/libdlr.so" /usr/dlr/libdlr.so
    fi

    if [ ! -f /usr/lib/libdlr.so ]; then
        ln -sf /usr/dlr/libdlr.so /usr/lib/libdlr.so
    fi

    if [ ! -d /usr/include/neo-ai-dlr ];then
        mkdir /usr/include/neo-ai-dlr
        ln -sf "$PYTHON_DIST/dlr/include" /usr/include/neo-ai-dlr/include
    fi

    cd -
}

# clean up
clean_up() {
    if [ "$DOWNLOAD_LIBS" = true ]; then
        rm -rf "${LIB_DIR}"
    fi
}

# download
download_files

# install python packages
pip_install_whl "$LIB_DIR/$DLR_WHL" "--upgrade --force-reinstall --disable-pip-version-check"
pip_install_whl "$LIB_DIR/$ONNX_RT_WHL" "--disable-pip-version-check"
pip_install_whl "$LIB_DIR/$TFLITE_RT_WHL" "--upgrade --force-reinstall --disable-pip-version-check"
pip3 install numpy -t $PYTHON_DIST --upgrade --force-reinstall --no-cache-dir --disable-pip-version-check

# install libs
install_libs

# clean up
clean_up

echo "$(basename "$0"): Completed!"
