#!/bin/bash
# Cut-down version of:
# https://github.com/TexasInstruments/edgeai-tidl-tools/blob/master/dockers/dependency_build/qemu/targetfs_load.sh
# Changes:
#   TARGET_FS_PATH=/
#   wget with -q option
#   cleaned out lines that are not effective
#   Use PYTHON_DIST (not PYTHONPATH)
#   PYTHON_DIST=/usr/local/lib/python3.10/dist-packages (Ubuntu 22.04 container)
#   Not installing opencv_4.2.0_aragoj7.tar.gz

REL=09_00_00_01
SCRIPTDIR=`pwd`
echo "installing dependedcies at $TARGET_FS_PATH"
cd $HOME

if [ ! -d arago_j7_pywhl ];then
    mkdir arago_j7_pywhl
fi
cd $HOME/arago_j7_pywhl

wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/$REL/OSRT_TOOLS/ARM_LINUX/ARAGO/dlr-1.13.0-py3-none-any.whl
wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/$REL/OSRT_TOOLS/ARM_LINUX/ARAGO/onnxruntime_tidl-1.7.0-cp310-cp310-linux_aarch64.whl
wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/$REL/OSRT_TOOLS/ARM_LINUX/ARAGO/tflite_runtime-2.8.2-cp310-cp310-linux_aarch64.whl

# ln -s /usr/bin/pip3 /usr/bin/pip3.10
pip3 install --upgrade --force-reinstall dlr-1.13.0-py3-none-any.whl
pip3 install onnxruntime_tidl-1.7.0-cp310-cp310-linux_aarch64.whl
pip3 install --upgrade --force-reinstall tflite_runtime-2.8.2-cp310-cp310-linux_aarch64.whl
pip3 install --upgrade --force-reinstall --no-cache-dir numpy
cd $HOME
rm -r $HOME/arago_j7_pywhl

if [  ! -d /usr/include/tensorflow ]; then
    wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/$REL/OSRT_TOOLS/ARM_LINUX/ARAGO/tflite_2.8_aragoj7.tar.gz
    tar xf tflite_2.8_aragoj7.tar.gz
    rm tflite_2.8_aragoj7.tar.gz
    mv tflite_2.8_aragoj7/tensorflow  /usr/include
    mv tflite_2.8_aragoj7/tflite_2.8  /usr/lib/
    cp tflite_2.8_aragoj7/libtensorflow-lite.a /usr/lib/
    rm -r tflite_2.8_aragoj7
    cd $HOME
else
    echo "skipping tensorflow setup: found /usr/include/tensorflow"
    echo "To redo the setup delete: /usr/include/tensorflow and run this script again"
fi

# if [  ! -d /usr/include/opencv-4.2.0 ]; then
#     wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/$REL/OSRT_TOOLS/ARM_LINUX/ARAGO/opencv_4.2.0_aragoj7.tar.gz
#     tar -xf opencv_4.2.0_aragoj7.tar.gz
#     rm opencv_4.2.0_aragoj7.tar.gz
#     cp -r opencv_4.2.0_aragoj7/opencv /usr/lib/
#     mv opencv_4.2.0_aragoj7/opencv-4.2.0 /usr/include/
#     cd $HOME
#     rm -r opencv_4.2.0_aragoj7
# else
#     echo "skipping opencv-4.2.0 setup: found /usr/include/opencv-4.2.0"
#     echo "To redo the setup delete: /usr/include/opencv-4.2.0 and run this script again"
# fi

if [  ! -d /usr/include/onnxruntime ]; then
    wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/$REL/OSRT_TOOLS/ARM_LINUX/ARAGO/onnx_1.7.0_aragoj7.tar.gz
    tar xf onnx_1.7.0_aragoj7.tar.gz
    rm onnx_1.7.0_aragoj7.tar.gz
    cp -r  onnx_1.7.0_aragoj7/libonnxruntime.so*   /usr/lib/
    cd   /usr/lib/
    ln -s libonnxruntime.so.1.7.0 libonnxruntime.so
    cd  $HOME
    mv onnx_1.7.0_aragoj7/onnxruntime /usr/include/
    rm -r onnx_1.7.0_aragoj7
    cd  $HOME
else
    echo "skipping onnxruntime setup: found /usr/include/onnxruntime"
    echo "To redo the setup delete: /usr/include/onnxruntime and run this script again"
fi

if [  ! -f  /usr/include/itidl_rt.h ]; then
    git clone -b master git://git.ti.com/processor-sdk-vision/arm-tidl.git
    cp arm-tidl/rt/inc/itidl_rt.h  /usr/include/
    cp arm-tidl/rt/inc/itvm_rt.h /usr/include/
    rm -r arm-tidl
    cd $HOME/
else
    echo "skipping itidl_rt.h setup: found /usr/include/itidl_rt.h"
    echo "To redo the setup delete: /usr/include/itidl_rt.h and run this script again"
fi

# establish soft links
PYTHON_DIST=/usr/local/lib/python3.10/dist-packages
# for gst-plugins
ln -s $PYTHON_DIST/dlr/libdlr.so /usr/lib/libdlr.so
# for dl-inferer
mkdir -p /usr/local/dlr
ln -s $PYTHON_DIST/dlr/libdlr.so /usr/local/dlr/libdlr.so
mkdir -p /usr/include/neo-ai-dlr
ln -s $PYTHON_DIST/dlr/include /usr/include/neo-ai-dlr/include

cd $SCRIPTDIR
