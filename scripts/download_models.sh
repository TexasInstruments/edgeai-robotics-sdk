#!/bin/bash
# Download models in the model zoo
ARCH=`arch`
CURRENT_DIR=$(pwd)
if [[ "$ARCH" == "aarch64" ]]; then
    Models=(
        ONR-OD-8020-ssd-lite-mobv2-mmdet-coco-512x512
        TFL-OD-2010-ssd-mobV2-coco-mlperf-300x300
        TFL-OD-2020-ssdLite-mobDet-DSP-coco-320x320
        ONR-OD-8220-yolox-s-lite-mmdet-coco-640x640
        ONR-SS-8818-deeplabv3lite-mobv2-qat-robokit-768x432
    )
    for Model in ${Models[@]}; do
        bash /opt/edgeai-gst-apps/download_models.sh -d $Model
    done
fi
cd $CURRENT_DIR
