#!/bin/bash
# Download models from the EdgeAI Model Zoo
ARCH=`arch`
CURRENT_DIR=$(pwd)
MODEL_DIR=/opt/model_zoo

if [[ "$SOC" == "j721e" ]]; then
    URLs="https://software-dl.ti.com/jacinto7/esd/modelzoo/$EDGEAI_SDK_VERSION/modelartifacts/TDA4VM/8bits/"
elif [[ "$SOC" == "j721s2" ]]; then
    URLs="https://software-dl.ti.com/jacinto7/esd/modelzoo/$EDGEAI_SDK_VERSION/modelartifacts/AM68A/8bits/"
elif [[ "$SOC" == "j784s4" ]]; then
    URLs="https://software-dl.ti.com/jacinto7/esd/modelzoo/$EDGEAI_SDK_VERSION/modelartifacts/AM69A/8bits/"
elif [[ "$SOC" == "am62a" ]]; then
    URLs="https://software-dl.ti.com/jacinto7/esd/modelzoo/$EDGEAI_SDK_VERSION/modelartifacts/AM62A/8bits/"
elif [[ "$SOC" == "am62x" ]]; then
    URLs="https://software-dl.ti.com/jacinto7/esd/modelzoo/$EDGEAI_SDK_VERSION/modelartifacts/AM62/32bits/"
else
    echo "ERROR: "$SOC" not supported"
    exit
fi

if [[ "$ARCH" == "aarch64" ]]; then
    echo "[download_models] Installing the models used in the SDK ..."

    # Download object detection and semantic segmentation models
    MODELS=(
        TFL-OD-2020-ssdLite-mobDet-DSP-coco-320x320
        ONR-OD-8020-ssd-lite-mobv2-mmdet-coco-512x512
        ONR-OD-8220-yolox-s-lite-mmdet-coco-640x640
        ONR-SS-8818-deeplabv3lite-mobv2-qat-robokit-768x432
    )
    for model in ${MODELS[@]}; do
        bash /opt/edgeai-gst-apps/download_models.sh -d $model
    done

    # Download 6D pose estimation and visual localization models
    MODEL_NAMES=(
        "ONR-6DP-7200-yolox-s-6d-object_pose-640x480"
        "visloc-7500_onnxrt_carla_edgeai-tv_tiad_dkaze_carla_768x384_model_onnx"
    )
    MODEL_FILES=(
        "6dpose-7200_onnxrt_ycbv_edgeai-yolox_yolox_s_object_pose_ti_lite_640x480_57p75_onnx.tar.gz"
        "visloc-7500_onnxrt_carla_edgeai-tv_tiad_dkaze_carla_768x384_model_onnx.tar.gz"
    )
    if [[ "$SOC" == "am62a" ]]; then
        unset 'MODEL_NAMES[1]'             # arrays are zero-indexed
        unset 'MODEL_FILES[1]'
        MODEL_NAMES=("${MODEL_NAMES[@]}")  # re-index the array
        MODEL_FILES=("${MODEL_FILES[@]}")
    fi

    for i in "${!MODEL_NAMES[@]}"; do
        model_name=${MODEL_NAMES[i]}
        model_file=${MODEL_FILES[i]}

        if [ -d $MODEL_DIR/$model_name ]; then
            echo "$model_name already exists under /opt/model_zoo"
        else
            mkdir -p $MODEL_DIR/$model_name
            wget --proxy off -P $MODEL_DIR $URLs/$model_file
            tar xzf $MODEL_DIR/$model_file -C $MODEL_DIR/$model_name
            rm -rf $MODEL_DIR/$model_file
        fi
    done

    echo "[download_models] Done"
fi
cd $CURRENT_DIR
