#!/bin/bash
# camera ID that is set by /opt/edgeai-gst-apps/scripts/setup_cameras.sh
CAM_ID=${CAM_ID:-0}
SUBDEV_ID=${SUBDEV_ID:-0}

# ZED camera serial number
ZED_SN=${ZED_SN:-SN18059}

# colcon build
if [[ -n "$SOC" ]]; then
    PACKAGES_SKIP="ti_viz_nodes"
    if [[ "$SOC" == "am62a" ]]; then
        PACKAGES_SKIP+=" zed_capture ti_sde ti_estop ti_vl"
    elif [[ "$SOC" == "j722s" ]]; then
        PACKAGES_SKIP+=" ti_vl"
    fi

else # PC side
    PACKAGES_SKIP="ti_external ti_vision_cnn ti_sde ti_estop ti_vl ti_objdet_range ti_ros_gst_plugins"
fi
ARGS="--cmake-force-configure --symlink-install --packages-skip $PACKAGES_SKIP"
if [[ -n "$SOC" && ( "$SOC" == "j721e" || "$SOC" == "am62a" ) ]]; then
    ARGS+=" --executor sequential"
fi
alias colcon_build="colcon build --base-paths $SDK_DIR/ros2 $ARGS && source install/setup.bash"
alias colcon_build_select='function _colcon_build_select() { colcon build --base-paths $SDK_DIR/ros2 --packages-select "${@:1}" --executor sequential --symlink-install && source install/setup.bash; }; _colcon_build_select'
alias clean_ws="rm -rf build/* install/* log/*"
alias clean_pkg='function _clean_pkg() { rm -rf build/"$1" install/"$1"; }; _clean_pkg'

# camera_setups on the SK board
alias setup_cameras='IMX219_CAM_FMT="[fmt:SRGGB10_1X10/1640x1232]" /opt/edgeai-gst-apps/scripts/setup_cameras.sh'

# demo launches with ROSBAG
alias bag_sde="ros2 launch ti_sde bag_sde_launch.py"
alias bag_sdepcl="ros2 launch ti_sde bag_sde_pcl_launch.py"
alias bag_ss="ros2 launch ti_vision_cnn bag_semseg_cnn_launch.py"
alias bag_od="ros2 launch ti_vision_cnn bag_objdet_cnn_launch.py"
alias bag_estop="ros2 launch ti_estop bag_estop_launch.py"
alias bag_6dpose="ros2 launch ti_vision_cnn bag_6dpose_cnn_launch.py"
alias bag_hp="ros2 launch ti_vision_cnn bag_humanpose_cnn_launch.py"
alias bag_vl="ros2 launch ti_vl bag_visloc_launch.py"
alias bag_odrange="ros2 launch ti_objdet_range bag_objdet_range_launch.py"
alias bag_odradar="ros2 launch ti_objdet_radar bag_objdet_radar_launch.py"

# demo launches with webcam
alias mono_cap="ros2 launch mono_capture mono_capture_launch.py cam_id:=\$CAM_ID"
alias gscam_cap="ros2 launch gscam2 v4l_mjpg_launch.py cam_id:=\$CAM_ID"
alias gscam_ss="ros2 launch ti_vision_cnn gscam_semseg_cnn_launch.py cam_id:=\$CAM_ID"
alias gscam_od="ros2 launch ti_vision_cnn gscam_objdet_cnn_launch.py cam_id:=\$CAM_ID"
alias gscam_6dpose="ros2 launch ti_vision_cnn gscam_6dpose_cnn_launch.py cam_id:=\$CAM_ID"
alias gscam_hp="ros2 launch ti_vision_cnn gscam_humanpose_cnn_launch.py cam_id:=\$CAM_ID"
alias gscam_odradar="ros2 launch ti_objdet_radar gscam_mmwave_objdet_radar_launch.py cam_id:=\$CAM_ID"

# demo launches with ZED stereo camera
alias zed_cap="ros2 launch zed_capture zed_capture_launch.py cam_id:=\$CAM_ID zed_sn_str:=\$ZED_SN"
alias zed_sde="ros2 launch ti_sde zed_sde_launch.py cam_id:=\$CAM_ID zed_sn:=\$ZED_SN"
alias zed_sdepcl="ros2 launch ti_sde zed_sde_pcl_launch.py cam_id:=\$CAM_ID zed_sn:=\$ZED_SN"
alias zed_ss="ros2 launch ti_vision_cnn zed_semseg_cnn_launch.py cam_id:=\$CAM_ID zed_sn:=\$ZED_SN"
alias zed_od="ros2 launch ti_vision_cnn zed_objdet_cnn_launch.py cam_id:=\$CAM_ID zed_sn:=\$ZED_SN"
alias zed_estop="ros2 launch ti_estop zed_estop_launch.py cam_id:=\$CAM_ID zed_sn:=\$ZED_SN"
alias zed_odrange="ros2 launch ti_objdet_range zed_objdet_range_launch.py cam_id:=\$CAM_ID zed_sn:=\$ZED_SN"

# demo launches with IMX219 CSI camera
alias imx219_cap="ros2 launch gscam2 v4l_imx219_launch.py cam_id:=\$CAM_ID subdev_id:=\$SUBDEV_ID"
alias imx219_ss="ros2 launch ti_vision_cnn gscam_semseg_cnn_imx219_launch.py cam_id:=\$CAM_ID subdev_id:=\$SUBDEV_ID"
alias imx219_od="ros2 launch ti_vision_cnn gscam_objdet_cnn_imx219_launch.py cam_id:=\$CAM_ID subdev_id:=\$SUBDEV_ID"

# demo launches with IMX390 CSI camera with Fusion board
alias gen_imx390_ldc="bash /opt/robotics_sdk/tools/mono_camera/imx390_ldc.sh"
alias imx390_cap="ros2 launch gscam2 v4l_imx390_launch.py cam_id:=\$CAM_ID subdev_id:=\$SUBDEV_ID"
alias imx390_cap_fhd="ros2 launch gscam2 v4l_imx390_launch.py cam_id:=\$CAM_ID subdev_id:=\$SUBDEV_ID width:=1920 height:=1080"
alias imx390_cap_raw="ros2 launch gscam2 v4l_imx390_raw_launch.py cam_id:=\$CAM_ID subdev_id:=\$SUBDEV_ID"
alias imx390_ss="ros2 launch ti_vision_cnn gscam_semseg_cnn_imx390_launch.py cam_id:=\$CAM_ID subdev_id:=\$SUBDEV_ID"
alias imx390_od="ros2 launch ti_vision_cnn gscam_objdet_cnn_imx390_launch.py cam_id:=\$CAM_ID subdev_id:=\$SUBDEV_ID"

# mmWave radar
alias mmwave_cap="ros2 launch ti_mmwave_rospkg 6843ISK_Standard.py rviz:=false"

# visualization on Ubuntu PC
alias viz_zed="ros2 run image_view image_view --ros-args --remap /image:=/camera/right/image_raw"
alias viz_gscam="ros2 launch ti_viz_nodes gscam_nv12_launch.py"
alias viz_gscam_imx390raw="ros2 launch ti_viz_nodes gscam_nv12_launch.py width:=1936 height:=1100"
alias viz_mono="ros2 run image_view image_view --ros-args --remap /image:=/camera/image_raw"
alias viz_sde="ros2 launch ti_viz_nodes rviz_sde_launch.py"
alias viz_sdepcl="ros2 launch ti_viz_nodes rviz_sde_pcl_launch.py"
alias viz_ss="ros2 launch ti_viz_nodes rviz_semseg_cnn_launch.py"
alias viz_od="ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py"
alias viz_6dpose="ros2 launch ti_viz_nodes rviz_6dpose_cnn_launch.py width:=1280 height:=960"
alias viz_6dhd="ros2 launch ti_viz_nodes rviz_6dpose_cnn_launch.py width:=1280 height:=720"
alias viz_hp="ros2 launch ti_viz_nodes rviz_humanpose_cnn_launch.py"
alias viz_estop="ros2 launch ti_viz_nodes rviz_estop_launch.py"
alias viz_odrange="ros2 launch ti_viz_nodes rviz_objdet_range_launch.py"
alias viz_vl="ros2 launch ti_viz_nodes rviz_visloc_launch.py"
alias viz_mmwave="ros2 launch ti_mmwave_rospkg rviz_launch.py"
alias viz_odradar="ros2 launch ti_viz_nodes rviz_objdet_radar_launch.py"

