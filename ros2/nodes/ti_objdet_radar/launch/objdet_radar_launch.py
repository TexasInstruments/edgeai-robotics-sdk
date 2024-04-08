import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
import math

# Input image format: 0 - VX_DF_IMAGE_U8, 1 - VX_DF_IMAGE_NV12, 2 - VX_DF_IMAGE_UYVY
image_format = 1
enable_ldc_node = 1
lut_file_path = "/opt/robotics_sdk/tools/camera_info/IMX219_HD_LUT.bin"

# path to the DL model
soc = os.getenv('SOC')
if soc in ['j721e', 'j721s2', 'j784s4']:
    dl_model_path = "/opt/model_zoo/ONR-OD-8020-ssd-lite-mobv2-mmdet-coco-512x512"
elif soc in ['j722s', 'am62a']:
    dl_model_path = "/opt/model_zoo/TFL-OD-2020-ssdLite-mobDet-DSP-coco-320x320"
else:
    print('{} not supported'.format(soc))

def get_launch_file(pkg, file_name):
    pkg_dir = get_package_share_directory(pkg)
    return os.path.join(pkg_dir, 'launch', file_name)

def generate_launch_description():
    # declare arguments
    exportPerfStats_arg = DeclareLaunchArgument(
        "exportPerfStats", default_value=TextSubstitution(text="0")
    )

    exportPerfStats_str_arg = DeclareLaunchArgument(
        "exportPerfStats_str", default_value=[LaunchConfiguration('exportPerfStats')]
    )

    detVizThreshold_arg = DeclareLaunchArgument(
        "detVizThreshold", default_value=TextSubstitution(text="0.5")
    )

    detVizThreshold_str_arg = DeclareLaunchArgument(
        "detVizThreshold_str", default_value=[LaunchConfiguration('detVizThreshold')]
    )

    max_sync_offset_arg = DeclareLaunchArgument(
        "max_sync_offset", default_value="0.033333"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false"
    )

    # ti_vision_cnn node
    params = [
        os.path.join(get_package_share_directory('ti_vision_cnn'), 'config', 'params.yaml'),
        {
            "width":           1280,
            "height":          720,
            "image_format":    image_format,
            "enable_ldc_node": enable_ldc_node,
            "lut_file_path":   lut_file_path,
            "dl_model_path":   dl_model_path,
            "input_topic_name":         "camera/image_raw_nv12",
            "rectified_image_topic":    "camera/image_rect_nv12",
            "rectified_image_frame_id": "camera",
            "vision_cnn_tensor_topic":  "vision_cnn/tensor",
            "exportPerfStats": LaunchConfiguration('exportPerfStats_str'),
            "detVizThreshold": LaunchConfiguration('detVizThreshold_str'),
            "use_sim_time": LaunchConfiguration('use_sim_time'),
        },
    ]
    vision_cnn_node = Node(
        package = "ti_vision_cnn",
        executable = "vision_cnn",
        name = "vision_cnn",
        output = "screen",
        emulate_tty = True,
        parameters = params
    )

    # objdet_radar
    objdet_radar_fusion_node = Node(
        package='ti_objdet_radar',
        executable='objdet_radar_fusion',
        output='screen',
        parameters=[{
            'radar_tracker_topic': 'ti_mmwave/radar_track_array',
            'vision_cnn_tensor_topic': 'vision_cnn/tensor',
            'camera_info_topic': 'camera/camera_info',
            'outout_topic': 'camera/fused_objdet_radar',
            'radar_frame_id': 'ti_mmwave_0',
            'camera_frame_id': 'camera',
            'camera_info_file': 'file:///opt/robotics_sdk/tools/camera_info/IMX219_HD_camera_info.yaml',
            'radar_track_xy_only': True,
            'max_sync_offset': LaunchConfiguration('max_sync_offset'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    ld = LaunchDescription([
        exportPerfStats_arg,
        exportPerfStats_str_arg,
        detVizThreshold_arg,
        detVizThreshold_str_arg,
        use_sim_time_arg,
        vision_cnn_node,
        objdet_radar_fusion_node,
    ])

    return ld
