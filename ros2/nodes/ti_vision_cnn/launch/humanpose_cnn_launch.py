import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def finalize_node(context, *args, **kwargs):
    image_format = int(LaunchConfiguration("image_format").perform(context))
    zed_sn = LaunchConfiguration("zed_sn").perform(context)
    lut_folder = os.path.join(os.getenv('SDK_DIR', '/opt/robotics_sdk'), 'tools', 'camera_info')
    lut_file_path = os.path.join(lut_folder, zed_sn+"_HD_LUT_right.bin")
    enable_ldc_node = int(LaunchConfiguration("enable_ldc_node").perform(context))
    dl_model_path   = LaunchConfiguration("dl_model_path").perform(context)
    exportPerfStats = int(LaunchConfiguration("exportPerfStats").perform(context))
    detVizThreshold = float(LaunchConfiguration("detVizThreshold").perform(context))
    print(f"exportPerfStats = {exportPerfStats}")

    params = [
        os.path.join(get_package_share_directory('ti_vision_cnn'),'config','params.yaml'),
        {
            "width":                    1280,
            "height":                   720,
            "image_format":             image_format,
            "lut_file_path":            lut_file_path,
            "enable_ldc_node":          enable_ldc_node,
            "dl_model_path":            dl_model_path,
            "input_topic_name":         "camera/image_raw",
            "rectified_image_topic":    "camera/image_rect_nv12",
            "rectified_image_frame_id": "right_frame",
            "vision_cnn_tensor_topic":  "vision_cnn/tensor",
            "exportPerfStats":           exportPerfStats,
            "detVizThreshold":           detVizThreshold,
        },
    ]

    node = Node(
        package = "ti_vision_cnn",
        executable = "vision_cnn",
        name = "vision_cnn",
        output = "screen",
        emulate_tty = True,
        parameters = params
    )

    return [node]


def generate_launch_description():

    # ZED camera serial number string
    zed_sn = DeclareLaunchArgument(
        name="zed_sn",
        default_value=TextSubstitution(text="SN5867575"),
        description='string for ZED camera serial number'
    )

    # Input image format: 0 - VX_DF_IMAGE_U8, 1 - VX_DF_IMAGE_NV12, 2 - VX_DF_IMAGE_UYVY
    image_format = DeclareLaunchArgument(
        name="image_format",
        default_value=TextSubstitution(text='1'),
        description='input image format'
    )

    # enable ldc node flag
    enable_ldc_node = DeclareLaunchArgument(
        name="enable_ldc_node",
        default_value=TextSubstitution(text='0'),
        description='enable ldc node flag'
    )

    # DL model path
    dl_model_path = DeclareLaunchArgument(
        name="dl_model_path",
        default_value=TextSubstitution(text="/opt/model_zoo/ONR-KD-7060-human-pose-yolox-s-640x640"),
        description='DL model path'
    )

    # Flag for exporting the performance data to a file: 0 - disable, 1 - enable
    exportPerfStats = DeclareLaunchArgument(
        name="exportPerfStats",
        default_value=TextSubstitution(text="0"),
        description='flag for exporting the performance data'
    )

    # Threshold for object detection visualization
    detVizThreshold = DeclareLaunchArgument(
        name="detVizThreshold",
        default_value=TextSubstitution(text="0.5"),
        description='Threshold for 6D Pose visualization'
    )

    ld = LaunchDescription()
    ld.add_action(zed_sn)
    ld.add_action(image_format)
    ld.add_action(enable_ldc_node)
    ld.add_action(dl_model_path)
    ld.add_action(exportPerfStats)
    ld.add_action(detVizThreshold)
    ld.add_action(OpaqueFunction(function=finalize_node))

    return ld