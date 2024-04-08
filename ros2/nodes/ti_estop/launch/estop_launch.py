import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def finalize_node(context, *args, **kwargs):
    zed_sn = LaunchConfiguration("zed_sn").perform(context)
    dl_model_path = "/opt/model_zoo/ONR-SS-7618-deeplabv3lite-mobv2-qat-robokit-768x432"

    lut_folder = "/opt/robotics_sdk/ros1/drivers/zed_capture/config"
    left_lut_file_path  = os.path.join(lut_folder, zed_sn+"_HD_LUT_left.bin")
    right_lut_file_path = os.path.join(lut_folder, zed_sn+"_HD_LUT_right.bin")

    exportPerfStats = int(LaunchConfiguration("exportPerfStats").perform(context))
    print(f"exportPerfStats = {exportPerfStats}")

    params = [
        os.path.join(get_package_share_directory('ti_estop'),'config','params.yaml'),
        {
            "left_lut_file_path":       left_lut_file_path,
            "right_lut_file_path":      right_lut_file_path,
            "dl_model_path":            dl_model_path,
            "left_input_topic_name":    "camera/left/image_raw",
            "right_input_topic_name":   "camera/right/image_raw",
            "camera_info_topic":        "camera/right/camera_info",
            "rectified_image_topic":    "camera/right/image_rect_mono",
            "semseg_cnn_tensor_topic":  "vision_cnn/tensor",
            "bounding_box_topic":       "detection3D/BB3D",
            "raw_disparity_topic_name": "camera/disparity/raw",
            "ogmap_topic_name":         "detection3D/ogmap",
            "estop_topic_name":         "detection3D/estop",
            "exportPerfStats":           exportPerfStats,
        }
    ]

    node = Node(
        package = "ti_estop",
        executable = "estop",
        name = "estop",
        output = "screen",
        emulate_tty = True,
        parameters = params
    )

    return [node]

def generate_launch_description():
    ld = LaunchDescription()

    # String for ZED camera serial number
    zed_sn_arg = DeclareLaunchArgument(
        name="zed_sn",
        default_value=TextSubstitution(text="SN5867575"),
        description='string for ZED camera serial number'
    )

    # Flag for exporting the performance data to a file: 0 - disable, 1 - enable
    exportPerfStats_arg = DeclareLaunchArgument(
        name="exportPerfStats",
        default_value=TextSubstitution(text="0"),
        description='flag for exporting the performance data'
    )

    ld.add_action(zed_sn_arg)
    ld.add_action(exportPerfStats_arg)
    ld.add_action(OpaqueFunction(function=finalize_node))

    return ld
