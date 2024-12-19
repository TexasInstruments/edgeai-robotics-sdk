import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def finalize_node(context, *args, **kwargs):
    zed_sn        = LaunchConfiguration("zed_sn").perform(context)
    sde_algo_type = int(LaunchConfiguration("sde_algo_type").perform(context))
    enable_pc     = int(LaunchConfiguration("enable_pc").perform(context))
    print(f"sde_algo_type = {sde_algo_type}")
    print(f"enable_pc = {enable_pc}")

    lut_folder = os.path.join(os.getenv('SDK_DIR', '/opt/robotics_sdk'), 'tools', 'camera_info')
    left_lut_file_path  = os.path.join(lut_folder, zed_sn+"_HD_LUT_left.bin")
    right_lut_file_path = os.path.join(lut_folder, zed_sn+"_HD_LUT_right.bin")

    exportPerfStats = int(LaunchConfiguration("exportPerfStats").perform(context))
    print(f"exportPerfStats = {exportPerfStats}")

    params = [
        os.path.join(get_package_share_directory('ti_sde'),'config','params.yaml'),
        {
            "left_lut_file_path":  left_lut_file_path,
            "right_lut_file_path": right_lut_file_path,
            "sde_algo_type":       sde_algo_type,
            "num_layers":          2,
            "enable_pc":           enable_pc,
            "left_input_topic":    "camera/left/image_raw",
            "right_input_topic":   "camera/right/image_raw",
            "camera_info_topic":   "camera/right/camera_info",
            "disparity_topic":     "camera/disparity/raw",
            "point_cloud_topic":   "point_cloud",
            "exportPerfStats":     exportPerfStats,
        }
    ]

    node = Node(
        package = "ti_sde",
        executable = "sde",
        name = "sde",
        output = "screen",
        emulate_tty = True,
        parameters = params
    )

    return [node]


def generate_launch_description():
    ld = LaunchDescription()

    # ZED camera serial number string
    zed_sn = DeclareLaunchArgument(
        name="zed_sn",
        default_value=TextSubstitution(text="SN5867575"),
        description='string for ZED camera serial number'
    )

    # SDE algorithm type: 0 - single-layer SDE, 1 - multi-layer SDE
    sde_algo_type = DeclareLaunchArgument(
        name="sde_algo_type",
        default_value=TextSubstitution(text="1"),
        description='string for ZED camera serial number'
    )

    # Enable point-cloud generation: 0 - disable, 1 - enable
    enable_pc = DeclareLaunchArgument(
        name="enable_pc",
        default_value=TextSubstitution(text="1"),
        description='Enable point-cloud generation'
    )

    # Flag for exporting the performance data to a file: 0 - disable, 1 - enable
    exportPerfStats_arg = DeclareLaunchArgument(
        name="exportPerfStats",
        default_value=TextSubstitution(text="0"),
        description='flag for exporting the performance data'
    )

    ld.add_action(zed_sn)
    ld.add_action(sde_algo_type)
    ld.add_action(enable_pc)
    ld.add_action(exportPerfStats_arg)
    ld.add_action(OpaqueFunction(function=finalize_node))

    return ld

