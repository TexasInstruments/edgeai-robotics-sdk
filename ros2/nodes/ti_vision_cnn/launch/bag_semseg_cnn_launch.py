import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

# path to the DL model
dl_model_path = "/opt/model_zoo/ONR-SS-7618-deeplabv3lite-mobv2-qat-robokit-768x432"

bagfile_default = os.path.join(os.environ['WORK_DIR'],
    'data/ros_bag/zed1_2020-11-09-18-01-08')

def generate_launch_description():
    exportPerfStats_arg = DeclareLaunchArgument(
        "exportPerfStats", default_value=TextSubstitution(text="0")
    )

    bagfile_arg = DeclareLaunchArgument(
        name="bagfile",
        default_value=TextSubstitution(text=bagfile_default)
    )

    dl_model_path_arg = DeclareLaunchArgument(
        "dl_model_path", default_value=TextSubstitution(text=dl_model_path)
    )

    pkg_dir    = get_package_share_directory('ti_vision_cnn')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Include SEMSEG launch file
    cnn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'semseg_cnn_launch.py')),
        launch_arguments={
            "exportPerfStats": LaunchConfiguration('exportPerfStats'),
            "dl_model_path": LaunchConfiguration('dl_model_path'),
        }.items()
    )

    # Include rosbag launch file
    # The rosbag launch is located under ti_sde package
    pkg_dir    = get_package_share_directory('ti_vision_cnn')
    launch_dir = os.path.join(pkg_dir, 'launch')
    bag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rosbag_remap_launch.py')
        ),
        launch_arguments={
            "bagfile": LaunchConfiguration('bagfile'),
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(exportPerfStats_arg)
    ld.add_action(bagfile_arg)
    ld.add_action(dl_model_path_arg)
    ld.add_action(cnn_launch)
    ld.add_action(bag_launch)

    return ld
