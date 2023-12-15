import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

# path to the DL model
soc = os.getenv('SOC')
if soc in ['j721e', 'j721s2', 'j784s4']:
    dl_model_path = "/opt/model_zoo/ONR-OD-8020-ssd-lite-mobv2-mmdet-coco-512x512"
elif soc in ['am62a']:
    dl_model_path = "/opt/model_zoo/TFL-OD-2020-ssdLite-mobDet-DSP-coco-320x320"
else:
    print('{} not supported'.format(soc))

bagfile_default = os.path.join(os.environ['WORK_DIR'],
    'data/ros_bag/zed1_2020-11-09-18-01-08')

def generate_launch_description():
    ld = LaunchDescription()

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

    detVizThreshold_arg = DeclareLaunchArgument(
        "detVizThreshold", default_value=TextSubstitution(text="0.5")
    )

    pkg_dir    = get_package_share_directory('ti_vision_cnn')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Include OBJDET launch file
    cnn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'objdet_cnn_launch.py')),
        launch_arguments={
            "exportPerfStats": LaunchConfiguration('exportPerfStats'),
            "dl_model_path":   LaunchConfiguration('dl_model_path'),
            "detVizThreshold": LaunchConfiguration('detVizThreshold'),
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

    ld.add_action(exportPerfStats_arg)
    ld.add_action(bagfile_arg)
    ld.add_action(dl_model_path_arg)
    ld.add_action(detVizThreshold_arg)
    ld.add_action(cnn_launch)
    ld.add_action(bag_launch)

    return ld
