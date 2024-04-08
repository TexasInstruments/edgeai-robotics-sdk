import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

dl_model_path = "/opt/model_zoo/ONR-KD-7060-human-pose-yolox-s-640x640"

bagfile_default = os.path.join(os.environ['WORK_DIR'],
                                   'data/ros_bag/human_pose_nv12')

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

    detVizThreshold_arg = DeclareLaunchArgument(
        "detVizThreshold", default_value=TextSubstitution(text="0.5")
    )

    pkg_dir    = get_package_share_directory('ti_vision_cnn')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Include 6DPose launch file
    cnn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'humanpose_cnn_launch.py')),
        launch_arguments={
            "exportPerfStats": LaunchConfiguration('exportPerfStats'),
            "dl_model_path": LaunchConfiguration('dl_model_path'),
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

    ld = LaunchDescription()
    ld.add_action(exportPerfStats_arg)
    ld.add_action(bagfile_arg)
    ld.add_action(dl_model_path_arg)
    ld.add_action(detVizThreshold_arg)
    ld.add_action(cnn_launch)
    ld.add_action(bag_launch)

    return ld