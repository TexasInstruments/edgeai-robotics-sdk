import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter

def get_launch_file(pkg, file_name):
    pkg_dir = get_package_share_directory(pkg)
    return os.path.join(pkg_dir, 'launch', file_name)

def generate_launch_description():

    # include rosbag_play_launch.py
    # ros2 launch ti_objdet_radar rosbag_play_launch.py
    gscam_mmwave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_launch_file('ti_objdet_radar', 'rosbag_play_launch.py')
        )
    )

    # include objdet_radar_launch.py
    # ros2 launch ti_objdet_radar objdet_radar_launch.py
    vision_cnn_objdet_radar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_file('ti_objdet_radar', 'objdet_radar_launch.py')),
        launch_arguments={
            "use_sim_time": "true",
            "max_sync_offset": "0.3"
        }.items()
    )

    ld = LaunchDescription([
        gscam_mmwave_launch,
        vision_cnn_objdet_radar_launch,
    ])

    return ld
