import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import math

def get_launch_file(pkg, file_name):
    pkg_dir = get_package_share_directory(pkg)
    return os.path.join(pkg_dir, 'launch', file_name)

def generate_launch_description():
    cam_id_arg = DeclareLaunchArgument(
        'cam_id',
        default_value='0',
        description='video ID of the camera'
    )

    subdev_id_arg = DeclareLaunchArgument(
        'subdev_id',
        default_value='0',
        description='subdev ID of the camera'
    )

    # include gscam2 launch file
    # ros2 launch ti_objdet_radar v4l_imx219_launch.py image_topic_name:=image_raw_nv12
    gscam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_file('gscam2', 'v4l_imx219_launch.py')),
        launch_arguments={
            "cam_id": LaunchConfiguration('cam_id'),
            "subdev_id": LaunchConfiguration('subdev_id'),
            "image_topic_name": "image_raw_nv12",
        }.items()
    )

    # include mmwave driver launch file
    # ros2 launch ti_mmwave_rospkg IWR6843.py cfg_file:=6843ISK_Tracking.cfg rviz:=false
    mmwave_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_launch_file('ti_mmwave_rospkg', 'IWR6843.py')),
        launch_arguments={
            "cfg_file": '6843ISK_Tracking.cfg',
            "command_port": "/dev/ttyUSB0",
            "data_port": "/dev/ttyUSB1",
            "rviz": 'false',
        }.items()
    )

    # TFs
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'base_link'],
        name='map_to_base_link'
    )

    base_link_to_radar_frame_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.3', '0.0', '0.0', '0.0', 'base_link', 'ti_mmwave_0'],
        name='base_link_to_radar_frame'
    )

    radar_frame_to_camera_frame_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.018', f'{-math.pi/2.}', '0.0', f'{-math.pi/2.}', 'ti_mmwave_0', 'camera'],
        name='radar_frame_to_camera_frame'
    )

    ld = LaunchDescription([
        cam_id_arg,
        subdev_id_arg,
        gscam_launch,
        mmwave_include,
        map_to_base_link_node,
        base_link_to_radar_frame_node,
        radar_frame_to_camera_frame_node
    ])

    return ld
