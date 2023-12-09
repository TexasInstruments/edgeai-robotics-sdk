import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    viz_dir = get_package_share_directory('ti_viz_nodes')
    launch_dir = os.path.join(viz_dir, 'launch')
    rviz_dir = os.path.join(viz_dir, 'rviz')

    # Include color conversion launch file
    yuv_to_rgb_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'yuv_to_rgb_launch.py')
        )
    )

    # Include disparity visualization launch file
    viz_disparity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'viz_disparity_launch.py')
        )
    )

    # rviz node
    rviz2_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "screen",
        arguments=["-d", os.path.join(rviz_dir, 'sde.rviz')]
    )

    # Create the launch description with launch and node information
    ld.add_action(yuv_to_rgb_launch)
    ld.add_action(viz_disparity_launch)
    ld.add_action(rviz2_node)

    return ld
