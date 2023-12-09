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

    # Include ESTOP launch file
    viz_estop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'viz_estop_launch.py')
        )
    )

    # rviz node
    rviz2_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "screen",
        arguments=["-d", os.path.join(rviz_dir, 'estop_ogmap.rviz')]
    )

    # Create the launch description with launch and node information
    ld.add_action(viz_estop_launch)
    ld.add_action(rviz2_node)

    return ld
