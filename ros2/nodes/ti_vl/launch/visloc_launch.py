import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    exportPerfStats_arg = DeclareLaunchArgument(
        "exportPerfStats", default_value=TextSubstitution(text="0")
    )

    params = [
        os.path.join( get_package_share_directory('ti_vl'), 'config', 'params.yaml'),
        {
            "input_topic":     "camera/image_raw",
            "out_image_topic": "vis_localize/out_image",
            "out_pose_topic":  "vis_localize/pose",
            "map_topic":       "vis_localize/map",
            "exportPerfStats":  LaunchConfiguration('exportPerfStats'),
        }
    ]

    node = Node(
        package = "ti_vl",
        executable = "vl",
        name = "vl",
        output = "screen",
        emulate_tty = True,
        parameters = params
    )

    ld.add_action(exportPerfStats_arg)
    ld.add_action(node)

    return ld
