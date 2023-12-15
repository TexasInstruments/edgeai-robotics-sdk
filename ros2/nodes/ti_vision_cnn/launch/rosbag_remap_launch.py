import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import LogInfo

def generate_launch_description():
    ld = LaunchDescription()

    # Get the location of th ROS bags using environment variable
    bagfile_default = os.path.join(os.environ['WORK_DIR'],
                                   'data/ros_bag/zed1_2020-11-09-18-01-08')

    # ratefactor: arg that can be set from the command line or a default will be used
    ratefactor = DeclareLaunchArgument(
        name="ratefactor",
        default_value=TextSubstitution(text="1.0")
    )

    # bagfile: arg that can be set from the command line or a default will be used
    bagfile = DeclareLaunchArgument(
        name="bagfile",
        default_value=TextSubstitution(text=bagfile_default)
    )

    ld.add_action(ratefactor)
    ld.add_action(bagfile)

    # rosbag process
    rosbag_process = ExecuteProcess(
        output = "screen",
        cmd=['ros2', 'bag', 'play',
            '-r', LaunchConfiguration('ratefactor'),
            '-l', LaunchConfiguration('bagfile'),
            '--read-ahead-queue-size', '5000',
            '--topics',
            '/camera/left/image_raw',
            '/camera/right/image_raw',
            '/camera/left/camera_info',
            '/camera/right/camera_info',
            '--remap',
            '/camera/right/image_raw:=/camera/image_raw',
            '/camera/right/camera_info:=/camera/camera_info'
        ],
        on_exit=[
            LogInfo(msg="rosbag2 exited")
        ]
    )

    ld.add_action(rosbag_process)

    return ld
