import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    viz_dir = get_package_share_directory('ti_viz_nodes')
    rviz_dir = os.path.join(viz_dir, 'rviz')

    # Declare launch arguments
    width_arg = DeclareLaunchArgument(
        name='width',
        default_value='1280',
        description='Width of the input image'
    )

    height_arg = DeclareLaunchArgument(
        name='height',
        default_value='720',
        description='Height of the input image'
    )

    # generate disparity map and confidence map from disparity_raw
    viz_disparity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(viz_dir, 'launch', 'viz_disparity_launch.py')
        )
    )

    # color conversion for image_rect_nv12 for visualization
    yuv2rbg_node = Node(
        package = "ti_viz_nodes",
        executable = "viz_color_conv_yuv2rgb",
        name = "viz_color_conv_yuv2rgb_node_rect",
        output = "screen",
        parameters = [{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'input_yuv_topic': "camera/right/image_rect_nv12",
            'output_rgb_topic': "camera/right/image_rect_color",
            'yuv_format': 'YUV420',
            'yuv420_luma_only': False
        }]
    )

    # viz_objdet_range
    viz_node = Node(
        package = "ti_viz_nodes",
        executable = "viz_objdet_range",
        name = "viz_objdet_range",
        output = "screen",
        parameters = [{
            "fused_obj_range_topic": "camera/fused_objdet_range",
            "rectified_image_topic": "camera/right/image_rect_color",
            "output_image_topic": "vision_cnn/out_image",
            "box_color_rgb": [102, 255, 102],
            "patch_color_rgb": [255, 128, 0],
            "text_color_rgb": [244,  35, 232],
            "text_bg_color_rgb": [120, 120, 120],
        }]
    )

    # rviz node
    rviz2_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "screen",
        arguments=["-d", os.path.join(rviz_dir, 'objdet_range_disp_pcl.rviz')]
    )

    # Create the launch description with launch and node information
    ld = LaunchDescription([
        width_arg,
        height_arg,
        viz_disparity_launch,
        yuv2rbg_node,
        viz_node,
        rviz2_node,
    ])

    return ld
