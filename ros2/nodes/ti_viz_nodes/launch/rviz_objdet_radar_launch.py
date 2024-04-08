import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    viz_dir = get_package_share_directory('ti_viz_nodes')
    rviz_dir = os.path.join(viz_dir, 'rviz')

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

    # color conversion for image_rect_nv12 for visualization
    params = [{
        "width": LaunchConfiguration('width'),
        "height": LaunchConfiguration('height'),
        "input_yuv_topic": "camera/image_rect_nv12",
        "output_rgb_topic": "camera/image_rect",
        "yuv_format": "YUV420",
        "yuv420_luma_only": False
    }]
    rect_yuv2rbg_node = Node(
        package = "ti_viz_nodes",
        executable = "viz_color_conv_yuv2rgb",
        name = "viz_color_conv_yuv2rgb_node_rect",
        output = "screen",
        parameters = params
    )

    # color conversion for image_raw_nv12 for visualization
    params = [{
        "width": LaunchConfiguration('width'),
        "height": LaunchConfiguration('height'),
        "input_yuv_topic": "camera/image_raw_nv12",
        "output_rgb_topic": "camera/image_raw",
        "yuv_format": "YUV420",
        "yuv420_luma_only": False
    }]
    raw_yuv2rbg_node = Node(
        package = "ti_viz_nodes",
        executable = "viz_color_conv_yuv2rgb",
        name = "viz_color_conv_yuv2rgb_node_rect",
        output = "screen",
        parameters = params
    )

    # viz_objdet_radar
    viz_node = Node(
        package = "ti_viz_nodes",
        executable = "viz_objdet_radar",
        name = "viz_objdet_radar",
        output = "screen",
        parameters = [{
            "fused_obj_radar_topic": "camera/fused_objdet_radar",
            "rectified_image_topic": "camera/image_rect",
            "output_image_topic": "vision_cnn/out_image",
            "box_color_rgb": [102, 255, 102],
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
        arguments=["-d", os.path.join(rviz_dir, 'objdet_radar.rviz')]
    )

    # Create the launch description with launch and node information
    ld.add_action(width_arg)
    ld.add_action(height_arg)
    ld.add_action(rect_yuv2rbg_node)
    ld.add_action(raw_yuv2rbg_node)
    ld.add_action(viz_node)
    ld.add_action(rviz2_node)

    return ld
