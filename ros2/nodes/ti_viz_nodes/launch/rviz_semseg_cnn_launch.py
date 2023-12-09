import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    viz_dir = get_package_share_directory('ti_viz_nodes')
    rviz_dir = os.path.join(viz_dir, 'rviz')

    # color conversion for image_rect_nv12 for color visualization
    params = [{
        "width": 1280,
        "height": 720,
        "input_yuv_topic": "camera/image_rect_nv12",
        "output_rgb_topic": "camera/image_rect_color",
        "yuv_format": "YUV420",
        "yuv420_luma_only": False
    }]
    yuv2rbg_node1 = Node(
        package = "ti_viz_nodes",
        executable = "viz_color_conv_yuv2rgb",
        name = "viz_color_conv_yuv2rgb_rect_color",
        output = "screen",
        parameters = params
    )

    # color conversion for image_rect_nv12 for gray visualization
    params = [{
        "width": 1280,
        "height": 720,
        "input_yuv_topic": "camera/image_rect_nv12",
        "output_rgb_topic": "camera/image_rect_gray",
        "yuv_format": "YUV420",
        "yuv420_luma_only": True
    }]
    yuv2rbg_node2 = Node(
        package = "ti_viz_nodes",
        executable = "viz_color_conv_yuv2rgb",
        name = "viz_color_conv_yuv2rgb_rect_gray",
        output = "screen",
        parameters = params
    )

    # color conversion for image_rect_nv12 for visualization
    params = [{
        "rectified_image_topic": "camera/image_rect_gray",
        "vision_cnn_tensor_topic": "vision_cnn/tensor",
        "vision_cnn_image_topic": "vision_cnn/out_image"
    }]
    viz_semseg_node = Node(
        package = "ti_viz_nodes",
        executable = "viz_semseg",
        name = "viz_semseg",
        output = "screen",
        parameters = params
    )

    # rviz node
    rviz2_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "screen",
        arguments=["-d", os.path.join(rviz_dir, 'semseg_cnn.rviz')]
    )

    # Create the launch description with launch and node information
    ld.add_action(yuv2rbg_node1)
    ld.add_action(yuv2rbg_node2)
    ld.add_action(viz_semseg_node)
    ld.add_action(rviz2_node)

    return ld
