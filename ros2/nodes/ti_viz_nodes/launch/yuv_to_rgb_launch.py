from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    params = [{
        "width":            1280,
        "height":           720,
        "input_yuv_topic":  "camera/right/image_raw",
        "output_rgb_topic": "camera/right/image_raw_rgb",
        "yuv_format":       "YUV422"
    }]

    node = Node(
        package = "ti_viz_nodes",
        executable = "viz_color_conv_yuv2rgb",
        name = "viz_color_conv_yuv2rgb",
        output = "screen",
        parameters = params
    )

    ld.add_action(node)
    return ld
