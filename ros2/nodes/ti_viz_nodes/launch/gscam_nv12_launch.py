from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    width_arg = DeclareLaunchArgument(
        name ='width',
        default_value ='1280',
        description ='Width of the image'
    )

    height_arg = DeclareLaunchArgument(
        name ='height',
        default_value = '720',
        description = 'Height of the image'
    )

    # color conversion for input image_raw for visualization
    params = [{
        "width": LaunchConfiguration('width'),
        "height": LaunchConfiguration('height'),
        "input_yuv_topic": "camera/image_raw",
        "output_rgb_topic": "camera/image_raw_rgb",
        "yuv_format": "YUV420",
        "yuv420_luma_only": False,
    }]
    yuv2rbg_node = Node(
        package = "ti_viz_nodes",
        executable = "viz_color_conv_yuv2rgb",
        name = "viz_color_conv_yuv2rgb_node",
        output = "screen",
        parameters = params
    )

    # image_view node
    image_view_node = Node(
        package = "image_view",
        executable = "image_view",
        name = "image_view",
        output = "screen",
        parameters = [
            {"autosize": True},
        ],
        remappings=[
            ("image", "/camera/image_raw_rgb"),
        ]
    )

    # Create the launch description with launch and node information
    ld = LaunchDescription()
    ld.add_action(width_arg)
    ld.add_action(height_arg)
    ld.add_action(yuv2rbg_node)
    ld.add_action(image_view_node)

    return ld
