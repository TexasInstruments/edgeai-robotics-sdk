from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    params = [{
        "width": 1280,
        "height": 720,
        "tensor_width": 768,
        "tensor_height": 432,
        "rectified_image_topic": "camera/right/image_rect_mono",
        "vision_cnn_tensor_topic": "vision_cnn/tensor",
        "vision_cnn_image_topic": "vision_cnn/out_image"
    }]

    node = Node(
        package = "ti_viz_nodes",
        executable = "viz_semseg",
        name = "viz_semseg",
        output = "screen",
        parameters = params
    )

    ld.add_action(node)
    return ld
