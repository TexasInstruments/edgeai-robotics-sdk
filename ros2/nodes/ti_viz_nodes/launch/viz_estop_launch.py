from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    params = [{
        "width": 1280,
        "height": 720,
        "tensor_width": 768,
        "tensor_height": 432,
        "show_ground": 0,
        "rectified_image_topic": "camera/right/image_rect_mono",
        "bounding_box_topic": "detection3D/BB3D",
        "semseg_cnn_tensor_topic": "vision_cnn/tensor",
        "bounding_box_image_topic": "detection3D/BB3DImage"
    }]

    node = Node(
        package = "ti_viz_nodes",
        executable = "viz_estop",
        name = "viz_estop",
        output = "screen",
        parameters = params
    )

    ld.add_action(node)
    return ld

