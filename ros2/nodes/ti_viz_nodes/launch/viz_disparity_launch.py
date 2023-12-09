from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    params = [{
        "width": 1280,
        "height": 720,
        "disparity_topic": "camera/disparity/raw",
        "cc_disparity_topic": "camera/disparity/ccDisparity",
        "cc_confidence_topic": "camera/disparity/ccConfidence"
    }]

    node = Node(
        package = "ti_viz_nodes",
        executable = "viz_disparity",
        name = "viz_disparity",
        output = "screen",
        parameters = params
    )

    ld.add_action(node)
    return ld
