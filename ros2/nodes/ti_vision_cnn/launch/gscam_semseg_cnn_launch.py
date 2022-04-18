import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

# Input image format: 0 - VX_DF_IMAGE_U8, 1 - VX_DF_IMAGE_NV12, 2 - VX_DF_IMAGE_UYVY
image_format = 1
lut_file_path = "/opt/robotics_sdk/ros1/drivers/mono_capture/config/C920_HD_LUT.bin"
dl_model_path = "/opt/model_zoo/TVM-SS-5818-deeplabv3lite-mobv2-qat-robokit-768x432"
# dl_model_path = "/opt/model_zoo/ONR-SS-8818-deeplabv3lite-mobv2-qat-robokit-768x432"

def get_launch_file(pkg, file_name):
    pkg_dir = get_package_share_directory(pkg)
    return os.path.join(pkg_dir, 'launch', file_name)

def generate_launch_description():
    ld = LaunchDescription()

    # ti_vision_cnn node
    config = os.path.join(
        get_package_share_directory('ti_vision_cnn'),
        'config',
        'params.yaml'
    )
    params = [
        config,
        {
            "image_format":             image_format,
            "lut_file_path":            lut_file_path,
            "dl_model_path":            dl_model_path,
            "input_topic_name":         "camera/image_raw",
            "rectified_image_topic":    "camera/image_rect_nv12",
            "rectified_image_frame_id": "right_frame",
            "vision_cnn_tensor_topic":  "vision_cnn/tensor",
        },
    ]
    cnn_node = Node(
        package = "ti_vision_cnn",
        executable = "vision_cnn",
        name = "vision_cnn",
        output = "screen",
        parameters = params
    )

    # Include gscam2 launch file
    cam_launch_file = get_launch_file(
        pkg='gscam2',
        file_name='v4l_mjpg_launch.py'
    )
    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cam_launch_file)
    )

    ld.add_action(cnn_node)
    ld.add_action(cam_launch)

    return ld

