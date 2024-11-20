import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# path to the DL model
soc = os.getenv('SOC')
if soc in ['j721e', 'j721s2', 'j784s4']:
    default_dl_model_path = "/opt/model_zoo/ONR-OD-8020-ssd-lite-mobv2-mmdet-coco-512x512"
elif soc in ['j722s', 'am62a']:
    default_dl_model_path = "/opt/model_zoo/TFL-OD-2020-ssdLite-mobDet-DSP-coco-320x320"
else:
    print('{} not supported'.format(soc))

def generate_launch_description():

    zed_sn = DeclareLaunchArgument('zed_sn', default_value='SN5867575')
    exportPerfStats = DeclareLaunchArgument('exportPerfStats', default_value='0')
    detVizThreshold = DeclareLaunchArgument('detVizThreshold', default_value='0.5')
    dl_model_path = DeclareLaunchArgument('dl_model_path',
        default_value=default_dl_model_path
    )

    exportPerfStats_str_arg = DeclareLaunchArgument("exportPerfStats_str", default_value=[LaunchConfiguration('exportPerfStats')])
    detVizThreshold_str_arg = DeclareLaunchArgument("detVizThreshold_str", default_value=[LaunchConfiguration('detVizThreshold')])

    lut_folder = os.path.join(os.getenv('SDK_DIR', '/opt/robotics_sdk'), 'tools', 'camera_info')
    left_lut_file_arg  = DeclareLaunchArgument('left_lut_file', default_value=[LaunchConfiguration('zed_sn'), '_HD_LUT_left.bin'])
    right_lut_file_arg = DeclareLaunchArgument('right_lut_file', default_value=[LaunchConfiguration('zed_sn'), '_HD_LUT_right.bin'])
    left_lut_file_path  = PathJoinSubstitution([lut_folder, LaunchConfiguration('left_lut_file')])
    right_lut_file_path = PathJoinSubstitution([lut_folder, LaunchConfiguration('right_lut_file')])

    node_sde = Node(
        package='ti_sde',
        executable='sde',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('ti_sde'),'config','params.yaml']),
            {
                'left_lut_file_path': left_lut_file_path,
                'right_lut_file_path': right_lut_file_path,
                'sde_algo_type': 1,
                'disparity_max': 2, # This will override the value in params.yaml
                'num_layers': 2,
                'enable_pc': 0,
                'left_input_topic': 'camera/left/image_raw',
                'right_input_topic': 'camera/right/image_raw',
                'camera_info_topic': 'camera/right/camera_info',
                'disparity_topic': 'camera/disparity/raw',
                'point_cloud_topic': 'point_cloud',
                'exportPerfStats': 0
            }
        ]
    )

    node_vision_cnn = Node(
        package='ti_vision_cnn',
        executable='vision_cnn',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('ti_vision_cnn'),'config','params.yaml']),
            {
                'image_format': 2,
                'lut_file_path': right_lut_file_path,
                'dl_model_path': LaunchConfiguration('dl_model_path'),
                'input_topic_name': 'camera/right/image_raw',
                'rectified_image_topic': 'camera/right/image_rect_nv12',
                'rectified_image_frame_id': 'right_frame',
                'vision_cnn_tensor_topic': 'vision_cnn/tensor',
                'exportPerfStats': LaunchConfiguration('exportPerfStats_str'),
                'detVizThreshold': LaunchConfiguration('detVizThreshold_str')
            }
        ]
    )

    node_objdet_range = Node(
        package='ti_objdet_range',
        executable='objdet_disparity_fusion',
        output='screen',
        parameters=[{
            'disparity_topic': 'camera/disparity/raw',
            'vision_cnn_tensor_topic': 'vision_cnn/tensor',
            'camera_info_topic': 'camera/right/camera_info',
            'outout_topic': 'camera/fused_objdet_range',
            'camera_baseline': 0.12,
            'band_height': 7,
            'confidenceTh': 1,
            # disparity_filter: 0 - max, 1 - median, 2 - mean
            'disparity_filter': 1,
            # horiz_scan_pcl: true - turn on, false to turn off
            # 'horiz_scan_pcl': True,
            'pcl_topic': 'camera/horiz_scan_pcl',
            'pcl_frame': 'camera_horiz_scan',
            'disparity_width': 1280,
            'pixels_to_exclude': 128,
            'patch_size': 16,
            'patch_stride': 8,
            # pcl_disparity_filter: 0 - max, 1 - median, 2 - mean
            'pcl_disparity_filter': 1,
            'patch_row_to_use': 20,
            'valid_pixel_ratio_th': 0.3,
        }]
    )

    base_link_to_right_frame_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.125', '-0.06', '-0.064', '-1.5707963267948966', '0.078539816339745', '-1.5707963267948966', 'base_link', 'right_frame'],
        name='base_link_to_right_frame'
    )

    base_link_to_camera_horiz_scan_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.125', '-0.06', '-0.064', '0', '0.078539816339745', '0', 'base_link', 'camera_horiz_scan'],
        name='base_link_to_camera_horiz_scan'
    )

    ld = LaunchDescription([
        zed_sn,
        exportPerfStats,
        detVizThreshold,
        dl_model_path,
        exportPerfStats_str_arg,
        detVizThreshold_str_arg,
        left_lut_file_arg,
        right_lut_file_arg,
        node_sde,
        node_vision_cnn,
        node_objdet_range,
        base_link_to_right_frame_node,
        base_link_to_camera_horiz_scan_node,
    ])


    return ld
