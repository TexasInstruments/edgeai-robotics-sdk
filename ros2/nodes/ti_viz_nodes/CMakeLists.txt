cmake_minimum_required(VERSION 3.5)
project(ti_viz_nodes)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/ros2_common.cmake)

if (${BUILD_VISUALIZATION_NODES})
    # viz_disparity
    set(VIZ_DISP_SRCS src/viz_disparity_node.cpp)
    build_node(viz_disparity VIZ_DISP_SRCS)

    # YUV to RGB conversion
    set(VIZ_COL_CONV_SRCS src/viz_color_conversion_node.cpp)
    build_node(viz_color_conv_yuv2rgb VIZ_COL_CONV_SRCS)

    # viz_semseg
    set(VIZ_SEMSEG_SRCS src/viz_semseg_node.cpp)
    build_node(viz_semseg VIZ_SEMSEG_SRCS)

    # viz_objdet
    set(VIZ_OBJDET_SRCS src/viz_objdet_node.cpp)
    build_node(viz_objdet VIZ_OBJDET_SRCS)

    # viz_estop
    set(VIZ_ESTOP_SRCS src/viz_estop_node.cpp)
    build_node(viz_estop VIZ_ESTOP_SRCS)

    # viz_objdet_range_node
    set(VIZ_OBJDET_RANGE_SRCS src/viz_objdet_range_node.cpp)
    build_node(viz_objdet_range VIZ_OBJDET_RANGE_SRCS)

    # viz_pose6d
    set(VIZ_POSE6D_SRCS src/viz_pose6d_node.cpp)
    build_node(viz_pose6d VIZ_POSE6D_SRCS)

    ament_package()
endif()
