cmake_minimum_required(VERSION 3.0.2)

project(ti_core)

find_package(Eigen3 3.1.0 REQUIRED)

if(${BUILD_CORE_NODES})
    include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ti_core_common.cmake)

    set(COMMON_SRCS
        common/src/cm_common.cpp
        common/src/cm_ldc_node_cntxt.cpp
        common/src/cm_preproc_node_cntxt.cpp
        common/src/cm_profile.cpp
        common/src/cm_scaler_node_cntxt.cpp
        common/src/cm_pre_process_image.cpp
        common/src/cm_post_process_image.cpp
        common/src/cm_post_process_image_segmentation.cpp
        common/src/cm_post_process_image_object_detect.cpp
        common/src/cm_post_process_pose6D.cpp
        common/src/cm_post_process_humanpose.cpp
    )

    if (${SDE_AVAILABLE})
        list(APPEND COMMON_SRCS
            common/src/cm_remote_service.cpp
            common/src/cm_displarity_utils.cpp
        )
    endif()

    if (${TIADALG_AVAILABLE})
        list(APPEND COMMON_SRCS
            common/src/cm_pose_calc_node_cntxt.cpp
            common/src/cm_pose_viz_node_cntxt.cpp
        )
    endif()

    set(VISION_CNN_CORE_SRCS
        ti_vision_cnn/src/vision_cnn.cpp
        ti_vision_cnn/src/vision_cnn_manage_graph.cpp
    )

    set(SDE_CORE_SRCS
        ti_sde/src/sde.cpp
        ti_sde/src/sde_manage_graph.cpp
    )

    set(ESTOP_CORE_SRCS
        ti_estop/src/estop.cpp
        ti_estop/src/estop_manage_graph.cpp
    )

    set(VL_CORE_SRCS
        ti_vl/src/visloc.cpp
        ti_vl/src/visloc_manage_graph.cpp
    )

    set(SDE_SUBGRAPH_SRCS
        subgraphs/sde_ldc/src/sde_ldc.cpp
        subgraphs/sde_ldc/src/sde_ldc_module.cpp
        subgraphs/sde_singlelayer/src/sde_singlelayer.cpp
        subgraphs/sde_multilayer/src/sde_multilayer.cpp
        subgraphs/sde_triangulate/src/sde_triangulate.cpp
        subgraphs/ss_sde_detection/src/ss_sde_detection.cpp
    )

    set(EDGEAI_COMMON_SRC
        ${TI_EDGEAI_ROOT_DIR}/common/src/edgeai_debug.cpp
    )

    set(ALL_CORE_SRCS
        ${COMMON_SRCS}
        ${VISION_CNN_CORE_SRCS}
        ${EDGEAI_COMMON_SRC}
    )

    if (${SDE_AVAILABLE})
        list(APPEND ALL_CORE_SRCS
            ${SDE_CORE_SRCS}
            ${SDE_SUBGRAPH_SRCS}
            ${ESTOP_CORE_SRCS}
        )
    endif()

    if (${TIADALG_AVAILABLE})
        list(APPEND ALL_CORE_SRCS
            ${VL_CORE_SRCS}
        )
    endif()

    # Build library
    build_lib(${PROJECT_NAME} ALL_CORE_SRCS STATIC)

    # Add the EdgeAI libraries
    set(TI_EDGEAI_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR})

    add_subdirectory(${TI_EDGEAI_ROOT_DIR}/utils ${TI_EDGEAI_BUILD_DIR}/utils)
    set_target_properties(edgeai_utils
                          PROPERTIES ARCHIVE_OUTPUT_DIRECTORY
                          ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})

endif()
