cmake_minimum_required(VERSION 3.5)
project(ti_vl)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/ros2_common.cmake)

if(${BUILD_CORE_NODES} AND ${TIADALG_AVAILABLE})
    find_package(Eigen3 3.1.0 REQUIRED)

    include_directories(${EIGEN3_INCLUDE_DIR})

    # Visual localization example
    set(APP_VISLOC_SRCS
        src/visloc_main.cpp
        src/visloc_node.cpp)

    # Build nodes
    build_node(vl APP_VISLOC_SRCS)

    ament_package()
endif()
