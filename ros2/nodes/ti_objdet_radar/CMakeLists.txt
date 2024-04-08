cmake_minimum_required(VERSION 3.5)
project(ti_objdet_radar)

include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/common_platform.cmake)

# Function for building a node:
# ARG0: app name
# ARG1: source list
function(build_node)
    set(app ${ARGV0})
    set(src ${ARGV1})

    add_executable(${app} ${${src}})

    target_link_libraries(
        ${app}
        -Wl,--start-group
        ${COMMON_LINK_LIBS}
        ${TARGET_LINK_LIBS}
        -Wl,--end-group
    )

    ament_target_dependencies(
        ${app}
        ${common_msgs_EXPORTED_TARGETS}
        rclcpp
        cv_bridge
        image_transport
        tf2_ros
        tf2_geometry_msgs
        sensor_msgs
        common_msgs
        ti_mmwave_rospkg_msgs
    )

    install(TARGETS ${app}
            DESTINATION lib/${PROJECT_NAME})

    # Install launch files, if needed
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
        install(DIRECTORY launch
                DESTINATION share/${PROJECT_NAME})
    endif()

    # Install rviz files, if needed
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
        install(DIRECTORY rviz
                DESTINATION share/${PROJECT_NAME})
    endif()

    # Install config files, if needed
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
        install(DIRECTORY config
                DESTINATION share/${PROJECT_NAME})
    endif()

endfunction(build_node)

if(${BUILD_CORE_NODES})

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(rcutils REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(image_transport REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(common_msgs REQUIRED)
    find_package(ti_mmwave_rospkg_msgs REQUIRED)
    find_package(OpenCV REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)

    include_directories(SYSTEM
        ${OpenCV_INCLUDE_DIRS}
    )

    # objdet_radar ROS node
    set(OBJDET_RADAR_SRCS src/objdet_radar_fusion.cpp)

    # Build the node
    build_node(objdet_radar_fusion OBJDET_RADAR_SRCS)

    ament_package()

endif()