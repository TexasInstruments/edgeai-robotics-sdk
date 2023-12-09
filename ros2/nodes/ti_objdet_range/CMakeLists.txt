cmake_minimum_required(VERSION 3.5)
project(ti_objdet_range)

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
        common_msgs
        sensor_msgs
        cv_bridge
        image_transport
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

if(${BUILD_CORE_NODES} AND ${SDE_AVAILABLE})

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(rcutils REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(image_transport REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(common_msgs REQUIRED)
    find_package(OpenCV REQUIRED)
    find_package(PCL REQUIRED)
    find_package(pcl_msgs REQUIRED)
    find_package(pcl_conversions REQUIRED)

    include_directories(SYSTEM
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${pcl_msgs_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDE_DIRS}
    )

    # objdet_range ROS node
    set(OBJDET_RANGE_SRCS src/objdet_disparity_fusion.cpp)

    # Build the node
    build_node(objdet_disparity_fusion OBJDET_RANGE_SRCS)

    ament_package()

endif()