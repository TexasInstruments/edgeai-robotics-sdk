include(${CMAKE_CURRENT_LIST_DIR}/common_platform.cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# Function for building a node:
# ARG0: app name
# ARG1: source list
function(build_node)
    set(app ${ARGV0})
    set(src ${ARGV1})

    add_executable(${app} ${${src}})

    target_link_libraries(${app}
                          -Wl,--start-group
                          ${COMMON_LINK_LIBS}
                          ${TARGET_LINK_LIBS}
                          -Wl,--end-group
                         )

    ament_target_dependencies(${app}
                              ${common_msgs_EXPORTED_TARGETS}
                              rclcpp
                              common_msgs
                              sensor_msgs
                              nav_msgs
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

# Function for building a lib:
# ARG0: app name
# ARG1: source list
function(build_lib)
    set(app ${ARGV0})
    set(src ${ARGV1})

    add_library(${app} SHARED ${${src}})

    target_link_libraries(${app}
                          -Wl,--start-group
                          ${COMMON_LINK_LIBS}
                          ${TARGET_LINK_LIBS}
                          -Wl,--end-group
                         )

    ament_target_dependencies(${app}
                              ${common_msgs_EXPORTED_TARGETS}
                              rclcpp
                              common_msgs
                              sensor_msgs
                              nav_msgs
                              cv_bridge
                              image_transport
                              )
    install(TARGETS ${app}
            DESTINATION lib/${PROJECT_NAME})

endfunction(build_lib)