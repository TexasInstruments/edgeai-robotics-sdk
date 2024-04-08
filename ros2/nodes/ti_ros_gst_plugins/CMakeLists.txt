cmake_minimum_required(VERSION 3.5)
project(ti_ros_gst_plugins)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/ros2_common.cmake)

if(${BUILD_CORE_NODES})
    find_package(builtin_interfaces REQUIRED)

    set(APP_ROS_GST_PLUGINS
        src/gsttiros.cpp
        src/gsttirosutils.cpp
        src/gsttirosimgsrc.cpp
        src/gsttirossink.cpp)

    include_directories(
        ${rclcpp_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${common_msgs_INCLUDE_DIRS}
        SYSTEM ${TARGET_FS}/usr/local/include
        SYSTEM ${TARGET_FS}/usr/include/gstreamer-1.0
        SYSTEM ${TARGET_FS}/usr/include/glib-2.0
        SYSTEM ${TARGET_FS}/usr/lib/glib-2.0/include
        SYSTEM ${TARGET_FS}/usr/lib/aarch64-linux-gnu/glib-2.0/include
        SYSTEM ${TARGET_FS}/usr/lib/x86_64-linux-gnu/glib-2.0/include/
        SYSTEM ${TARGET_FS}/usr/include/edgeai-apps-utils
    )

    set(TARGET_LINK_LIBS
        ${TARGET_LINK_LIBS}
        ${rclcpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${common_msgs_LIBRARIES}
        gstreamer-1.0
        glib-2.0
        gobject-2.0
        gstapp-1.0
        gstbase-1.0
        gstvideo-1.0
    )

    if(${BUILD_EMULATION_MODE} STREQUAL no)
        add_definitions (-DBUILD_FOR_TARGET)
        set(TARGET_LINK_LIBS ${TARGET_LINK_LIBS} edgeai-apps-utils)
    endif()

    build_lib(ros_gst_plugins APP_ROS_GST_PLUGINS)

    ament_package(
        CONFIG_EXTRAS
    )
endif()