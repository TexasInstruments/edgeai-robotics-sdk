cmake_minimum_required(VERSION 3.5)

project(zed_capture)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

# node
add_executable(${PROJECT_NAME}
    src/usb_stereo_camera.cpp
    src/zed_capture_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
                          rclcpp
                          cv_bridge
                          image_transport
                          camera_info_manager
                          OpenCV
                          )

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
    install(DIRECTORY launch
            DESTINATION share/${PROJECT_NAME})
else()
    message("No launch directory exists." ${CMAKE_CURRENT_SOURCE_DIR})
endif()


ament_package()
