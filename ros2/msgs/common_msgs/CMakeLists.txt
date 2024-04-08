cmake_minimum_required(VERSION 3.5)
project(common_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    msg/Detection3D.msg
    msg/Disparity.msg
    msg/DriveSpace.msg
    msg/ObjectPos3D.msg
    msg/ObjectPos.msg
    msg/StereoDetection.msg
    msg/BoundingBox2D.msg
    msg/Detection2D.msg
    msg/BoundingBox2Dp.msg
    msg/BoundingBox2Dpva.msg
    msg/Detection2Dp.msg
    msg/Detection2Dpva.msg
    msg/Transform.msg
    msg/Pose6D.msg
    msg/HumanPose.msg
    DEPENDENCIES
      std_msgs
      geometry_msgs
)

ament_package()
