# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

add_compile_options(-std=c++17)

SET(CMAKE_FIND_LIBRARY_PREFIXES "" "lib")
SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a" ".lib" ".so")

# Get the path of this file
get_filename_component(SELF_DIR        "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(TI_ROS_ROOT_DIR "${SELF_DIR}/.." ABSOLUTE)

# From the self path, derive and set the ti_core directory
get_filename_component(TI_CORE_ROOT_DIR   "${TI_ROS_ROOT_DIR}/ti_core" ABSOLUTE)
get_filename_component(TI_EDGEAI_ROOT_DIR "/opt/edgeai-gst-apps/apps_cpp" ABSOLUTE)
get_filename_component(INSTALL_DIR        ${CMAKE_INSTALL_PREFIX}/.. ABSOLUTE)

# PSDKRA base folder location
set(PSDK_DIR $ENV{PSDK_BASE_PATH})

# Set PROFLE based on CMAKE_BUILD_TYPE which is passed from command line,
# e.g., -DCMAKE_BUILD_TYPE=Debug
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if ((${CMAKE_BUILD_TYPE} STREQUAL Debug) OR
    (${CMAKE_BUILD_TYPE} STREQUAL RelWithDebInfo))
    set(PROFILE debug)
else()
    set(PROFILE release)
endif()

message(STATUS "PROFILE = ${PROFILE}")

set(TI_EXTERNAL_INCLUDE_DIRS
    ${TI_CORE_ROOT_DIR}/common/include
    ${TI_CORE_ROOT_DIR}/utils/include
    ${TI_CORE_ROOT_DIR}/ti_vision_cnn/include
    ${TI_CORE_ROOT_DIR}/ti_sde/include
    ${TI_CORE_ROOT_DIR}/ti_estop/include
    ${TI_CORE_ROOT_DIR}/ti_vl/include
    ${TI_CORE_ROOT_DIR}/subgraphs/sde_ldc/include
    ${TI_CORE_ROOT_DIR}/subgraphs/sde_singlelayer/include
    ${TI_CORE_ROOT_DIR}/subgraphs/sde_multilayer/include
    ${TI_CORE_ROOT_DIR}/subgraphs/sde_triangulate/include
    ${TI_CORE_ROOT_DIR}/subgraphs/ss_sde_detection/include
    ${TI_EDGEAI_ROOT_DIR}
   )

if (NOT "$ENV{MMALIB_PACKAGE_ROOT}" STREQUAL "")
    set(MMALIB_PACKAGE_ROOT "$ENV{MMALIB_PACKAGE_ROOT}"
        CACHE INTERNAL
        "Copied MMALIB_PACKAGE_ROOT from environment variable")
else()
    set(MMALIB_PACKAGE_ROOT mmalib_02_01_00_07)
endif()

if (NOT "$ENV{TIDL_PACKAGE_ROOT}" STREQUAL "")
    set(TIDL_PACKAGE_ROOT "$ENV{TIDL_PACKAGE_ROOT}/ti_dl"
        CACHE INTERNAL
        "Copied TIDL_PACKAGE_ROOT from environment variable")
else()
    # tidl_j7 -> tidl_j7_<version> should be extablished in the root filesystem
    set(TIDL_PACKAGE_ROOT tidl_j7/arm-tidl/rt)
endif()

set(CGT7X_ROOT          ti-cgt-c7000_1.4.2.LTS)
set(PDK_PACKAGE_ROOT    pdk/packages/ti)
set(TIADALG_PATH        tiadalg)

set(TENSORFLOW_INSTALL_DIR /usr/include/tensorflow)
set(ONNXRT_INSTALL_DIR     /usr/include/onnxruntime)
set(TFLITE_INSTALL_DIR     /usr/lib/tflite_2.8)

set(TARGET_SOC_LOWER $ENV{SOC})

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    set(BUILD_CORE_NODES          OFF)
    set(BUILD_VISUALIZATION_NODES ON CACHE BOOL "Build Visualization nodes")
    set(TARGET_PLATFORM           PC)
    set(TARGET_CPU                x86_64)
    set(BUILD_EMULATION_MODE      yes)
    set(TIVISION_APPS_TYPE        static)
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    set(BUILD_CORE_NODES          ON CACHE BOOL "Build core nodes")
    set(BUILD_VISUALIZATION_NODES OFF CACHE BOOL "Do not build Visualization nodes")
    set(BUILD_EMULATION_MODE      no)
    set(TIVISION_APPS_TYPE        shared)

    if ("${TARGET_SOC_LOWER}" STREQUAL "j721e")
        set(TARGET_PLATFORM     J7)
        set(TARGET_CPU          A72)
        set(TARGET_OS           LINUX)
        set(TARGET_SOC          J721E)
    elseif ("${TARGET_SOC_LOWER}" STREQUAL "j721s2")
        set(TARGET_PLATFORM     J7)
        set(TARGET_CPU          A72)
        set(TARGET_OS           LINUX)
        set(TARGET_SOC          J721S2)
    elseif ("${TARGET_SOC_LOWER}" STREQUAL "j784s4")
        set(TARGET_PLATFORM     J7)
        set(TARGET_CPU          A72)
        set(TARGET_OS           LINUX)
        set(TARGET_SOC          J784S4)
    elseif ("${TARGET_SOC_LOWER}" STREQUAL "am62a")
        set(TARGET_PLATFORM     SITARA)
        set(TARGET_CPU          A53)
        set(TARGET_OS           LINUX)
        set(TARGET_SOC          AM62A)
    elseif ("${TARGET_SOC_LOWER}" STREQUAL "am62")
        set(TARGET_PLATFORM     SITARA)
        set(TARGET_CPU          A53)
        set(TARGET_OS           LINUX)
        set(TARGET_SOC          AM62)
    else()
        message(FATAL_ERROR "SOC ${TARGET_SOC_LOWER} is not supported.")
    endif()

    # message(STATUS "SOC=${TARGET_SOC_LOWER}")
else()
    message(FATAL_ERROR "Unknown processor:" ${CMAKE_SYSTEM_PROCESSOR})
endif()

set(TENSORFLOW_RT_LIBS
    tensorflow-lite
    flatbuffers
    fft2d_fftsg2d
    fft2d_fftsg
    cpuinfo
    clog
    farmhash
    ruy_allocator
    ruy_apply_multiplier
    ruy_blocking_counter
    ruy_block_map
    ruy_context
    ruy_context_get_ctx
    ruy_cpuinfo
    ruy_ctx
    ruy_denormal
    ruy_frontend
    ruy_have_built_path_for_avx2_fma
    ruy_have_built_path_for_avx512
    ruy_have_built_path_for_avx
    ruy_kernel_arm
    ruy_kernel_avx2_fma
    ruy_kernel_avx512
    ruy_kernel_avx
    ruy_pack_arm
    ruy_pack_avx2_fma
    ruy_pack_avx512
    ruy_pack_avx
    ruy_prepacked_cache
    ruy_prepare_packed_matrices
    ruy_system_aligned_alloc
    ruy_thread_pool
    ruy_trmul
    ruy_tune
    ruy_wait
    pthreadpool
    #xnn lib
    XNNPACK
)

if (${BUILD_CORE_NODES})
    set(TI_EXTERNAL_LIBS
        ti_core
        edgeai_utils
        ncurses
        dlr
        onnxruntime
        pthread
        dl
        yaml-cpp
        ${TENSORFLOW_RT_LIBS}
       )
else()
    set(TI_EXTERNAL_LIBS "")
endif()

set(TARGET_OS LINUX)

# pass the macros
add_definitions(
    -DTARGET_CPU=${TARGET_CPU}
    -DTARGET_OS=${TARGET_OS}
    -DPROFILE=${PROFILE}
    -DBUILD_EMULATION_MODE=${BUILD_EMULATION_MODE}
    -DSOC_${TARGET_SOC}
)

# TIOVX, VISION_APPS, PTK_DEMOS: include folders
set(TIOVX_INCLUDE_DIRS
    ${PSDK_DIR}/tiovx/include
    ${PSDK_DIR}/imaging/kernels/include
    ${PSDK_DIR}/tidl_j7/arm-tidl/tiovx_kernels/include
    ${PSDK_DIR}/tiovx/utils/include/
    ${PSDK_DIR}/tiovx/kernels/include/
)

set(PTK_INCLUDE_DIRS
    ${PSDK_DIR}/ti-perception-toolkit/include
)

set(MMALIB_INCLUDE_DIRS
    ${PSDK_DIR}/${MMALIB_PACKAGE_ROOT}/include
)

set(TIDL_INCLUDE_DIRS
    ${PSDK_DIR}/${TIDL_PACKAGE_ROOT}/inc
)

set(TIADALG_INCLUDE_DIRS
    ${PSDK_DIR}/${TIADALG_PATH}/include
)

set(VISION_APP_INCLUDE_DIRS
    ${PSDK_DIR}/vision_apps
    ${PSDK_DIR}/imaging
    ${PSDK_DIR}/vision_apps/kernels/img_proc/include
    ${PSDK_DIR}/vision_apps/kernels/img_proc/host
    ${PSDK_DIR}/vision_apps/kernels/stereo/include
    src
)

# should add more directories in ${PSDK_DIR}/vision_apps/kernels
set(VISION_APP_KERNEL_INCLUDE_DIRS
    ${PSDK_DIR}/vision_apps/kernels/img_proc/include
    ${PSDK_DIR}/vision_apps/kernels/stereo/include
)

set(OTHER_INCLUDE_DIRS
    ${PSDK_DIR}/${CGT7X_ROOT}/host_emulation/include/C7100
    ${PSDK_DIR}/j7_c_models/include
    ${PSDK_DIR}/ivision
    ${PSDK_DIR}/app_utils
)

include_directories(
    ${TI_EXTERNAL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${TIOVX_INCLUDE_DIRS}
    ${PTK_INCLUDE_DIRS}
    ${VISION_APP_INCLUDE_DIRS}
    ${VISION_APP_KERNEL_INCLUDE_DIRS}
    ${MMALIB_INCLUDE_DIRS}
    ${TIDL_INCLUDE_DIRS}
    ${TIADALG_INCLUDE_DIRS}
    ${OTHER_INCLUDE_DIRS}
    /usr/include/edgeai_dl_inferer
    /usr/include/opencv4
)

set(COMMON_LINK_LIBS
    ${TI_EXTERNAL_LIBS}
    )

set(TI_EXTERNAL_LIB_DIRS
    ${INSTALL_DIR}/ti_external/lib
   )

set(TIADALG_AVAILABLE FALSE)
set(SDE_AVAILABLE FALSE)

# BUILD and LINK
if (NOT ${TARGET_PLATFORM} STREQUAL PC)
    if (${TARGET_PLATFORM} STREQUAL SITARA)
        set(SDE_AVAILABLE FALSE)
        set(TIADALG_AVAILABLE FALSE)
    else()
        set(SDE_AVAILABLE TRUE)
        set(TIADALG_AVAILABLE TRUE)
    endif()

    set(TARGET_LINK_DIRECTORIES
        ${TI_EXTERNAL_LIB_DIRS}
        /usr/local/dlr
        /usr/lib
        ${TENSORFLOW_INSTALL_DIR}/tensorflow/lite/tools/make/gen/linux_aarch64/lib
        ${TFLITE_INSTALL_DIR}/ruy-build
        ${TFLITE_INSTALL_DIR}/xnnpack-build
        ${TFLITE_INSTALL_DIR}/pthreadpool
        ${TFLITE_INSTALL_DIR}/fft2d-build
        ${TFLITE_INSTALL_DIR}/cpuinfo-build
        ${TFLITE_INSTALL_DIR}/flatbuffers-build
        ${TFLITE_INSTALL_DIR}/clog-build
        ${TFLITE_INSTALL_DIR}/farmhash-build
        )

    set(TARGET_LINK_LIBS
        edgeai_dl_inferer
        edgeai_pre_process
        edgeai_post_process
        dlr)

    if(NOT ${TARGET_SOC} STREQUAL "AM62")
        set(TARGET_LINK_LIBS ${TARGET_LINK_LIBS} tivision_apps)
    endif()

endif()

link_directories(${TARGET_LINK_DIRECTORIES})

function(print_vars)
    message("CMAKE_SYSTEM_PROCESSOR   =" ${CMAKE_SYSTEM_PROCESSOR})
    message("PROJECT_SOURCE_DIR       =" ${PROJECT_SOURCE_DIR})
    message("CMAKE_SOURCE_DIR         =" ${CMAKE_SOURCE_DIR})
    message("CMAKE_CURRENT_SOURCE_DIR =" ${CMAKE_CURRENT_SOURCE_DIR})
    message("CMAKE_INSTALL_PREFIX     =" ${CMAKE_INSTALL_PREFIX})
    message("CMAKE_CURRENT_BINARY_DIR =" ${CMAKE_CURRENT_BINARY_DIR})
    message("INSTALL_DIR              =" ${INSTALL_DIR})
    message("MMALIB_PACKAGE_ROOT      =" ${MMALIB_PACKAGE_ROOT})
    message("TIDL_PACKAGE_ROOT        =" ${TIDL_PACKAGE_ROOT})
    message("TI_CORE_ROOT_DIR         =" ${TI_CORE_ROOT_DIR})
    message("TI_EDGEAI_ROOT_DIR       =" ${TI_EDGEAI_ROOT_DIR})

endfunction(print_vars)

