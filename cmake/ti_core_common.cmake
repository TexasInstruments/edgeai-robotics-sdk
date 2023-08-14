include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

include(${CMAKE_CURRENT_LIST_DIR}/common_platform.cmake)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${TI_CORE_ROOT_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TI_CORE_ROOT_DIR}/bin/${CMAKE_BUILD_TYPE})

#message("TI_CORE::PROJECT_NAME             = " ${PROJECT_NAME})
#message("TI_CORE::CMAKE_PROJECT_NAME       = " ${CMAKE_PROJECT_NAME})
#message("TI_CORE::CMAKE_SYSTEM_PROCESSOR   = " ${CMAKE_SYSTEM_PROCESSOR})
#message("TI_CORE::PROJECT_SOURCE_DIR       = " ${PROJECT_SOURCE_DIR})
#message("TI_CORE::CMAKE_SOURCE_DIR         = " ${CMAKE_SOURCE_DIR})
#message("TI_CORE::CMAKE_CURRENT_SOURCE_DIR = " ${CMAKE_CURRENT_SOURCE_DIR})
#message("TI_CORE::CMAKE_INSTALL_LIBDIR     = " ${CMAKE_INSTALL_LIBDIR})
#message("TI_CORE::CMAKE_INSTALL_BINDIR     = " ${CMAKE_INSTALL_BINDIR})
#message("TI_CORE::CMAKE_INSTALL_INCLUDEDIR = " ${CMAKE_INSTALL_INCLUDEDIR})
#message("TI_CORE::CMAKE_CURRENT_BINARY_DIR = " ${CMAKE_CURRENT_BINARY_DIR})
#message("TI_CORE::CMAKE_INSTALL_PREFIX     = " ${CMAKE_INSTALL_PREFIX})

# Function for building an executable:
# ARG0: app name
# ARG1: source list
function(build_exe)
    set(app ${ARGV0})
    set(src ${ARGV1})
    add_executable(${app} ${${src}})
    target_link_libraries(${app}
                          -Wl,--start-group
                          ${COMMON_LINK_LIBS}
                          ${TARGET_LINK_LIBS}
                          ${SYSTEM_LINK_LIBS}
                          -Wl,--end-group)
endfunction(build_exe)

# Function for building a node:
# ARG0: lib name
# ARG1: source list
# ARG2: type (STATIC, SHARED)
function(build_lib)
    set(lib ${ARGV0})
    set(src ${ARGV1})
    set(type ${ARGV2})
    set(version 1.0.0)

    add_library(${lib} ${type} ${${src}})

    set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/${CMAKE_INSTALL_INCLUDEDIR})

    include_directories(
        ${EIGEN3_INCLUDE_DIR}
    )

    install(TARGETS ${lib}
            EXPORT ${lib}Targets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # Shared Libs
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # Static Libs
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # Executables, DLLs
            INCLUDES DESTINATION ${CMAKE_INSTALL_LIBDIR}/${CMAKE_INSTALL_INCLUDEDIR}
    )

    FILE(GLOB HDRS
        ${TI_CORE_ROOT_DIR}/*/include/*.h
    )

    # Specify the header files to install
    install(FILES
        ${HDRS}
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/${CMAKE_INSTALL_INCLUDEDIR}
    )

    set(CONFIG_PACKAGE_LOCATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

    install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        DESTINATION ${CONFIG_PACKAGE_LOCATION}
    )

    export(EXPORT ${PROJECT_NAME}Targets
        FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
    )

    # Generate the version file for the config file
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        VERSION "${version}"
        COMPATIBILITY AnyNewerVersion
    )

    # Create config file
    configure_package_config_file(${TI_ROS_ROOT_DIR}/cmake/ti_core_config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/${lib}Config.cmake
        INSTALL_DESTINATION ${CONFIG_PACKAGE_LOCATION}
        PATH_VARS INCLUDE_INSTALL_DIR
    )

    # install the package configuration files
    install(FILES
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
      DESTINATION ${CONFIG_PACKAGE_LOCATION}
    )

endfunction(build_lib)

