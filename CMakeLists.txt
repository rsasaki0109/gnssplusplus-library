cmake_minimum_required(VERSION 3.14)
project(gnss_lib VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Find necessary packages
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Doxygen QUIET)
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(nav_msgs QUIET)

# List source files for the main library
set(GNSS_SOURCES
    src/gnss.cpp
    src/io/ntrip.cpp
    src/io/rinex.cpp
    src/io/rtcm.cpp
    src/io/rtcm_stream.cpp
    src/io/ubx.cpp
    src/core/solution.cpp
    src/core/types.cpp
    src/algorithms/ppp.cpp
)

# List source files for the no-optimization library
set(GNSS_SOURCES_NO_OPTIMIZATION
    src/algorithms/rtk.cpp
    src/algorithms/spp.cpp
    src/algorithms/lambda.cpp
    src/core/navigation.cpp
    src/core/observation.cpp
)

# Create the main library
add_library(gnss_lib STATIC ${GNSS_SOURCES})
target_include_directories(gnss_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(gnss_lib PUBLIC Eigen3::Eigen)

# Create the no-optimization library
add_library(gnss_lib_noopt STATIC ${GNSS_SOURCES_NO_OPTIMIZATION})
target_compile_options(gnss_lib_noopt PRIVATE -O0)
target_include_directories(gnss_lib_noopt PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(gnss_lib_noopt PUBLIC Eigen3::Eigen)

install(
    TARGETS gnss_lib gnss_lib_noopt
    EXPORT libgnssppTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/libgnssppConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/libgnssppConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libgnsspp
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/libgnssppConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(
    EXPORT libgnssppTargets
    FILE libgnssppTargets.cmake
    NAMESPACE libgnsspp::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libgnsspp
)
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/libgnssppConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/libgnssppConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libgnsspp
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/libgnsspp.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/libgnsspp.pc
    @ONLY
)
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/libgnsspp.pc
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)

if(DOXYGEN_FOUND)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Doxyfile.in
        ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        @ONLY
    )
    add_custom_target(
        doc_doxygen
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()

set(CPACK_PACKAGE_NAME "libgnsspp")
set(CPACK_PACKAGE_VENDOR "libgnss++")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "GNSS processing library and CLI toolchain")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE)
set(CPACK_GENERATOR "TGZ;DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "libgnss++ maintainers")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
include(CPack)

# Add subdirectories
add_subdirectory(examples)
add_subdirectory(apps)
add_subdirectory(python)
if(ament_cmake_FOUND AND rclcpp_FOUND AND sensor_msgs_FOUND AND geometry_msgs_FOUND AND nav_msgs_FOUND)
    set(GNSSPP_HAS_ROS2 ON)
    add_subdirectory(ros2)
else()
    set(GNSSPP_HAS_ROS2 OFF)
    message(STATUS "ROS2 dependencies not found; skipping ros2/ node build")
endif()
add_subdirectory(tests)
