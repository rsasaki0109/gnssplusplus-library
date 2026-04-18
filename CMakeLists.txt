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
find_package(std_msgs QUIET)
find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs)

option(CLASLIB_PARITY_LINK "Link the external CLASLIB C sources for explicit parity bridge runs" OFF)
set(CLASLIB_ROOT_DIR
    ""
    CACHE PATH "Path to the CLASLIB checkout root")

# List source files for the main library
set(GNSS_SOURCES
    src/gnss.cpp
    src/io/ntrip.cpp
    src/io/rinex.cpp
    src/io/rtcm.cpp
    src/io/rtcm_stream.cpp
    src/io/ubx.cpp
    src/io/qzss_l6.cpp
    src/core/solution.cpp
    src/core/types.cpp
    src/algorithms/ppp_ar.cpp
    src/algorithms/ppp_atmosphere.cpp
    src/algorithms/ppp_clas.cpp
    src/algorithms/ppp_claslib_pntpos.cpp
    src/algorithms/ppp_claslib_zdres.cpp
    src/algorithms/ppp_claslib_full.cpp
    src/algorithms/ppp_clasnat.cpp
    src/algorithms/ppp_clasnat_osr.cpp
    src/algorithms/ppp_clasnat_zdres.cpp
    src/algorithms/ppp_clasnat_trop.cpp
    src/algorithms/ppp_clas_sd.cpp
    src/algorithms/ppp_osr.cpp
    src/algorithms/ppp_wlnl.cpp
    src/algorithms/ppp_clas_epoch.cpp
    src/algorithms/clasnat_parity.cpp
    src/algorithms/claslib_oracle.cpp
    src/algorithms/ppp_claslib_bridge.cpp
    src/algorithms/ppp.cpp
    src/algorithms/tidal.cpp
)

# List source files for the no-optimization library
set(GNSS_SOURCES_NO_OPTIMIZATION
    src/algorithms/rtk_ar_selection.cpp
    src/algorithms/rtk_ar_evaluation.cpp
    src/algorithms/rtk_measurement.cpp
    src/algorithms/rtk_selection.cpp
    src/algorithms/rtk_update.cpp
    src/algorithms/rtk_validation.cpp
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

if(CLASLIB_PARITY_LINK)
    set(CLASLIB_SRC_DIR "${CLASLIB_ROOT_DIR}/src")
    set(CLASLIB_DATA_DIR "${CLASLIB_ROOT_DIR}/data")
    if(NOT EXISTS "${CLASLIB_SRC_DIR}/rtklib.h")
        message(FATAL_ERROR "CLASLIB source tree not found: ${CLASLIB_SRC_DIR}")
    endif()
    find_package(BLAS REQUIRED)
    find_package(LAPACK REQUIRED)

    set(CLASLIB_SRCS
        ${CLASLIB_SRC_DIR}/rtkcmn.c
        ${CLASLIB_SRC_DIR}/rinex.c
        ${CLASLIB_SRC_DIR}/rtkpos.c
        ${CLASLIB_SRC_DIR}/postpos.c
        ${CLASLIB_SRC_DIR}/solution.c
        ${CLASLIB_SRC_DIR}/lambda.c
        ${CLASLIB_SRC_DIR}/geoid.c
        ${CLASLIB_SRC_DIR}/sbas.c
        ${CLASLIB_SRC_DIR}/preceph.c
        ${CLASLIB_SRC_DIR}/pntpos.c
        ${CLASLIB_SRC_DIR}/ephemeris.c
        ${CLASLIB_SRC_DIR}/options.c
        ${CLASLIB_SRC_DIR}/ppp.c
        ${CLASLIB_SRC_DIR}/ppp_ar.c
        ${CLASLIB_SRC_DIR}/rtcm.c
        ${CLASLIB_SRC_DIR}/rtcm2.c
        ${CLASLIB_SRC_DIR}/rtcm3.c
        ${CLASLIB_SRC_DIR}/rtcm3e.c
        ${CLASLIB_SRC_DIR}/ionex.c
        ${CLASLIB_SRC_DIR}/qzslex.c
        ${CLASLIB_SRC_DIR}/rcvraw.c
        ${CLASLIB_SRC_DIR}/stec.c
        ${CLASLIB_SRC_DIR}/isb.c
        ${CLASLIB_SRC_DIR}/grid.c
        ${CLASLIB_SRC_DIR}/cssr.c
        ${CLASLIB_SRC_DIR}/ppprtk.c
        ${CLASLIB_SRC_DIR}/cssr2osr.c
        ${CLASLIB_SRC_DIR}/rtkvrs.c
        ${CLASLIB_SRC_DIR}/rcv/binex.c
        ${CLASLIB_SRC_DIR}/rcv/novatel.c
        ${CLASLIB_SRC_DIR}/rcv/ublox.c
    )
    add_library(claslib STATIC ${CLASLIB_SRCS})
    target_include_directories(claslib PUBLIC ${CLASLIB_SRC_DIR})
    target_compile_definitions(claslib
        PRIVATE
            TRACE
            ENAGAL
            ENAQZS
            NFREQ=3
            ENA_PPP_RTK
            ENA_REL_VRS
            LAPACK
    )
    target_link_libraries(claslib PUBLIC ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} m Threads::Threads)

    target_include_directories(gnss_lib PRIVATE ${CLASLIB_SRC_DIR})
    target_compile_definitions(gnss_lib
        PUBLIC GNSSPP_HAS_CLASLIB_BRIDGE=1
        PRIVATE
            TRACE
            ENAGAL
            ENAQZS
            NFREQ=3
            ENA_PPP_RTK
            ENA_REL_VRS
            LAPACK
            "GNSSPP_CLASLIB_ROOT=\"${CLASLIB_ROOT_DIR}\""
            "GNSSPP_CLASLIB_DATA_DIR=\"${CLASLIB_DATA_DIR}\""
    )
    target_link_libraries(gnss_lib PUBLIC claslib)
else()
    target_compile_definitions(gnss_lib PUBLIC GNSSPP_HAS_CLASLIB_BRIDGE=0)
endif()

# Create the no-optimization library
add_library(gnss_lib_noopt STATIC ${GNSS_SOURCES_NO_OPTIMIZATION})
target_compile_options(gnss_lib_noopt PRIVATE -O0)
target_include_directories(gnss_lib_noopt PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(gnss_lib_noopt PUBLIC Eigen3::Eigen)

set(GNSS_INSTALL_TARGETS gnss_lib gnss_lib_noopt)
if(CLASLIB_PARITY_LINK)
    list(APPEND GNSS_INSTALL_TARGETS claslib)
endif()
install(
    TARGETS ${GNSS_INSTALL_TARGETS}
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

# Visibility library (fisheye camera, satellite LOS/NLOS)
if(OpenCV_FOUND)
    add_library(gnss_visibility_lib STATIC
        src/visibility/fisheye_camera.cpp
        src/visibility/proj_fisheye.cpp
        src/visibility/im2fisheye.cpp
        src/visibility/mask_fisheye.cpp
        src/visibility/kml_tour.cpp
        src/visibility/satellite_visibility.cpp
    )
    target_include_directories(gnss_visibility_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
    target_link_libraries(gnss_visibility_lib PUBLIC ${OpenCV_LIBS})
else()
    message(STATUS "OpenCV not found; skipping visibility library")
endif()

# KML library (Google Earth KML/KMZ generation)
add_library(gnss_kml_lib STATIC
    src/kml/types.cpp src/kml/utils.cpp src/kml/elements.cpp
    src/kml/animation.cpp src/kml/camera.cpp src/kml/tour.cpp src/kml/writer.cpp
)
target_include_directories(gnss_kml_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Add subdirectories
add_subdirectory(examples)
add_subdirectory(apps)
add_subdirectory(python)
if(ament_cmake_FOUND AND rclcpp_FOUND AND sensor_msgs_FOUND AND geometry_msgs_FOUND AND nav_msgs_FOUND AND std_msgs_FOUND)
    set(GNSSPP_HAS_ROS2 ON)
    add_subdirectory(ros2)
else()
    set(GNSSPP_HAS_ROS2 OFF)
    message(STATUS "ROS2 dependencies not found; skipping ros2/ node build")
endif()
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
