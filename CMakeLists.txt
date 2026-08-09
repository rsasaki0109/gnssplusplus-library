cmake_minimum_required(VERSION 3.14)
project(gnss_lib VERSION 0.1.0 LANGUAGES C CXX)

set(GNSSPP_CXX_STANDARD 20 CACHE STRING
    "C++ standard (use 17 with legacy Eigen 3.3 toolchains that reject C++20 rewritten comparisons)")
set(CMAKE_CXX_STANDARD ${GNSSPP_CXX_STANDARD})
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(MSVC OR (WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
    # MSVC portability: expose M_PI etc. from <cmath>, keep <windows.h>
    # from defining min/max macros that clash with std::min/std::max.
    # Also needed for Clang targeting the MSVC ABI/headers (e.g. the
    # build/clang-ninja config): CMAKE_CXX_COMPILER_ID is "Clang" there
    # (MSVC is false), but the toolchain still resolves <cmath>/<math.h>
    # to the MSVC STL headers, which gate M_PI on this same macro.
    add_compile_definitions(_USE_MATH_DEFINES NOMINMAX)
endif()
include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Find necessary packages
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Doxygen QUIET)
option(GNSSPP_BUILD_PYTHON_BINDINGS "Build the optional pybind11 Python module" ON)
if(GNSSPP_BUILD_PYTHON_BINDINGS)
    find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
    find_package(pybind11 CONFIG REQUIRED)
endif()
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(nav_msgs QUIET)
find_package(std_msgs QUIET)
find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs)
find_package(GTSAM QUIET CONFIG PATHS /usr/local/lib/cmake/GTSAM)
if(GTSAM_FOUND)
    message(STATUS "GTSAM found; enabling optional FGO GTSAM backend")
else()
    message(STATUS "GTSAM not found; optional FGO GTSAM backend disabled")
endif()
# GNSSPP_HAS_GTSAM (compile definition, exported PUBLIC below) gates all
# GTSAM-dependent code (see src/algorithms/fgo_gtsam_backend.cpp and the
# FGOProcessor::optimizeProblem backend selector in src/algorithms/fgo.cpp).
# A build without GTSAM_FOUND is unchanged: FGOBackend::GTSAM silently falls
# back to the native Eigen backend.
option(GNSSPP_USE_CHOLMOD "Use CHOLMOD for optional FGO sparse covariance solves" ON)
if(GNSSPP_USE_CHOLMOD)
    find_path(CHOLMOD_INCLUDE_DIR
        NAMES cholmod.h
        PATH_SUFFIXES suitesparse)
    find_library(CHOLMOD_LIBRARY NAMES cholmod)
    if(CHOLMOD_INCLUDE_DIR AND CHOLMOD_LIBRARY)
        message(STATUS "CHOLMOD found; enabling optional FGO covariance solver")
    else()
        message(STATUS "CHOLMOD not found; using Eigen sparse covariance solver")
    endif()
else()
    message(STATUS "CHOLMOD disabled; using Eigen sparse covariance solver")
endif()
option(MADOCALIB_PARITY_LINK "Link external MADOCALIB C sources for opt-in parity oracle tests" OFF)
option(
    MADOCALIB_PARITY_ORACLE
    "Enable the full MADOCALIB C++ oracle wrapper (disable for bridge-only parity runs)"
    ON)
set(MADOCALIB_ROOT_DIR
    ""
    CACHE PATH "Path to the MADOCALIB checkout root")

# Vendored third-party libraries.
# These are built as STATIC sub-targets and consumed by gnss_lib through
# the libgnss++/iers/ wrapper API (see include/libgnss++/iers/).
add_subdirectory(third_party/sofa)
add_subdirectory(third_party/ginan-iers2010)

# List source files for the main library
set(GNSS_SOURCES
    src/gnss.cpp
    src/io/imu.cpp
    src/io/ntrip.cpp
    src/io/rinex.cpp
    src/io/rinex4.cpp
    src/io/rtcm.cpp
    src/io/rtcm_stream.cpp
    src/io/ubx.cpp
    src/io/qzss_l6.cpp
    src/io/madoca_l6.cpp
    src/io/madoca_l6d.cpp
    src/core/solution.cpp
    src/core/types.cpp
    src/fusion/attitude.cpp
    src/fusion/mechanization.cpp
    src/fusion/preintegration.cpp
    src/fusion/tight_coupling_processor.cpp
    src/fusion/fusion_process_noise.cpp
    src/fusion/fusion_measurement.cpp
    src/fusion/fusion_update.cpp
    src/fusion/fusion_initialization.cpp
    src/fusion/fusion_processor.cpp
    src/fusion/dd_imu_bridge.cpp
    src/algorithms/ppp_ar.cpp
    src/algorithms/ppp_atmosphere.cpp
    src/algorithms/ppp_clas.cpp
    src/algorithms/ppp_clas_diagnostics.cpp
    src/algorithms/ppp_clas_dd.cpp
    src/algorithms/ppp_clas_sd.cpp
    src/algorithms/ppp_osr.cpp
    src/algorithms/ppp_wlnl.cpp
    src/algorithms/ppp_clas_epoch.cpp
    src/algorithms/ppp_env_overrides.cpp
    src/algorithms/madoca_core.cpp
    src/algorithms/madoca_parity.cpp
    src/algorithms/madocalib_oracle.cpp
    src/algorithms/madocalib_bridge.cpp
    src/algorithms/ppp.cpp
    src/algorithms/ppp_ambiguity.cpp
    src/algorithms/ppp_corrections.cpp
    src/algorithms/ppp_measurement.cpp
    src/algorithms/ppp_observations.cpp
    src/algorithms/ppp_products.cpp
    src/algorithms/ppp_state.cpp
    src/iers/earth_rotation.cpp
    src/iers/tides.cpp
    src/iers/ephemeris.cpp
    src/iers/eop_table.cpp
    src/iers/sub_daily_eop.cpp
)

# RTK/SPP/LAMBDA sources are shared by the no-optimization support archive and
# the optimized runtime archive. Keep this list canonical so the two targets
# cannot silently drift apart when a solver source is added or removed.
set(GNSS_RTK_RUNTIME_SOURCES
    src/algorithms/rtk_ar_selection.cpp
    src/algorithms/rtk_ar_evaluation.cpp
    src/algorithms/nlos_weights.cpp
    src/algorithms/float_trust_policy.cpp
    src/algorithms/rtk_adaptive_noise.cpp
    src/algorithms/rtk_measurement.cpp
    src/algorithms/rtk_selection.cpp
    src/algorithms/rtk_cmc_reference.cpp
    src/algorithms/rtk_cp_pr_gate.cpp
    src/algorithms/rtk_ddpr_anchor.cpp
    src/algorithms/rtk_update.cpp
    src/algorithms/rtk_ins_time_update.cpp
    src/algorithms/rtk_tdcp_diagnostics.cpp
    src/algorithms/rtk_validation.cpp
    src/algorithms/integrity_consensus.cpp
    src/algorithms/rtk.cpp
    src/algorithms/spp.cpp
    src/algorithms/spp_velocity.cpp
    src/algorithms/lambda.cpp
    src/core/navigation.cpp
    src/core/observation.cpp
)

# List source files for the no-optimization library. The FGO-only sources stay
# here because tests and the FGO applications intentionally use this archive.
set(GNSS_SOURCES_NO_OPTIMIZATION
    ${GNSS_RTK_RUNTIME_SOURCES}
    src/algorithms/fgo_ddpr_gnc.cpp
    src/algorithms/fgo.cpp
    src/algorithms/ppp_env_overrides.cpp
)

# Production RTK applications must not inherit the historical -O0 workaround
# used by the monolithic FGO/test support archive.  Keep FGO in
# gnss_lib_noopt, while compiling the online RTK/SPP/LAMBDA path as its own
# optimized archive so the documented per-epoch runtime budget is meaningful.
# Create the main library
add_library(gnss_lib STATIC ${GNSS_SOURCES})
target_include_directories(gnss_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(gnss_lib PUBLIC Eigen3::Eigen)
# IERS wrappers: SOFA + ginan-iers2010 are PRIVATE so the vendor types
# (e.g. iauPV vector types, the iers2010:: namespace) do not leak into
# the public libgnss++ API. Public consumers see only the wrappers in
# include/libgnss++/iers/.
target_link_libraries(gnss_lib PRIVATE sofa ginan_iers2010)

if(MADOCALIB_PARITY_LINK)
    set(MADOCALIB_SRC_DIR "${MADOCALIB_ROOT_DIR}/src")
    if(NOT EXISTS "${MADOCALIB_SRC_DIR}/rtklib.h")
        message(FATAL_ERROR "MADOCALIB source tree not found: ${MADOCALIB_SRC_DIR}")
    endif()

    add_library(madocalib STATIC
        ${MADOCALIB_SRC_DIR}/rtkcmn.c
        ${MADOCALIB_SRC_DIR}/rinex.c
        ${MADOCALIB_SRC_DIR}/rtkpos.c
        ${MADOCALIB_SRC_DIR}/postpos.c
        ${MADOCALIB_SRC_DIR}/solution.c
        ${MADOCALIB_SRC_DIR}/lambda.c
        ${MADOCALIB_SRC_DIR}/geoid.c
        ${MADOCALIB_SRC_DIR}/sbas.c
        ${MADOCALIB_SRC_DIR}/preceph.c
        ${MADOCALIB_SRC_DIR}/pntpos.c
        ${MADOCALIB_SRC_DIR}/ephemeris.c
        ${MADOCALIB_SRC_DIR}/options.c
        ${MADOCALIB_SRC_DIR}/ppp.c
        ${MADOCALIB_SRC_DIR}/ppp_ar.c
        ${MADOCALIB_SRC_DIR}/ppp_iono.c
        ${MADOCALIB_SRC_DIR}/rtcm.c
        ${MADOCALIB_SRC_DIR}/rtcm2.c
        ${MADOCALIB_SRC_DIR}/rtcm3.c
        ${MADOCALIB_SRC_DIR}/rtcm3e.c
        ${MADOCALIB_SRC_DIR}/ionex.c
        ${MADOCALIB_SRC_DIR}/tides.c
        ${MADOCALIB_SRC_DIR}/mdccssr.c
        ${MADOCALIB_SRC_DIR}/mdciono.c
    )
    target_include_directories(madocalib
        PUBLIC
            $<BUILD_INTERFACE:${MADOCALIB_SRC_DIR}>
    )
    target_compile_definitions(madocalib
        PRIVATE
            TRACE
            ENAGLO
            ENAQZS
            ENAGAL
            ENACMP
            ENAIRN
            NFREQ=5
            NEXOBS=5
    )
    target_link_libraries(madocalib PUBLIC Threads::Threads)
    if(WIN32)
        # The Visual Studio generator defines WIN32 implicitly, but Ninja with
        # MSVC does not. MADOCALIB uses this spelling (rather than _WIN32) to
        # select its Win32 headers and synchronization implementation. The
        # native oracle wrapper includes the same upstream header.
        target_compile_definitions(madocalib PRIVATE WIN32)
        target_compile_definitions(gnss_lib PRIVATE WIN32)
        # rtkcmn.c tickget() calls timeGetTime.
        target_link_libraries(madocalib PUBLIC winmm)
    else()
        # libm is implicit in the MSVC CRT; an explicit m.lib breaks the link.
        target_link_libraries(madocalib PUBLIC m)
    endif()
    if(UNIX AND NOT APPLE)
        target_link_libraries(madocalib PUBLIC rt)
    endif()

    target_include_directories(gnss_lib PRIVATE ${MADOCALIB_SRC_DIR})
    target_compile_definitions(gnss_lib
        PUBLIC
            GNSSPP_HAS_MADOCALIB_BRIDGE=1
        PRIVATE
            TRACE
            ENAGLO
            ENAQZS
            ENAGAL
            ENACMP
            ENAIRN
            NFREQ=5
            NEXOBS=5
            "GNSSPP_MADOCALIB_ROOT=\"${MADOCALIB_ROOT_DIR}\""
    )
    if(MADOCALIB_PARITY_ORACLE)
        target_compile_definitions(gnss_lib PUBLIC GNSSPP_HAS_MADOCALIB_ORACLE=1)
    else()
        target_compile_definitions(gnss_lib PUBLIC GNSSPP_HAS_MADOCALIB_ORACLE=0)
    endif()
    target_compile_definitions(gnss_lib PRIVATE GNSSPP_HAS_MADOCALIB_TIDE=1)
    target_link_libraries(gnss_lib PUBLIC madocalib)
    if(MSVC)
        # Optimizing the C oracle wrapper can make cl.exe consume more than
        # 20 GiB even though the wrapper is not runtime-hot. Keep Windows
        # parity builds locally repeatable without changing solver code.
        set_source_files_properties(
            src/algorithms/madocalib_oracle.cpp
            PROPERTIES COMPILE_OPTIONS "/Od")
    endif()
else()
    target_compile_definitions(gnss_lib
        PUBLIC
            GNSSPP_HAS_MADOCALIB_BRIDGE=0
            GNSSPP_HAS_MADOCALIB_ORACLE=0)
endif()

# Historically -O0/-Od for debuggability; the hot solver paths in this list
# (lambda, navigation, spp, observation) made kinematic runs ~2.3x slower.
# -O2 output is byte-identical on the CLAS kinematic parity runs.
add_library(gnss_lib_noopt STATIC ${GNSS_SOURCES_NO_OPTIMIZATION})
if(MSVC)
    target_compile_options(gnss_lib_noopt PRIVATE /O2)
else()
    target_compile_options(gnss_lib_noopt PRIVATE -O2)
endif()
target_include_directories(gnss_lib_noopt PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(gnss_lib_noopt PUBLIC Eigen3::Eigen)
if(CHOLMOD_INCLUDE_DIR AND CHOLMOD_LIBRARY)
    target_include_directories(gnss_lib_noopt PRIVATE ${CHOLMOD_INCLUDE_DIR})
    target_compile_definitions(gnss_lib_noopt PRIVATE GNSSPP_HAS_CHOLMOD=1)
    target_link_libraries(gnss_lib_noopt PRIVATE ${CHOLMOD_LIBRARY})
endif()

add_library(gnss_rtk_runtime STATIC ${GNSS_RTK_RUNTIME_SOURCES})
target_include_directories(gnss_rtk_runtime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(gnss_rtk_runtime PUBLIC Eigen3::Eigen)
if(MSVC)
    target_compile_options(gnss_rtk_runtime PRIVATE /O2)
else()
    target_compile_options(gnss_rtk_runtime PRIVATE -O2)
endif()
if(GTSAM_FOUND)
    # fgo.cpp (FGOProcessor::optimizeProblem) lives in gnss_lib_noopt, so the
    # GTSAM backend TU and the gtsam link dependency must live here too, not
    # on gnss_lib, otherwise the backend-selector forward declaration in
    # fgo.cpp would be an unresolved symbol at link time.
    target_sources(gnss_lib_noopt PRIVATE src/algorithms/fgo_gtsam_backend.cpp)
    target_link_libraries(gnss_lib_noopt PUBLIC gtsam)
    target_compile_definitions(gnss_lib_noopt PUBLIC GNSSPP_HAS_GTSAM=1)
    if(MSVC)
        # gtsam.dll dllexports a handful of STL template instantiations that back
        # its Key containers (std::map<uint64,uint64>, std::vector<uint64>, ...).
        # Those same instantiations also appear as COMDAT symbols in our own
        # object files (e.g. fgo.obj), so every executable that links both
        # gnss_lib_noopt and gtsam hits LNK2005 "already defined". /FORCE:MULTIPLE
        # tells the linker to keep the first definition and drop the duplicates;
        # this is the standard GTSAM-on-Windows shared-build workaround and is
        # safe here because the colliding symbols are identical inline STL code.
        # Propagated as an INTERFACE option so every consumer (apps, tests) that
        # transitively links gtsam through gnss_lib_noopt inherits it.
        target_link_options(gnss_lib_noopt INTERFACE /FORCE:MULTIPLE)
    endif()
endif()

set(GNSS_INSTALL_TARGETS
    gnss_lib
    gnss_lib_noopt
    gnss_rtk_runtime
    sofa
    ginan_iers2010)
if(MADOCALIB_PARITY_LINK)
    list(APPEND GNSS_INSTALL_TARGETS madocalib)
endif()
install(
    # Note: sofa and ginan_iers2010 are PRIVATE link deps of gnss_lib but
    # must still appear in the same export set because static archives do
    # not carry their dependency closure — downstream consumers of an
    # installed libgnsspp::gnss_lib need the vendor symbols at link time.
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
if(GNSSPP_BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)
endif()
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
