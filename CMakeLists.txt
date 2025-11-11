cmake_minimum_required(VERSION 3.24)

project(
    rtklib2
    VERSION 0.1.0
    DESCRIPTION "Modern GNSS positioning engine"
    LANGUAGES CXX
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

option(RTKLIB2_BUILD_TESTS "Build rtklib2 tests" ON)
option(RTKLIB2_BUILD_EXAMPLES "Build rtklib2 examples" ON)
option(RTKLIB2_ENABLE_WARNINGS "Enable compiler warnings" ON)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not supported. Please use a separate build directory.")
endif()

set(RTKLIB2_SOURCES
    src/rtklib2/api/library.cpp
    src/rtklib2/common/version.cpp
    src/rtklib2/core/system_context.cpp
    src/rtklib2/io/data_stream.cpp
    src/rtklib2/pipeline/pipeline.cpp
    src/rtklib2/services/service_registry.cpp
    src/rtklib2/telemetry/logger.cpp
)

add_library(rtklib2 STATIC ${RTKLIB2_SOURCES})

target_sources(
    rtklib2
    PUBLIC
        FILE_SET public_headers
        TYPE HEADERS
        BASE_DIRS include
        FILES
            include/rtklib2/api/rtklib2.hpp
            include/rtklib2/common/version.hpp
            include/rtklib2/core/system_context.hpp
            include/rtklib2/io/data_stream.hpp
            include/rtklib2/pipeline/pipeline.hpp
            include/rtklib2/services/service_registry.hpp
            include/rtklib2/telemetry/logger.hpp
)

target_include_directories(
    rtklib2
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(rtklib2 PUBLIC cxx_std_20)

if(RTKLIB2_ENABLE_WARNINGS)
    if(MSVC)
        target_compile_options(rtklib2 PRIVATE /W4 /permissive- /external:W1)
    else()
        target_compile_options(rtklib2 PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    endif()
endif()

include(GNUInstallDirs)

install(
    TARGETS rtklib2
    EXPORT rtklib2Targets
    FILE_SET public_headers
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    EXPORT rtklib2Targets
    NAMESPACE rtklib2::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rtklib2
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    cmake/rtklib2Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/rtklib2Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rtklib2
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/rtklib2ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/rtklib2Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/rtklib2ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rtklib2
)

if(RTKLIB2_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(RTKLIB2_BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory(tests)
endif()
