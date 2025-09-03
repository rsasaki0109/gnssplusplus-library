cmake_minimum_required(VERSION 3.15)
project(LibGNSSPlusPlus VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_TOOLS "Build command-line tools" ON)
option(USE_OPENMP "Enable OpenMP parallelization" ON)

# Find dependencies
find_package(Eigen3 REQUIRED)

if(USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
endif()

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3 -march=native")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# Header-only library
add_library(libgnss++ INTERFACE)
target_include_directories(libgnss++ INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(libgnss++ INTERFACE cxx_std_17)

# Link Eigen3
target_link_libraries(libgnss++ INTERFACE Eigen3::Eigen)

if(OpenMP_CXX_FOUND)
    target_link_libraries(libgnss++ INTERFACE OpenMP::OpenMP_CXX)
endif()

# Add subdirectories
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

# Installation
install(TARGETS libgnss++
    EXPORT libgnss++Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT libgnss++Targets
    FILE libgnss++Targets.cmake
    NAMESPACE libgnss++::
    DESTINATION lib/cmake/libgnss++
)

# Create config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "libgnss++ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/libgnss++Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/libgnss++Config.cmake"
    INSTALL_DESTINATION lib/cmake/libgnss++
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/libgnss++Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/libgnss++ConfigVersion.cmake"
    DESTINATION lib/cmake/libgnss++
)
