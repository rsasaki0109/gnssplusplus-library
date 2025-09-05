cmake_minimum_required(VERSION 3.10)
project(gnss_lib CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find necessary packages
find_package(Eigen3 REQUIRED)

# List source files for the main library
set(GNSS_SOURCES
    src/gnss.cpp
    src/io/rinex.cpp
    src/io/rtcm.cpp
    src/core/solution.cpp
    src/core/types.cpp
    src/algorithms/ppp.cpp
)

# List problematic files for the no-optimization library
set(GNSS_SOURCES_NO_OPTIMIZATION
    src/algorithms/rtk.cpp
    src/algorithms/spp.cpp
    src/core/navigation.cpp
    src/core/observation.cpp
)

# Create the main library
add_library(gnss_lib STATIC ${GNSS_SOURCES})
target_include_directories(gnss_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# Create the no-optimization library
add_library(gnss_lib_noopt STATIC ${GNSS_SOURCES_NO_OPTIMIZATION})
target_compile_options(gnss_lib_noopt PRIVATE -O0)
target_include_directories(gnss_lib_noopt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# Add subdirectories
add_subdirectory(examples)
add_subdirectory(tests)
