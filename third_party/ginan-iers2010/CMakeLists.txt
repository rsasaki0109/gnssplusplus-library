# ginan-iers2010 — IERS Conventions 2010 routines (subset)
#
# Vendored from GeoscienceAustralia/ginan@535ef0a (Apache License 2.0).
# See ./README.md for provenance, scope, and the list of modifications
# made relative to upstream. The wrapper code is licensed under Apache
# License 2.0 (./LICENSE); this CMakeLists.txt is part of libgnss++ and
# is licensed under MIT.

cmake_minimum_required(VERSION 3.14)
project(ginan_iers2010 CXX)

find_package(Eigen3 REQUIRED)

set(GINAN_IERS2010_SOURCES
    ch9/fcul_a.cpp
    ch9/fcul_zd_hpa.cpp
    dehanttideinel/dehanttide_all.cpp
)

add_library(ginan_iers2010 STATIC ${GINAN_IERS2010_SOURCES})

# Public include path: the vendor root, so consumers can write
#     #include "iers2010.hpp"
# Internal (PRIVATE) include path: shim/, so the vendored .cpp files
# resolve their `common/eigenIncluder.hpp` and `common/constants.hpp`
# includes against the minimal libgnss++ stand-ins (see ./README.md).
target_include_directories(ginan_iers2010
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/shim
)

target_link_libraries(ginan_iers2010 PUBLIC Eigen3::Eigen)

set_target_properties(ginan_iers2010 PROPERTIES POSITION_INDEPENDENT_CODE ON)
