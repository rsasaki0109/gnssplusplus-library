# Examples CMakeLists.txt

# Simple SPP example
add_executable(simple_spp simple_spp.cpp)
target_link_libraries(simple_spp gnss_lib gnss_lib_noopt pthread stdc++)

add_executable(simple_test simple_test.cpp)
target_link_libraries(simple_test gnss_lib gnss_lib_noopt pthread stdc++)

# RTK positioning example
add_executable(rtk_positioning rtk_positioning.cpp)
target_link_libraries(rtk_positioning gnss_lib gnss_lib_noopt pthread stdc++)

# Create output directories
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/data)

# Install examples
install(TARGETS simple_spp rtk_positioning
    RUNTIME DESTINATION bin/examples
)
