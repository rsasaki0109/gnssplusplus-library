# Examples CMakeLists.txt

# Simple SPP example
add_executable(simple_spp simple_spp.cpp)
target_link_libraries(simple_spp PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(simple_test simple_test.cpp)
target_link_libraries(simple_test PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

# RTK positioning example
add_executable(rtk_positioning rtk_positioning.cpp)
target_link_libraries(rtk_positioning PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

# Create output directories
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/data)

# Install examples
install(TARGETS simple_spp rtk_positioning
    RUNTIME DESTINATION bin/examples
)
