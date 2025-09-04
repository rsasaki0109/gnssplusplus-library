# Examples CMakeLists.txt

# Simple SPP example
add_executable(simple_spp simple_spp.cpp)
target_link_libraries(simple_spp libgnss++)

# RTK positioning example
add_executable(rtk_positioning rtk_positioning.cpp)
target_link_libraries(rtk_positioning libgnss++)

# Create output directories
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/data)

# Install examples
install(TARGETS simple_spp rtk_positioning
    RUNTIME DESTINATION bin/examples
)
