# Tools CMakeLists.txt

# GNSS positioning tool
add_executable(gnss_pos gnss_pos.cpp)
target_link_libraries(gnss_pos libgnss++)

# Install tools
install(TARGETS gnss_pos
    RUNTIME DESTINATION bin
)
