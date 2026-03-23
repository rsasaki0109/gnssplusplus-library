add_executable(gnss_solve gnss_solve.cpp)
target_link_libraries(gnss_solve PRIVATE gnss_lib gnss_lib_noopt pthread stdc++)

install(TARGETS gnss_solve
    RUNTIME DESTINATION bin
)
