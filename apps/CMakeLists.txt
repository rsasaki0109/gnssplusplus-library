add_executable(gnss_solve gnss_solve.cpp)
target_link_libraries(gnss_solve PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_rinex_info gnss_rinex_info.cpp)
target_link_libraries(gnss_rinex_info PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_stream gnss_stream.cpp)
target_link_libraries(gnss_stream PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_ubx_info gnss_ubx_info.cpp)
target_link_libraries(gnss_ubx_info PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_convert gnss_convert.cpp)
target_link_libraries(gnss_convert PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_replay gnss_replay.cpp)
target_link_libraries(gnss_replay PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_live gnss_live.cpp)
target_link_libraries(gnss_live PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

install(TARGETS gnss_solve gnss_rinex_info gnss_stream gnss_ubx_info gnss_convert gnss_replay gnss_live
    RUNTIME DESTINATION bin
)
