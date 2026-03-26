set(GNSS_APP_TARGETS
    gnss_spp
    gnss_solve
    gnss_ppp
    gnss_nav_products
    gnss_rinex_info
    gnss_stream
    gnss_ubx_info
    gnss_convert
    gnss_replay
    gnss_live
)

add_executable(gnss_spp gnss_spp.cpp)
target_link_libraries(gnss_spp PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_solve gnss_solve.cpp)
target_link_libraries(gnss_solve PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_ppp gnss_ppp.cpp)
target_link_libraries(gnss_ppp PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_nav_products gnss_nav_products.cpp)
target_link_libraries(gnss_nav_products PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

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

install(TARGETS ${GNSS_APP_TARGETS}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss.py
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    RENAME gnss
)

install(PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_rcv.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_runtime.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_odaiba_benchmark.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_odaiba_scan.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_short_baseline_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_rtk_kinematic_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_ppp_static_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_ppp_kinematic_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_clas_ppp.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_nmea_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_novatel_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_sbp_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_sbf_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_trimble_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_skytraq_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_binex_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_qzss_l6_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/gnss_pos2kml
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(PROGRAMS
    ${CMAKE_SOURCE_DIR}/tools/rtk_stats.py
    ${CMAKE_SOURCE_DIR}/tools/compare_rtklib.py
    ${CMAKE_SOURCE_DIR}/tools/plot_rtk.py
    ${CMAKE_SOURCE_DIR}/tools/plot_trajectory.py
    DESTINATION tools
)

install(PROGRAMS
    ${CMAKE_SOURCE_DIR}/scripts/convert_rtklib_pos.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_driving_comparison.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_odaiba_scorecard.py
    DESTINATION scripts
)

install(FILES
    ${CMAKE_SOURCE_DIR}/configs/live.example.conf
    DESTINATION configs
)
