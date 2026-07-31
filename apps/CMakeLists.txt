set(GNSS_APP_TARGETS
    gnss_spp
    gnss_fgo
    gnss_fuse
    gnss_vel_d
    gnss_pos_pdc
    gnss_pos_pd
    gnss_pos_vel_pdc
    gnss_pos_vel_pd
    gnss_solve
    gnss_ppp
    gnss_visibility
    gnss_nav_products
    gnss_rinex_info
    gnss_stream
    gnss_ubx_info
    gnss_convert
    gnss_replay
    gnss_live
)

set(GNSS_NATIVE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/native)

add_executable(gnss_spp ${GNSS_NATIVE_DIR}/gnss_spp.cpp)
target_link_libraries(gnss_spp PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_fgo ${GNSS_NATIVE_DIR}/gnss_fgo.cpp)
target_link_libraries(gnss_fgo PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)
if(GTSAM_FOUND)
    target_compile_definitions(gnss_fgo PRIVATE GNSS_WITH_GTSAM=1)
    target_link_libraries(gnss_fgo PRIVATE gtsam)
endif()

# Real-data Eigen-vs-GTSAM FGO backend parity harness. Only meaningful when the
# library is built with the GTSAM backend (otherwise FGOBackend::GTSAM falls
# back to Eigen). gnss_lib_noopt carries the gtsam link + /FORCE:MULTIPLE as an
# INTERFACE requirement, so this target inherits both automatically.
if(GTSAM_FOUND)
    add_executable(gnss_fgo_parity ${GNSS_NATIVE_DIR}/gnss_fgo_parity.cpp)
    target_link_libraries(gnss_fgo_parity PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)
endif()

add_executable(gnss_fuse
    ${GNSS_NATIVE_DIR}/gnss_fuse.cpp
    ${GNSS_NATIVE_DIR}/cli_toml_config.cpp
)
target_link_libraries(gnss_fuse PRIVATE gnss_lib gnss_rtk_runtime Threads::Threads)

add_executable(gnss_safe_bridge gnss_safe_bridge.cpp)
target_link_libraries(gnss_safe_bridge PRIVATE gnss_lib gnss_rtk_runtime)

add_executable(gnss_vel_d ${GNSS_NATIVE_DIR}/gnss_vel_d.cpp)
target_link_libraries(gnss_vel_d PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_pos_pdc ${GNSS_NATIVE_DIR}/gnss_pos_pdc.cpp)
target_link_libraries(gnss_pos_pdc PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_pos_pd ${GNSS_NATIVE_DIR}/gnss_pos_pd.cpp)
target_link_libraries(gnss_pos_pd PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_pos_vel_pdc ${GNSS_NATIVE_DIR}/gnss_pos_vel_pdc.cpp)
target_link_libraries(gnss_pos_vel_pdc PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_pos_vel_pd ${GNSS_NATIVE_DIR}/gnss_pos_vel_pd.cpp)
target_link_libraries(gnss_pos_vel_pd PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_solve
    ${GNSS_NATIVE_DIR}/gnss_solve.cpp
    ${GNSS_NATIVE_DIR}/cli_toml_config.cpp
)
target_link_libraries(gnss_solve PRIVATE gnss_lib gnss_rtk_runtime Threads::Threads)

add_executable(gnss_ppp ${GNSS_NATIVE_DIR}/gnss_ppp.cpp)
target_link_libraries(gnss_ppp PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

if(MADOCALIB_PARITY_LINK)
    # The linked MADOCALIB runtime is an oracle-only CI surface. Keep its
    # command-line selector out of the installed production gnss_ppp binary.
    add_executable(gnss_madocalib_oracle ${GNSS_NATIVE_DIR}/gnss_ppp.cpp)
    target_compile_definitions(
        gnss_madocalib_oracle PRIVATE GNSSPP_MADOCALIB_ORACLE_CLI=1)
    target_link_libraries(
        gnss_madocalib_oracle PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)
endif()

add_executable(gnss_visibility ${GNSS_NATIVE_DIR}/gnss_visibility.cpp)
target_link_libraries(gnss_visibility PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_nav_products ${GNSS_NATIVE_DIR}/gnss_nav_products.cpp)
target_link_libraries(gnss_nav_products PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_rinex_info ${GNSS_NATIVE_DIR}/gnss_rinex_info.cpp)
target_link_libraries(gnss_rinex_info PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_stream ${GNSS_NATIVE_DIR}/gnss_stream.cpp)
target_link_libraries(gnss_stream PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_ubx_info ${GNSS_NATIVE_DIR}/gnss_ubx_info.cpp)
target_link_libraries(gnss_ubx_info PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_convert ${GNSS_NATIVE_DIR}/gnss_convert.cpp)
target_link_libraries(gnss_convert PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_replay ${GNSS_NATIVE_DIR}/gnss_replay.cpp)
target_link_libraries(gnss_replay PRIVATE gnss_lib gnss_lib_noopt Threads::Threads)

add_executable(gnss_live ${GNSS_NATIVE_DIR}/gnss_live.cpp)
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
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/diagnostics/gnss_doctor.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/diagnostics/gnss_robotics_smoke.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/diagnostics/gnss_ros2_doctor.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/diagnostics/gnss_ros2_bag_doctor.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/diagnostics/gnss_field_report.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/receivers/gnss_rcv.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/visualization/gnss_web.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/diagnostics/gnss_mode_compare.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppc_taroz_amb_pdc_smoke.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_taroz_p_dogfood.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_taroz_pd_dogfood.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_taroz_pc_dogfood.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_taroz_observable_dogfood.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_taroz_pos_vel_amb_pdc_dogfood.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_taroz_oracle_suite.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/positioning/gnss_live_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/positioning/gnss_moving_base_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/positioning/gnss_scorpion_moving_base_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/positioning/gnss_moving_base_prepare.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/visualization/gnss_moving_base_plot.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/products/gnss_fetch_products.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/visualization/gnss_artifact_manifest.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/products/gnss_ionex_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/products/gnss_dcb_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/visualization/gnss_visibility_plot.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_odaiba_benchmark.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_odaiba_scan.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/positioning/gnss_short_baseline_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/positioning/gnss_rtk_kinematic_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/positioning/gnss_ppp_static_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/positioning/gnss_ppp_kinematic_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppp_iers_solid_tide_bench.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppp_iers_pole_tide_bench.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppp_iers_atm_tidal_loading_bench.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppp_iers_atm_tidal_loading_multisite_bench.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppp_iers_pole_tide_multisite_bench.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppp_iers_truth_bench.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppp_iers_truth_multisite_bench.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppp_iers_sub_daily_eop_bench.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppp_iers_sub_daily_eop_multisite_bench.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppp_iers_ocean_loading_bench.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/products/gnss_vmf_atl.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/positioning/gnss_ppp_products_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppc_demo.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppc_commercial.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppc_metrics.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_ppc_rtk_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_public_rtk_benchmarks.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_smartloc_adapter.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/benchmarks/gnss_smartloc_signoff.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/positioning/gnss_clas_ppp.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/receivers/gnss_nmea_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/receivers/gnss_novatel_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/receivers/gnss_sbp_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/receivers/gnss_sbf_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/receivers/gnss_trimble_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/receivers/gnss_skytraq_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/receivers/gnss_binex_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/receivers/gnss_qzss_l6_info.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/receivers/gnss_qzss_l6_bias.py
    ${CMAKE_CURRENT_SOURCE_DIR}/compat/gnss_pos2kml
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/support/__init__.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/support/gnss_runtime.py
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/support/gnss_toml_config.py
    DESTINATION ${CMAKE_INSTALL_BINDIR}/support
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
    ${CMAKE_SOURCE_DIR}/scripts/generate_architecture_diagram.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_driving_comparison.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_feature_overview_card.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_odaiba_scorecard.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_odaiba_social_card.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_ppc_rtk_scorecard.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_ppc_tail_cleanup_scorecard.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_ppc_rtk_trajectory.py
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_p_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_d_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_pos_pdc_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_pos_pd_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_pos_vel_pdc_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_pos_vel_ambiguity_pdc_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_pd_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/analyze_ppc_dual_profile_selector_matrix.py
    ${CMAKE_SOURCE_DIR}/scripts/apply_ppc_dual_profile_selector.py
    ${CMAKE_SOURCE_DIR}/scripts/analyze_ppc_profile_segment_delta.py
    ${CMAKE_SOURCE_DIR}/scripts/analyze_ppc_residual_reset_sweep.py
    ${CMAKE_SOURCE_DIR}/scripts/analyze_ppc_segment_selector_sweep.py
    ${CMAKE_SOURCE_DIR}/scripts/update_ppc_coverage_readme.py
    DESTINATION scripts
)

install(FILES
    ${CMAKE_SOURCE_DIR}/configs/README.md
    DESTINATION configs
)

install(FILES
    ${CMAKE_SOURCE_DIR}/configs/examples/fuse.example.toml
    ${CMAKE_SOURCE_DIR}/configs/examples/live.example.conf
    ${CMAKE_SOURCE_DIR}/configs/examples/solve.example.toml
    ${CMAKE_SOURCE_DIR}/configs/examples/web.example.toml
    DESTINATION configs/examples
)

install(FILES
    ${CMAKE_SOURCE_DIR}/configs/signoff/live_signoff.example.toml
    ${CMAKE_SOURCE_DIR}/configs/signoff/moving_base_signoff.example.toml
    ${CMAKE_SOURCE_DIR}/configs/signoff/ppc_rtk_signoff.example.toml
    ${CMAKE_SOURCE_DIR}/configs/signoff/ppp_products_ppc.example.toml
    DESTINATION configs/signoff
)

install(FILES
    ${CMAKE_SOURCE_DIR}/configs/benchmarks/iers_atl_multisite_smoke.example.json
    ${CMAKE_SOURCE_DIR}/configs/benchmarks/iers_atl_multisite_vmf.example.json
    DESTINATION configs/benchmarks
)

install(FILES
    ${CMAKE_SOURCE_DIR}/test_data/iers/pert_vmf.atl
    ${CMAKE_SOURCE_DIR}/test_data/iers/tskb_synth.atl
    ${CMAKE_SOURCE_DIR}/test_data/iers/tskb_vmf.atl
    DESTINATION test_data/iers
)
