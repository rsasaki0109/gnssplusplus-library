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

file(GLOB GNSS_PYTHON_COMMAND_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/commands/*/*.py"
)

set(GNSS_PYTHON_COMMAND_PROGRAMS)
set(GNSS_PYTHON_COMMAND_INSTALL_KEYS)
list(SORT GNSS_PYTHON_COMMAND_FILES)
foreach(GNSS_PYTHON_COMMAND_FILE IN LISTS GNSS_PYTHON_COMMAND_FILES)
    get_filename_component(GNSS_PYTHON_COMMAND_CATEGORY_DIR
        "${GNSS_PYTHON_COMMAND_FILE}" DIRECTORY)
    get_filename_component(GNSS_PYTHON_COMMAND_CATEGORY_NAME
        "${GNSS_PYTHON_COMMAND_CATEGORY_DIR}" NAME)
    if(GNSS_PYTHON_COMMAND_CATEGORY_NAME STREQUAL "support")
        continue()
    endif()

    get_filename_component(GNSS_PYTHON_COMMAND_BASENAME
        "${GNSS_PYTHON_COMMAND_FILE}" NAME)
    string(TOLOWER "${GNSS_PYTHON_COMMAND_BASENAME}"
        GNSS_PYTHON_COMMAND_INSTALL_KEY)
    list(FIND GNSS_PYTHON_COMMAND_INSTALL_KEYS
        "${GNSS_PYTHON_COMMAND_INSTALL_KEY}"
        GNSS_PYTHON_COMMAND_DUPLICATE_INDEX)
    if(NOT GNSS_PYTHON_COMMAND_DUPLICATE_INDEX EQUAL -1)
        list(GET GNSS_PYTHON_COMMAND_PROGRAMS
            ${GNSS_PYTHON_COMMAND_DUPLICATE_INDEX}
            GNSS_PYTHON_COMMAND_PREVIOUS)
        get_filename_component(GNSS_PYTHON_COMMAND_PREVIOUS_BASENAME
            "${GNSS_PYTHON_COMMAND_PREVIOUS}" NAME)
        message(FATAL_ERROR
            "Python command install basename collision (case-insensitive): "
            "${GNSS_PYTHON_COMMAND_PREVIOUS_BASENAME} "
            "(${GNSS_PYTHON_COMMAND_PREVIOUS}) and "
            "${GNSS_PYTHON_COMMAND_BASENAME} (${GNSS_PYTHON_COMMAND_FILE})")
    endif()
    list(APPEND GNSS_PYTHON_COMMAND_INSTALL_KEYS
        "${GNSS_PYTHON_COMMAND_INSTALL_KEY}")
    list(APPEND GNSS_PYTHON_COMMAND_PROGRAMS "${GNSS_PYTHON_COMMAND_FILE}")
endforeach()
list(SORT GNSS_PYTHON_COMMAND_PROGRAMS)

file(GLOB GNSS_PYTHON_SUPPORT_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/commands/support/*.py"
)
list(SORT GNSS_PYTHON_SUPPORT_FILES)

install(PROGRAMS
    ${GNSS_PYTHON_COMMAND_PROGRAMS}
    ${CMAKE_CURRENT_SOURCE_DIR}/compat/gnss_pos2kml
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/commands/visualization/gnss_web.html
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES
    ${GNSS_PYTHON_SUPPORT_FILES}
    DESTINATION ${CMAKE_INSTALL_BINDIR}/support
)

install(PROGRAMS
    ${CMAKE_SOURCE_DIR}/tools/rtk_stats.py
    ${CMAKE_SOURCE_DIR}/tools/compare_rtklib.py
    ${CMAKE_SOURCE_DIR}/tools/plot_rtk.py
    ${CMAKE_SOURCE_DIR}/tools/plot_trajectory.py
    DESTINATION tools
)

install(FILES
    ${CMAKE_SOURCE_DIR}/tools/rtk_geometry.py
    DESTINATION tools
)

install(PROGRAMS
    ${CMAKE_SOURCE_DIR}/scripts/convert_rtklib_pos.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_architecture_diagram.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_driving_comparison.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_feature_overview_card.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_odaiba_scorecard.py
    ${CMAKE_SOURCE_DIR}/scripts/generate_odaiba_social_card.py
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_p_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_d_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_pos_pdc_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_pos_pd_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_pos_vel_pdc_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_pos_vel_ambiguity_pdc_debug.m
    ${CMAKE_SOURCE_DIR}/scripts/dump_taroz_pd_debug.m
    DESTINATION scripts
)

install(PROGRAMS
    ${CMAKE_SOURCE_DIR}/scripts/experiments/ppc/apply_ppc_dual_profile_selector.py
    ${CMAKE_SOURCE_DIR}/scripts/experiments/ppc/generate_ppc_rtk_scorecard.py
    ${CMAKE_SOURCE_DIR}/scripts/experiments/ppc/generate_ppc_rtk_trajectory.py
    ${CMAKE_SOURCE_DIR}/scripts/experiments/ppc/generate_ppc_tail_cleanup_scorecard.py
    ${CMAKE_SOURCE_DIR}/scripts/experiments/ppc/update_ppc_coverage_readme.py
    DESTINATION scripts/experiments/ppc
)

install(FILES
    ${CMAKE_SOURCE_DIR}/scripts/experiments/ppc/_paths.py
    DESTINATION scripts/experiments/ppc
)

install(PROGRAMS
    ${CMAKE_SOURCE_DIR}/scripts/analysis/analyze_ppc_dual_profile_selector_matrix.py
    ${CMAKE_SOURCE_DIR}/scripts/analysis/analyze_ppc_profile_segment_delta.py
    ${CMAKE_SOURCE_DIR}/scripts/analysis/analyze_ppc_residual_reset_sweep.py
    ${CMAKE_SOURCE_DIR}/scripts/analysis/analyze_ppc_segment_selector_sweep.py
    DESTINATION scripts/analysis
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
