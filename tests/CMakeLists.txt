# Tests CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

# Find Google Test
find_package(GTest REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

if(GTest_FOUND)
    # Consolidate all tests into a single executable.
    # The target is named gnss_run_tests (not run_tests): with the Visual
    # Studio generator, CMake emits a per-directory RUN_TESTS utility whose
    # project file collides case-insensitively with run_tests.vcxproj on
    # Windows. OUTPUT_NAME keeps the binary named run_tests for scripts/CI.
    add_executable(gnss_run_tests
        test_imu_csv.cpp
        test_android_raw_gnss.cpp
        test_smartphone_upstream_preprocessing.cpp
        test_attitude.cpp
        test_mechanization.cpp
        test_preintegration.cpp
        test_tight_coupling_processor.cpp
        test_fusion_process_noise.cpp
        test_fusion_update.cpp
        test_fusion_measurement.cpp
        test_fusion_zupt.cpp
        test_fusion_nhc.cpp
        test_fusion_initialization.cpp
        test_fusion_processor_synthetic.cpp
        test_dd_imu_bridge.cpp
        test_ppp_clas.cpp
        test_ppp_osr.cpp
        test_ppp_ar.cpp
        test_ppp_atmosphere.cpp
        test_ppp_correction_contract.cpp
        test_rtk_ar_evaluation.cpp
        test_rtk_ar_selection.cpp
        test_safe_fix_state_machine.cpp
          test_fixed_quality_gate.cpp
          test_fix_failure_budget.cpp
    test_inertial_fix_evidence.cpp
    test_disjoint_satellite_fix_evidence.cpp
        test_disjoint_constellation_partition.cpp
        test_causal_ambiguity_arc.cpp
        test_nlos_weights.cpp
        test_float_trust_policy.cpp
        test_rtk_slip_detection.cpp
        test_rtk_adaptive_noise.cpp
        test_rtk_doppler_rows.cpp
        test_rtk_float_stabilizer.cpp
        test_rtk_measurement.cpp
        test_rtk_update.cpp
        test_rtk_ins_time_update.cpp
        test_rtk_tdcp_diagnostics.cpp
        test_rtk_selection.cpp
        test_rtk_cmc_ref.cpp
        test_rtk_cp_pr_gate.cpp
        test_rtk_ddpr_anchor.cpp
        test_rtk_validation.cpp
        test_integrity_consensus.cpp
        test_fgo.cpp
        test_signal_policy.cpp
        test_types.cpp
        test_observation.cpp
        test_gnss_live.cpp
        test_ppp.cpp
        test_rinex_writer.cpp
        test_madoca_core.cpp
        test_madoca_l6d_snapshots.cpp
        test_madoca_parity.cpp
        test_spp.cpp
        test_spp_velocity.cpp
        test_rtk.cpp
        test_rtk_ins_prior.cpp
        test_rtk_smoke.cpp
        test_ntrip.cpp
        test_rtcm.cpp
        test_rtcm_stream.cpp
        test_ubx.cpp
        test_sofa_vendor.cpp
        test_ginan_iers2010_vendor.cpp
        test_iers_earth_rotation.cpp
        test_iers_tides.cpp
        test_iers_ephemeris.cpp
        test_iers_eop.cpp
        test_iers_sub_daily_eop.cpp
    )

    # test_rtk_legacy.cpp uses "#define private public" before including
    # rtk.hpp. MSVC encodes member access in mangled names, so the test's
    # references never link against the library's private symbols; the
    # trick only works on Itanium-ABI compilers (GCC/Clang). That mangling
    # is a property of the target ABI, not of which compiler emits it: Clang
    # targeting the MSVC ABI (e.g. a Windows build/clang-ninja config, where
    # CMAKE_CXX_COMPILER_ID is "Clang" and MSVC is therefore false here)
    # mangles member access the same way real MSVC does and hits the exact
    # same undefined-symbol link failure, so it must be excluded too.
    if(NOT MSVC AND NOT (WIN32 AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
        target_sources(gnss_run_tests PRIVATE test_rtk_legacy.cpp)
    endif()

    # GTSAM FGO backend parity tests (guarded at both CMake and #ifdef level:
    # the source itself is a no-op TU when GNSSPP_HAS_GTSAM isn't defined, but
    # we still only add it to the build when GTSAM was actually found so a
    # GTSAM-less build doesn't pay for an always-empty translation unit).
    if(GTSAM_FOUND)
        target_sources(gnss_run_tests PRIVATE test_fgo_gtsam_backend.cpp)
    endif()

    set_target_properties(gnss_run_tests PROPERTIES OUTPUT_NAME run_tests)
    # Backward-compatible alias for CI/scripts that build --target run_tests.
    # Skipped for Visual Studio generators, where the name collides with
    # CMake's auto-generated RUN_TESTS utility on a case-insensitive FS.
    if(NOT CMAKE_GENERATOR MATCHES "Visual Studio")
        add_custom_target(run_tests DEPENDS gnss_run_tests)
    endif()
    if(NOT MSVC)
        target_compile_options(gnss_run_tests PRIVATE -g)
    endif()
    target_compile_definitions(gnss_run_tests PRIVATE
        GNSSPP_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
        GNSSPP_BINARY_DIR="${CMAKE_BINARY_DIR}"
    )
    target_link_libraries(gnss_run_tests
        gnss_lib gnss_lib_solvers
        sofa
        ginan_iers2010
        GTest::GTest
        GTest::Main
        Threads::Threads
    )
    add_dependencies(gnss_run_tests
        gnss_convert
        gnss_fgo
        gnss_fuse
        gnss_live
        gnss_pos_pd
        gnss_pos_pdc
        gnss_pos_vel_pd
        gnss_pos_vel_pdc
        gnss_ppp
        gnss_replay
        gnss_rinex_info
        gnss_solve
        gnss_smartphone_fgo
        gnss_stream
        gnss_ubx_info
        gnss_vel_d
    )

    # Keep the MADOCALIB oracle job independent from the monolithic test
    # executable.  The parity workflow only executes tests from
    # test_madoca_parity.cpp; compiling every unrelated test translation unit
    # made the Ubuntu runner vulnerable to infrastructure SIGTERM (exit 143)
    # before the oracle tests could start.
    add_executable(gnss_madoca_parity_tests test_madoca_parity.cpp)
    set_target_properties(gnss_madoca_parity_tests PROPERTIES
        OUTPUT_NAME madoca_parity_tests
    )
    if(NOT MSVC)
        target_compile_options(gnss_madoca_parity_tests PRIVATE -g)
    endif()
    target_compile_definitions(gnss_madoca_parity_tests PRIVATE
        GNSSPP_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
        GNSSPP_BINARY_DIR="${CMAKE_BINARY_DIR}"
    )
    target_link_libraries(gnss_madoca_parity_tests
        gnss_lib gnss_lib_solvers
        sofa
        ginan_iers2010
        GTest::GTest
        GTest::Main
        Threads::Threads
    )
    add_dependencies(gnss_madoca_parity_tests gnss_ppp)

    add_executable(gnss_integrity_tests test_integrity_consensus.cpp)
    target_link_libraries(gnss_integrity_tests
        gnss_lib gnss_lib_solvers
        GTest::GTest
        GTest::Main
        Threads::Threads
    )

    # Add test to CTest
    add_test(NAME run_tests COMMAND gnss_run_tests)
    add_test(
        NAME clas_outage_legacy_kill_switch
        COMMAND gnss_run_tests
                --gtest_filter=PPPClasTest.MrtklibOutageResetKillSwitchRestoresLegacyOverflow
    )
    set_tests_properties(
        clas_outage_legacy_kill_switch
        PROPERTIES ENVIRONMENT "GNSS_PPP_CLAS_OUTAGE_RESET_PARITY=0"
    )
    add_test(
        NAME madoca_parity_tests
        COMMAND gnss_madoca_parity_tests
                --gtest_filter=*MadocaParity*:MadocaBridgeConfig.*
    )
    add_test(NAME integrity_tests COMMAND gnss_integrity_tests)
    add_test(
        NAME python_benchmark_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_benchmark_scripts.py
    )
    add_test(
        NAME python_fgo_arc_lifecycle_audit_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_fgo_arc_lifecycle_audit.py
    )
    add_test(
        NAME python_fgo_external_dr_witness_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_fgo_external_dr_witness.py
    )
    add_test(
        NAME python_smartphone_native_fgo_v2_1_contract_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_v2_1_contract.py
    )
    add_test(
        NAME python_smartphone_native_fgo_v2_1_confirmation_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_v2_1_confirmation.py
    )
    add_test(
        NAME python_smartphone_native_fgo_v3_heading_optional_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_v3_heading_optional.py
    )
    add_test(
        NAME python_smartphone_native_only_contract_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_only.py
    )
    add_test(
        NAME python_smartphone_native_gnss_pdc_bridge_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_gnss_pdc_bridge.py
    )
    add_test(
        NAME python_smartphone_native_gnss_pdc_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_gnss_pdc_eval.py
    )
    add_test(
        NAME python_ppc_integrity_policy_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_ppc_integrity_policy_ci.py
    )
    add_test(
        NAME python_ppc_goal_completion_audit_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/experiments/ppc/verify_ppc_goal_completion.py --check-only
    )
    add_test(
        NAME python_cli_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_cli_tools.py
    )
    add_test(
        NAME python_cli_ux_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_cli_ux.py
    )
    add_test(
        NAME python_cli_json_contract_validator_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_cli_json_contract_validator.py
    )
    add_test(
        NAME python_artifact_manifest_ux_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_artifact_manifest_ux.py
    )
    add_test(
        NAME python_artifact_manifest_contract_validator_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_artifact_manifest_contract_validator.py
    )
    add_test(
        NAME python_madoca_solution_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_madoca_solution_diff.py
    )
    add_test(
        NAME python_madoca_release_matrix_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_madoca_release_matrix.py
    )
    add_test(
        NAME python_madoca_residual_component_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_madoca_residual_component_diff.py
    )
    add_test(
        NAME python_madoca_residual_component_summary_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_madoca_residual_component_summary.py
    )
    add_test(
        NAME python_madoca_glonass_phase_audit_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_madoca_glonass_phase_audit.py
    )
    add_test(
        NAME python_madoca_materialization_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_madoca_materialization_diff.py
    )
    add_test(
        NAME python_madoca_materialization_summary_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_madoca_materialization_summary.py
    )
    add_test(
        NAME python_optional_madoca_residual_component_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_optional_madoca_residual_component_diff.py
    )
    add_test(
        NAME python_optional_madoca_materialization_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_optional_madoca_materialization_diff.py
    )
    add_test(
        NAME python_madoca_materialization_selfdiff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_madoca_materialization_selfdiff.py
    )
    add_test(
        NAME python_clas_zd_component_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_clas_zd_component_diff.py
    )
    add_test(
        NAME python_clas_dd_measurement_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_clas_dd_measurement_diff.py
    )
    add_test(
        NAME python_clas_filter_state_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_clas_filter_state_diff.py
    )
    add_test(
        NAME python_claslib_parity_scorecard_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_claslib_parity_scorecard.py
    )
    add_test(
        NAME python_qzss_l6_bias_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_qzss_l6_bias.py
    )
    add_test(
        NAME python_clas_zd_component_summary_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_clas_zd_component_summary.py
    )
    add_test(
        NAME python_claslib_zd_component_export_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_claslib_zd_component_export.py
    )
    add_test(
        NAME python_optional_clas_zd_component_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_optional_clas_zd_component_diff.py
    )
    add_test(
        NAME python_clas_a4b_native_selfdiff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_clas_a4b_native_selfdiff.py
    )
    add_test(
        NAME python_claslib_osr_zd_export_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_claslib_osr_zd_export.py
    )
    add_test(
        NAME python_madoca_satellite_set_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_madoca_satellite_set_diff.py
    )
    add_test(
        NAME python_mode_compare_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_mode_compare.py
    )
    add_test(
        NAME python_performance_report_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_gnss_performance_report.py
    )
    add_test(
        NAME python_smartphone_quality_report_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_quality_report.py
    )
    add_test(
        NAME python_smartphone_raw_quality_control_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_raw_quality_control.py
    )
    add_test(
        NAME python_smartphone_raw_quality_control_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_raw_quality_control_eval.py
    )
    add_test(
        NAME python_smartphone_kaggle_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_kaggle.py
    )
    add_test(
        NAME python_smartphone_trajectory_smoother_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_trajectory_smoother.py
    )
    add_test(
        NAME python_smartphone_trajectory_imu_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_trajectory_imu.py
    )
    add_test(
        NAME python_smartphone_imu_motion_q_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_imu_motion_q_eval.py
    )
    add_test(
        NAME python_smartphone_tdcp_trajectory_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_tdcp_trajectory.py
    )
    add_test(
        NAME python_smartphone_tdcp_trajectory_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_tdcp_trajectory_eval.py
    )
    add_test(
        NAME python_smartphone_observable_error_correction_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_observable_error_correction.py
    )
    add_test(
        NAME python_smartphone_doppler_position_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_doppler_position.py
    )
    add_test(
        NAME python_smartphone_generalization_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_generalization.py
    )
    add_test(
        NAME python_smartphone_reacquisition_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_reacquisition_eval.py
    )
    add_test(
        NAME python_smartphone_reacquisition_conservative_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_reacquisition_conservative_eval.py
    )
    add_test(
        NAME python_smartphone_segment_stability_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_segment_stability.py
    )
    add_test(
        NAME python_smartphone_segment_stability_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_segment_stability_eval.py
    )
    add_test(
        NAME python_smartphone_wls_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls.py
    )
    add_test(
        NAME python_smartphone_wls_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_eval.py
    )
    add_test(
        NAME python_smartphone_wls_device_family_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_device_family_eval.py
    )
    add_test(
        NAME python_smartphone_wls_stability_selector_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_stability_selector.py
    )
    add_test(
        NAME python_smartphone_wls_stability_selector_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_stability_selector_eval.py
    )
    add_test(
        NAME python_smartphone_wls_residual_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_residual.py
    )
    add_test(
        NAME python_smartphone_wls_residual_v2_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_residual_v2.py
    )
    add_test(
        NAME python_smartphone_wls_stability_selector_holdout_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_stability_selector_holdout_eval.py
    )
    add_test(
        NAME python_smartphone_wls_multi_phone_ensemble_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_multi_phone_ensemble.py
    )
    add_test(
        NAME python_smartphone_wls_multi_phone_ensemble_holdout_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_multi_phone_ensemble_holdout_eval.py
    )
    add_test(
        NAME python_smartphone_wls_test_batch_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_test_batch.py
    )
    add_test(
        NAME python_smartphone_native_fgo_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_eval.py
    )
    add_test(
        NAME python_smartphone_native_fgo_holdout_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_holdout_eval.py
    )
    add_test(
        NAME python_smartphone_native_fgo_test_batch_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_test_batch.py
    )
    add_test(
        NAME python_smartphone_native_fgo_v2_smoke_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_v2_smoke.py
    )
    add_test(
        NAME python_smartphone_native_fgo_test_submission_recovery_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_test_submission_recovery.py
    )
    add_test(
        NAME python_smartphone_native_fgo_test_submission_continuity_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_test_submission_continuity.py
    )
    add_test(
        NAME python_smartphone_native_fgo_test_submission_source_seam_bridge_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_test_submission_source_seam_bridge.py
    )
    add_test(
        NAME python_smartphone_native_fgo_optimizer_stop_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_optimizer_stop_eval.py
    )
    add_test(
        NAME python_smartphone_native_fgo_convergence_selector_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_convergence_selector_eval.py
    )
    add_test(
        NAME python_smartphone_native_fgo_float_carrier_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_float_carrier_eval.py
    )
    add_test(
        NAME python_smartphone_native_fgo_carrier_compatibility_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_carrier_compatibility_eval.py
    )
    add_test(
        NAME python_smartphone_native_fgo_pdc_bridge_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_pdc_bridge.py
    )
    add_test(
        NAME python_smartphone_native_fgo_pdc_wls_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_pdc_wls.py
    )
    add_test(
        NAME python_smartphone_native_fgo_pdc_factor_audit_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_pdc_factor_audit.py
    )
    add_test(
        NAME python_smartphone_native_fgo_pdc_factor_hypotheses_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_pdc_factor_hypotheses.py
    )
    add_test(
        NAME python_smartphone_native_fgo_pdc_witness_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_pdc_witness.py
    )
    add_test(
        NAME python_smartphone_native_fgo_pdc_windowed_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_pdc_windowed.py
    )
    add_test(
        NAME python_smartphone_native_fgo_pdc_windowed_stitch_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_pdc_windowed_stitch.py
    )
    add_test(
        NAME python_ppp_residual_row_set_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_ppp_residual_row_set_diff.py
    )
    add_test(
        NAME python_spp_residual_dump_summary_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_spp_residual_dump_summary.py
    )
    add_test(
        NAME python_spp_iteration_dump_summary_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_spp_iteration_dump_summary.py
    )
    add_test(
        NAME python_ppc_taroz_amb_pdc_smoke_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_ppc_taroz_amb_pdc_smoke.py
    )
    add_test(
        NAME python_taroz_pc_internal_parity_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_pc_internal_parity.py
    )
    add_test(
        NAME python_taroz_pc_dogfood_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_pc_dogfood.py
    )
    add_test(
        NAME python_taroz_p_dogfood_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_p_dogfood.py
    )
    add_test(
        NAME python_taroz_pd_dogfood_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_pd_dogfood.py
    )
    add_test(
        NAME python_taroz_observable_dogfood_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_observable_dogfood.py
    )
    add_test(
        NAME python_taroz_oracle_suite_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_oracle_suite.py
    )
    add_test(
        NAME python_taroz_p_internal_parity_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_p_internal_parity.py
    )
    add_test(
        NAME python_taroz_d_internal_parity_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_d_internal_parity.py
    )
    add_test(
        NAME python_taroz_pos_pd_internal_parity_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_pos_pd_internal_parity.py
    )
    add_test(
        NAME python_taroz_pos_pdc_internal_parity_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_pos_pdc_internal_parity.py
    )
    add_test(
        NAME python_taroz_pos_vel_pdc_internal_parity_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_pos_vel_pdc_internal_parity.py
    )
    add_test(
        NAME python_taroz_pos_vel_amb_pdc_factor_parity_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_pos_vel_amb_pdc_factor_parity.py
    )
    add_test(
        NAME python_taroz_pos_vel_amb_pdc_window_cost_parity_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_pos_vel_amb_pdc_window_cost_parity.py
    )
    add_test(
        NAME python_taroz_pos_vel_amb_pdc_ppc_window_parity_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_pos_vel_amb_pdc_ppc_window_parity.py
    )
    add_test(
        NAME python_taroz_pos_vel_amb_pdc_dogfood_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_pos_vel_amb_pdc_dogfood.py
    )
    add_test(
        NAME python_taroz_pd_internal_parity_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_taroz_pd_internal_parity.py
    )
    add_test(
        NAME python_packaging_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_packaging.py
    )
    add_test(
        NAME python_release_contract_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_release_contracts.py
    )
    add_test(
        NAME python_growth_snapshot_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_growth_snapshot.py
    )
    add_test(
        NAME python_community_health_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_community_health.py
    )
    add_test(
        NAME python_bindings_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_python_bindings.py
    )
    add_test(
        NAME python_web_ui_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_web_ui.py
    )
    add_test(
        NAME python_web_http_ux_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_web_http_ux.py
    )
    add_test(
        NAME python_docs_site_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_docs_site.py
    )
    add_test(
        NAME python_use_case_guide_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_use_case_guides.py
    )
    add_test(
        NAME python_urban_bridge_score_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_urban_bridge_score.py
    )
    add_test(
        NAME python_urban_continuity_bundle_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_urban_continuity_bundle.py
    )
    add_test(
        NAME python_japan_static_survey_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_japan_static_survey.py
    )
    add_test(
        NAME python_structural_displacement_signoff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_structural_displacement_signoff.py
    )
    add_test(
        NAME python_integrity_geofence_signoff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_integrity_geofence_signoff.py
    )
    add_test(
        NAME python_timing_holdover_signoff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_timing_holdover_signoff.py
    )
    add_test(
        NAME python_trajectory_bundle_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_trajectory_bundle.py
    )
    add_test(
        NAME python_clas_application_decision_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_clas_application_decision.py
    )
    add_test(
        NAME python_experiment_runner_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_experiment_runner.py
    )
    add_test(
        NAME python_search_imu_time_offset_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_search_imu_time_offset.py
    )
    add_test(
        NAME python_gnss_fuse_cli_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_gnss_fuse_cli.py
    )
    add_test(
        NAME python_gnss_solve_cli_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_gnss_solve_cli.py
    )
    add_test(
        NAME python_observable_native_cli_contract_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_observable_native_cli_contracts.py
    )
    set_tests_properties(
        python_gnss_fuse_cli_tests
        python_gnss_solve_cli_tests
        python_observable_native_cli_contract_tests
        PROPERTIES ENVIRONMENT "GNSSPP_BINARY_DIR=${CMAKE_BINARY_DIR}"
    )
    set_tests_properties(
        python_bindings_tests
        PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}/python"
    )
    set_tests_properties(
        python_packaging_tests
        PROPERTIES ENVIRONMENT "GNSSPP_BINARY_DIR=${CMAKE_BINARY_DIR}"
    )
    if(TARGET gnss_solution_node)
        add_test(
            NAME python_ros2_node_tests
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_ros2_node.py
        )
        set_tests_properties(
            python_ros2_node_tests
            PROPERTIES ENVIRONMENT "GNSSPP_BINARY_DIR=${CMAKE_BINARY_DIR}"
        )
    endif()

    set(GNSSPP_CORE_TESTS
        run_tests
        clas_outage_legacy_kill_switch
        python_fgo_arc_lifecycle_audit_tests
        python_fgo_external_dr_witness_tests
        python_ppc_integrity_policy_tests
        python_ppc_goal_completion_audit_tests
        python_artifact_manifest_contract_validator_tests
        python_artifact_manifest_ux_tests
        python_clas_a4b_native_selfdiff_tests
        python_clas_filter_state_diff_tests
        python_clas_zd_component_diff_tests
        python_clas_zd_component_summary_tests
        python_claslib_parity_scorecard_tests
        python_claslib_osr_zd_export_tests
        python_claslib_zd_component_export_tests
        python_cli_json_contract_validator_tests
        python_cli_ux_tests
        python_optional_clas_zd_component_diff_tests
        python_qzss_l6_bias_tests
        python_madoca_materialization_diff_tests
        python_madoca_materialization_selfdiff_tests
        python_madoca_materialization_summary_tests
        python_madoca_residual_component_diff_tests
        python_madoca_residual_component_summary_tests
        python_madoca_solution_diff_tests
        python_madoca_satellite_set_diff_tests
        python_mode_compare_tests
        python_performance_report_tests
        python_smartphone_quality_report_tests
        python_smartphone_raw_quality_control_tests
        python_smartphone_raw_quality_control_eval_tests
        python_smartphone_kaggle_tests
        python_smartphone_trajectory_smoother_tests
        python_smartphone_trajectory_imu_tests
        python_smartphone_imu_motion_q_eval_tests
        python_smartphone_observable_error_correction_tests
        python_smartphone_doppler_position_tests
        python_smartphone_reacquisition_eval_tests
        python_smartphone_reacquisition_conservative_eval_tests
        python_smartphone_segment_stability_tests
        python_smartphone_segment_stability_eval_tests
        python_smartphone_wls_tests
        python_smartphone_wls_eval_tests
        python_smartphone_wls_device_family_eval_tests
        python_smartphone_wls_stability_selector_tests
        python_smartphone_wls_stability_selector_eval_tests
        python_smartphone_wls_stability_selector_holdout_eval_tests
        python_smartphone_wls_multi_phone_ensemble_tests
        python_smartphone_wls_multi_phone_ensemble_holdout_tests
        python_smartphone_native_fgo_eval_tests
        python_smartphone_native_fgo_holdout_tests
        python_smartphone_native_fgo_test_batch_tests
        python_smartphone_native_fgo_v2_smoke_tests
        python_smartphone_native_gnss_pdc_bridge_tests
        python_smartphone_native_gnss_pdc_eval_tests
        python_smartphone_native_fgo_test_submission_recovery_tests
        python_smartphone_native_fgo_test_submission_continuity_tests
        python_smartphone_native_fgo_test_submission_source_seam_bridge_tests
        python_smartphone_native_fgo_optimizer_stop_tests
        python_smartphone_native_fgo_convergence_selector_tests
        python_smartphone_native_fgo_float_carrier_tests
        python_smartphone_native_fgo_carrier_compatibility_tests
        python_smartphone_native_fgo_pdc_wls_tests
        python_smartphone_native_fgo_pdc_windowed_tests
        python_smartphone_native_fgo_pdc_windowed_stitch_tests
        python_optional_madoca_materialization_diff_tests
        python_optional_madoca_residual_component_diff_tests
        python_ppp_residual_row_set_diff_tests
        python_spp_residual_dump_summary_tests
        python_spp_iteration_dump_summary_tests
        python_web_http_ux_tests
        python_experiment_runner_tests
        python_gnss_fuse_cli_tests
        python_gnss_solve_cli_tests
        python_observable_native_cli_contract_tests
        python_search_imu_time_offset_tests
        python_use_case_guide_tests
        python_urban_bridge_score_tests
        python_urban_continuity_bundle_tests
        python_japan_static_survey_tests
        python_structural_displacement_signoff_tests
        python_integrity_geofence_signoff_tests
        python_timing_holdover_signoff_tests
        python_release_contract_tests
        python_growth_snapshot_tests
        python_community_health_tests
    )
    set(GNSSPP_EXTENDED_TESTS
        python_packaging_tests
        python_bindings_tests
        python_web_ui_tests
        python_docs_site_tests
    )
    set(GNSSPP_OPTIONAL_TESTS
        python_benchmark_tests
        python_cli_tests
        python_ppc_taroz_amb_pdc_smoke_tests
        python_taroz_pc_internal_parity_tests
        python_taroz_pc_dogfood_tests
        python_taroz_p_dogfood_tests
        python_taroz_pd_dogfood_tests
        python_taroz_observable_dogfood_tests
        python_taroz_oracle_suite_tests
        python_taroz_p_internal_parity_tests
        python_taroz_d_internal_parity_tests
        python_taroz_pos_pd_internal_parity_tests
        python_taroz_pos_pdc_internal_parity_tests
        python_taroz_pos_vel_pdc_internal_parity_tests
        python_taroz_pos_vel_amb_pdc_factor_parity_tests
        python_taroz_pos_vel_amb_pdc_window_cost_parity_tests
        python_taroz_pos_vel_amb_pdc_ppc_window_parity_tests
        python_taroz_pos_vel_amb_pdc_dogfood_tests
        python_taroz_pd_internal_parity_tests
    )
    if(TARGET gnss_solution_node)
        list(APPEND GNSSPP_EXTENDED_TESTS python_ros2_node_tests)
    endif()
    set_tests_properties(${GNSSPP_CORE_TESTS} PROPERTIES LABELS core)
    set_tests_properties(${GNSSPP_EXTENDED_TESTS} PROPERTIES LABELS extended)
    set_tests_properties(${GNSSPP_OPTIONAL_TESTS} PROPERTIES LABELS optional)

else()
    message(WARNING "Google Test not found. Tests will not be built.")
endif()
