# Tests CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

# Find Google Test
find_package(GTest REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_test(NAME python_cli_argument_sequencing_tests
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_cli_argument_sequencing.py)
add_test(NAME python_gsdc_native_output_audit_tests
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_gsdc_native_output_audit.py)
add_test(NAME python_gsdc_native_submission_tests
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_gsdc_native_submission.py)

add_test(NAME python_fixed_lag_covariance_comparison_tests
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_fixed_lag_covariance_comparison.py)
set_tests_properties(python_fixed_lag_covariance_comparison_tests PROPERTIES LABELS core)

if(GTest_FOUND)
    # Consolidate all tests into a single executable.
    # The target is named gnss_run_tests (not run_tests): with the Visual
    # Studio generator, CMake emits a per-directory RUN_TESTS utility whose
    # project file collides case-insensitively with run_tests.vcxproj on
    # Windows. OUTPUT_NAME keeps the binary named run_tests for scripts/CI.
    add_executable(gnss_run_tests
        test_imu_csv.cpp
        test_android_raw_gnss.cpp
        test_barometer_height.cpp
        test_native_imu_refinement_handoff.cpp
        test_native_gnss_imu_initial_handoff.cpp
        test_native_imu_refinement_attitude_reset.cpp
        test_native_stage_position_offset.cpp
        test_native_gnss_initial_observations.cpp
        test_source_route_settings.cpp
        test_source_phone_imu_noise.cpp
        test_source_samsung_clock_drift.cpp
        test_source_mi8_clock_drift.cpp
        test_carrier_code_leveling.cpp
        test_smartphone_upstream_preprocessing.cpp
        test_smartphone_upstream_state_contract.cpp
        test_attitude.cpp
        test_mechanization.cpp
        test_preintegration.cpp
        test_tight_coupling_processor.cpp
        test_fusion_process_noise.cpp
        test_fusion_update.cpp
        test_fusion_measurement.cpp
        test_fusion_zupt.cpp
        test_fusion_nhc.cpp
        test_upstream_stop_constraints.cpp
        test_stationary_gyro_initializer.cpp
        test_source_tdcp_drift_factor.cpp
        test_tdcp_code_gate_ablation.cpp
        test_sparse_epoch_rank_control.cpp
        test_dual_frequency_tdcp_observability.cpp
        test_tdcp_residual_state_factor.cpp
        test_tdcp_frequency_pairing.cpp
        test_pseudorange_position_information.cpp
        test_ionosphere_temporal_plan.cpp
        test_joint_ionosphere_constant_mode.cpp
        test_geometry_free_code_carrier_closure.cpp
        test_joint_ionosphere_position_bias.cpp
        test_frozen_earth_rotation.cpp
        test_doppler_rotation_derivative.cpp
        test_gps_triple_code_closure.cpp
        test_adjacent_residual_moments.cpp
        test_tdcp_temporal_correlation_controls.cpp
        test_tdcp_endpoint_covariance.cpp
        test_rejected_residual_summary.cpp
        test_native_nhc_gate.cpp
        test_code_edge_candidates.cpp
        test_matched_pseudorange_doppler.cpp
        test_upstream_position_offset.cpp
        test_galileo_group_delay.cpp
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
        test_rtk_covariance_consistency.cpp
        test_rtk_tdcp_diagnostics.cpp
        test_rtk_selection.cpp
        test_rtk_cmc_ref.cpp
        test_rtk_cp_pr_gate.cpp
        test_rtk_ddpr_anchor.cpp
        test_rtk_validation.cpp
        test_integrity_consensus.cpp
        test_fgo.cpp
        test_raw_p_seed.cpp
        test_base_pseudorange_compensation.cpp
        test_phase127_glonass_channel_provenance.cpp
        test_phase128_glonass_provenance.cpp
        test_phase129_glonass_local_miss.cpp
        test_phase131_canonical_correction_key.cpp
        test_phase134_native_phase131_diagnostics_bridge.cpp
        test_native_direct_wls_ephemeral_seed.cpp
        test_signal_policy.cpp
        test_types.cpp
        test_observation.cpp
        test_gnss_live.cpp
        test_ppp.cpp
        test_rinex_writer.cpp
        test_rinex4_navigation.cpp
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
        test_galileo_has.cpp
        test_sofa_vendor.cpp
        test_ginan_iers2010_vendor.cpp
        test_iers_earth_rotation.cpp
        test_iers_tides.cpp
        test_iers_ephemeris.cpp
        test_iers_eop.cpp
        test_iers_sub_daily_eop.cpp
    )

    # test_rtk_legacy.cpp reaches RTKProcessor internals through the
    # befriended RTKProcessorTestAccess (not "#define private public", which
    # cannot link under the MSVC ABI), so it builds on every toolchain.
    target_sources(gnss_run_tests PRIVATE test_rtk_legacy.cpp)

    # GTSAM FGO backend parity tests (guarded at both CMake and #ifdef level:
    # the source itself is a no-op TU when GNSSPP_HAS_GTSAM isn't defined, but
    # we still only add it to the build when GTSAM was actually found so a
    # GTSAM-less build doesn't pay for an always-empty translation unit).
    if(GTSAM_FOUND)
        target_sources(gnss_run_tests PRIVATE test_doppler_rotation_rate_factor.cpp)
        target_sources(gnss_run_tests PRIVATE test_code_ionosphere_state_factor.cpp)
        target_sources(gnss_run_tests PRIVATE test_joint_ionosphere_scope.cpp)
        target_sources(gnss_run_tests PRIVATE test_ionosphere_huber_information.cpp)
        target_sources(gnss_run_tests PRIVATE test_fgo_gtsam_backend.cpp)
        target_sources(gnss_run_tests PRIVATE test_native_lm_lambda_floor.cpp)
        target_sources(gnss_run_tests PRIVATE test_pseudorange_cauchy.cpp)
        target_sources(gnss_run_tests PRIVATE test_gtsam_imu_time_indexing.cpp)
    endif()

    set_target_properties(gnss_run_tests PROPERTIES OUTPUT_NAME run_tests)
    # Backward-compatible alias for CI/scripts that build --target run_tests.
    # Skipped for Visual Studio generators, where the name collides with
    # CMake's auto-generated RUN_TESTS utility on a case-insensitive FS, and
    # for Ninja on non-Windows hosts, where the alias's phony tests/run_tests
    # collides with the extension-less executable ("multiple rules generate
    # tests/run_tests"); there `--target tests/run_tests` builds the binary.
    if(NOT CMAKE_GENERATOR MATCHES "Visual Studio" AND
       NOT (CMAKE_GENERATOR MATCHES "Ninja" AND NOT WIN32))
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
        gnss_doppler_contract_audit
        gnss_doppler_row_attrition_audit
        gnss_stream
        gnss_ubx_info
        gnss_has_info
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

    add_executable(gnss_online_tests test_online_rtk_imu.cpp)
    target_link_libraries(gnss_online_tests PRIVATE
        gnss_lib gnss_rtk_runtime GTest::GTest GTest::Main Threads::Threads)
    add_test(NAME online_rtk_imu_tests COMMAND gnss_online_tests)
    add_test(NAME python_pva_tests COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_pva.py)
    add_test(NAME python_pva_comparison_tests COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_pva_comparison.py)
    set_tests_properties(online_rtk_imu_tests PROPERTIES LABELS core)

    add_executable(gnss_rtcm_context_tests test_rtcm_time_context.cpp)
    target_link_libraries(gnss_rtcm_context_tests PRIVATE gnss_lib gnss_rtk_runtime GTest::GTest GTest::Main Threads::Threads)
    add_test(NAME rtcm_time_context_tests COMMAND gnss_rtcm_context_tests)
    set_tests_properties(rtcm_time_context_tests PROPERTIES LABELS core)

    # Explicit raw PPC staging/audit harness; no external data is substituted
    # for the independent historical fixtures of other test targets.
    add_executable(gnss_online_ppc_fixture ppc_online_fixture.cpp)
    target_link_libraries(gnss_online_ppc_fixture PRIVATE gnss_lib gnss_rtk_runtime Threads::Threads)

    add_executable(gnss_fix_recovery_tests test_fix_recovery_guard.cpp)
    target_include_directories(gnss_fix_recovery_tests PRIVATE ${CMAKE_SOURCE_DIR}/include)
    target_link_libraries(gnss_fix_recovery_tests PRIVATE Eigen3::Eigen GTest::GTest GTest::Main)
    add_test(NAME fix_recovery_guard_tests COMMAND gnss_fix_recovery_tests)
    set_tests_properties(fix_recovery_guard_tests PROPERTIES LABELS core)

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
        NAME python_gnss_reproduce_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_gnss_reproduce.py
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
        NAME python_smartphone_tdcp_trajectory_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_tdcp_trajectory.py
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
        NAME python_smartphone_wls_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls.py
    )
    add_test(
        NAME python_smartphone_wls_eval_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_eval.py
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
        NAME python_smartphone_wls_multi_phone_ensemble_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_multi_phone_ensemble.py
    )
    add_test(
        NAME python_smartphone_wls_test_batch_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_wls_test_batch.py
    )
    add_test(
        NAME python_smartphone_native_fgo_pdc_factor_hypotheses_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_native_fgo_pdc_factor_hypotheses.py
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
    # Pure-Python unit tests (no build artifacts, datasets or network).
    # Registered as python_<module>_tests and labelled core below.
    set(GNSSPP_PURE_PYTHON_TEST_MODULES
        convert_gici_nmea_to_pos
        fgo_motion_constraint_ab
        gnss_next
        gsdc_evaluation_grouping
        gsdc_frozen_runner
        gsdc_raw_clock_cleanup_audit
        integrity_consensus_replay
        observation_analysis
        native_integrity_audit
        ppc_clas_scorecard
        ppc_kinematic_integrity_loo
        ppc_native_replay
        ppc_residual_integrity_policy
        ppc_status_demotion
        ppc_wrong_fix_residuals
        sinex
        smartphone_gnss_adapter
        smartphone_gnss_signoff
        smartphone_gnss_workflow
        smartphone_mimir_adapter
        static_metrics
        uav_mars_acquire
        uav_mars_adapter
        uav_mars_signoff
        uav_mars_workflow
    )
    set(GNSSPP_PURE_PYTHON_TESTS)
    foreach(module IN LISTS GNSSPP_PURE_PYTHON_TEST_MODULES)
        add_test(
            NAME python_${module}_tests
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_${module}.py
        )
        list(APPEND GNSSPP_PURE_PYTHON_TESTS python_${module}_tests)
    endforeach()

    # These compile a helper extracted from apps/native/gnss_fgo_imu_no_base.cpp
    # with the host C++ compiler against the public headers and system Eigen.
    if(NOT WIN32)
        foreach(module IN ITEMS native_tdcp_correction_diagnostics native_tdcp_huber_reconstruction)
            add_test(
                NAME python_${module}_tests
                COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_${module}.py
            )
            set_tests_properties(
                python_${module}_tests
                PROPERTIES ENVIRONMENT "CXX=${CMAKE_CXX_COMPILER}"
            )
            list(APPEND GNSSPP_PURE_PYTHON_TESTS python_${module}_tests)
        endforeach()
    endif()

    # CLI admission tests for the GTSAM-only no-base smartphone research binary.
    # They are pytest-style (parametrized) and need the built executable.
    if(TARGET gnss_fgo_imu_no_base)
        execute_process(
            COMMAND ${Python3_EXECUTABLE} -c "import pytest"
            RESULT_VARIABLE GNSSPP_PYTEST_IMPORT_RESULT
            OUTPUT_QUIET ERROR_QUIET
        )
        set(GNSSPP_NO_BASE_CLI_TESTS)
        add_test(
            NAME python_smartphone_paired_epoch_states_cli_tests
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_smartphone_paired_epoch_states_cli.py
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
        list(APPEND GNSSPP_NO_BASE_CLI_TESTS python_smartphone_paired_epoch_states_cli_tests)
        if(GNSSPP_PYTEST_IMPORT_RESULT EQUAL 0)
            foreach(module IN ITEMS
                    smartphone_epoch_heading_seeds
                    smartphone_include_first_native_epoch
                    smartphone_omit_first_bias_prior
                    smartphone_omit_first_velocity_prior
                    smartphone_pseudorange_remasking
                    smartphone_relative_height_pairs
                    smartphone_source_tdcp_meter_sigma
                    smartphone_source_tdcp_resl_observable
                    smartphone_tdcp_only_affine_geometry)
                add_test(
                    NAME python_${module}_tests
                    COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider
                            ${CMAKE_CURRENT_SOURCE_DIR}/test_${module}.py
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                )
                list(APPEND GNSSPP_NO_BASE_CLI_TESTS python_${module}_tests)
            endforeach()
        endif()
        set_tests_properties(
            ${GNSSPP_NO_BASE_CLI_TESTS}
            PROPERTIES ENVIRONMENT "GSDC_NATIVE_EXE=$<TARGET_FILE:gnss_fgo_imu_no_base>"
                       LABELS optional
        )
    endif()

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
        python_smartphone_kaggle_tests
        python_smartphone_trajectory_smoother_tests
        python_smartphone_trajectory_imu_tests
        python_smartphone_observable_error_correction_tests
        python_smartphone_doppler_position_tests
        python_smartphone_reacquisition_eval_tests
        python_smartphone_reacquisition_conservative_eval_tests
        python_smartphone_segment_stability_tests
        python_smartphone_wls_tests
        python_smartphone_wls_eval_tests
        python_smartphone_wls_stability_selector_tests
        python_smartphone_wls_stability_selector_eval_tests
        python_smartphone_wls_multi_phone_ensemble_tests
        python_smartphone_native_gnss_pdc_bridge_tests
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
    list(APPEND GNSSPP_CORE_TESTS ${GNSSPP_PURE_PYTHON_TESTS})
    set_tests_properties(${GNSSPP_CORE_TESTS} PROPERTIES LABELS core)
    set_tests_properties(${GNSSPP_EXTENDED_TESTS} PROPERTIES LABELS extended)
    set_tests_properties(${GNSSPP_OPTIONAL_TESTS} PROPERTIES LABELS optional)

else()
    message(WARNING "Google Test not found. Tests will not be built.")
endif()
