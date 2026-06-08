# Tests CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

# Find Google Test
find_package(GTest REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

if(GTest_FOUND)
    # Consolidate all tests into a single executable
    add_executable(run_tests
        test_ppp_clas.cpp
        test_ppp_osr.cpp
        test_ppp_ar.cpp
        test_ppp_atmosphere.cpp
        test_rtk_ar_evaluation.cpp
        test_rtk_ar_selection.cpp
        test_rtk_slip_detection.cpp
        test_rtk_measurement.cpp
        test_rtk_update.cpp
        test_rtk_selection.cpp
        test_rtk_validation.cpp
        test_fgo.cpp
        test_signal_policy.cpp
        test_types.cpp
        test_observation.cpp
        test_gnss_live.cpp
        test_ppp.cpp
        test_rinex_writer.cpp
        test_madoca_parity.cpp
        test_rtk_legacy.cpp
        test_spp.cpp
        test_rtk.cpp
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

    target_compile_options(run_tests PRIVATE -g)
    target_compile_definitions(run_tests PRIVATE
        GNSSPP_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
        GNSSPP_BINARY_DIR="${CMAKE_BINARY_DIR}"
    )
    target_link_libraries(run_tests
        gnss_lib gnss_lib_noopt
        sofa
        ginan_iers2010
        GTest::GTest
        GTest::Main
        Threads::Threads
    )
    add_dependencies(run_tests
        gnss_convert
        gnss_fgo
        gnss_live
        gnss_ppp
        gnss_replay
        gnss_rinex_info
        gnss_solve
        gnss_stream
        gnss_ubx_info
        gnss_vel_d
    )

    # Add test to CTest
    add_test(NAME run_tests COMMAND run_tests)
    add_test(
        NAME python_benchmark_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_benchmark_scripts.py
    )
    add_test(
        NAME python_cli_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_cli_tools.py
    )
    add_test(
        NAME python_madoca_solution_diff_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_madoca_solution_diff.py
    )
    add_test(
        NAME python_mode_compare_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_mode_compare.py
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
        NAME python_bindings_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_python_bindings.py
    )
    add_test(
        NAME python_web_ui_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_web_ui.py
    )
    add_test(
        NAME python_docs_site_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_docs_site.py
    )
    add_test(
        NAME python_experiment_runner_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_experiment_runner.py
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
        python_madoca_solution_diff_tests
        python_mode_compare_tests
        python_experiment_runner_tests
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
