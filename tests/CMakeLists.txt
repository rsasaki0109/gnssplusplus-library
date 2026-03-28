# Tests CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

# Find Google Test
find_package(GTest REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

if(GTest_FOUND)
    # Consolidate all tests into a single executable
    add_executable(run_tests
        test_ppp_atmosphere.cpp
        test_rtk_ar_evaluation.cpp
        test_rtk_ar_selection.cpp
        test_rtk_measurement.cpp
        test_rtk_update.cpp
        test_rtk_selection.cpp
        test_rtk_validation.cpp
        test_signal_policy.cpp
        test_types.cpp
        test_observation.cpp
        test_gnss_live.cpp
        test_ppp.cpp
        test_rinex_writer.cpp
        test_rtk_legacy.cpp
        test_spp.cpp
        test_rtk_smoke.cpp
        test_ntrip.cpp
        test_rtcm.cpp
        test_rtcm_stream.cpp
        test_ubx.cpp
    )

    target_compile_options(run_tests PRIVATE -g)
    target_compile_definitions(run_tests PRIVATE
        GNSSPP_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
        GNSSPP_BINARY_DIR="${CMAKE_BINARY_DIR}"
    )
    target_link_libraries(run_tests
        gnss_lib gnss_lib_noopt
        GTest::GTest
        GTest::Main
        Threads::Threads
    )
    add_dependencies(run_tests
        gnss_convert
        gnss_live
        gnss_ppp
        gnss_replay
        gnss_rinex_info
        gnss_solve
        gnss_stream
        gnss_ubx_info
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
    set_tests_properties(
        python_bindings_tests
        PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}/python"
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

else()
    message(WARNING "Google Test not found. Tests will not be built.")
endif()
