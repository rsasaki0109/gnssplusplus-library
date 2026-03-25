# Tests CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

# Find Google Test
find_package(GTest REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

if(GTest_FOUND)
    # Consolidate all tests into a single executable
    add_executable(run_tests
        test_types.cpp
        test_observation.cpp
        test_gnss_live.cpp
        test_rinex_writer.cpp
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

else()
    message(WARNING "Google Test not found. Tests will not be built.")
endif()
