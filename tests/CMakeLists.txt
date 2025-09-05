# Tests CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

# Find Google Test
find_package(GTest REQUIRED)

if(GTest_FOUND)
    # Consolidate all tests into a single executable
    add_executable(run_tests
        test_types.cpp
        test_observation.cpp
        test_spp.cpp
        test_rtcm.cpp
    )

    target_compile_options(run_tests PRIVATE -g -pthread)
    target_link_libraries(run_tests
        gnss_lib gnss_lib_noopt
        GTest::GTest
        GTest::Main
        Threads::Threads
        -pthread
        stdc++
    )

    # Add test to CTest
    add_test(NAME run_tests COMMAND run_tests)

else()
    message(WARNING "Google Test not found. Tests will not be built.")
endif()
