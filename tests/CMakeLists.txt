# Tests CMakeLists.txt

# Find Google Test
find_package(GTest QUIET)

if(GTest_FOUND)
    # Core tests
    add_executable(test_core
        test_types.cpp
        test_observation.cpp
        test_navigation.cpp
        test_solution.cpp
    )
    target_link_libraries(test_core libgnss++ GTest::gtest GTest::gtest_main)
    
    # Algorithm tests
    add_executable(test_algorithms
        test_spp.cpp
        test_rtk.cpp
        test_ppp.cpp
    )
    target_link_libraries(test_algorithms libgnss++ GTest::gtest GTest::gtest_main)
    
    # IO tests
    add_executable(test_io
        test_rinex.cpp
    )
    target_link_libraries(test_io libgnss++ GTest::gtest GTest::gtest_main)
    
    # Add tests to CTest
    add_test(NAME CoreTests COMMAND test_core)
    add_test(NAME AlgorithmTests COMMAND test_algorithms)
    add_test(NAME IOTests COMMAND test_io)
    
    # Create test data directory
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/test_data)
    
else()
    message(WARNING "Google Test not found. Tests will not be built.")
endif()
