# Tests CMakeLists.txt

# Find Google Test
find_package(GTest QUIET)

if(GTest_FOUND)
    # Core tests – covers types, observation parsing, and SPP algorithm
    add_executable(test_core
        test_types.cpp
        test_observation.cpp
        test_spp.cpp
    )
    target_link_libraries(test_core libgnss++ GTest::gtest GTest::gtest_main)

    add_test(NAME CoreTests COMMAND test_core)
    set_tests_properties(CoreTests PROPERTIES LABELS "core")

    # Create test data directory
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/test_data)

else()
    message(WARNING "Google Test not found. Tests will not be built.")
endif()
