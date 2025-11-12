add_executable(rtklib2_sanity
    rtklib2_sanity.cpp
)

target_link_libraries(rtklib2_sanity PRIVATE rtklib2)

add_test(NAME rtklib2_sanity COMMAND rtklib2_sanity)
add_subdirectory(integration)

add_test(NAME spp_validation COMMAND spp_validation ${CMAKE_SOURCE_DIR}/data/2024-04-25-14-27-26)
set_tests_properties(spp_validation PROPERTIES SKIP_RETURN_CODE 77)
