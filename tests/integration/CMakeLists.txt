add_executable(spp_validation
    spp_validation.cpp
)

target_link_libraries(spp_validation PRIVATE rtklib2)

target_include_directories(spp_validation PRIVATE
    ${CMAKE_SOURCE_DIR}/legacy/v1/include
)
