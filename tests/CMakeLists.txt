add_executable(rtklib2_sanity
    rtklib2_sanity.cpp
)

target_link_libraries(rtklib2_sanity PRIVATE rtklib2)

add_test(NAME rtklib2_sanity COMMAND rtklib2_sanity)
