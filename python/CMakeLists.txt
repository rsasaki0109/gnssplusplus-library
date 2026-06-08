set(GNSSPP_PYTHON_PACKAGE_DIR ${CMAKE_BINARY_DIR}/python/libgnsspp)
set(GNSSPP_PYTHON_INSTALL_DIR
    ${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/libgnsspp
)

file(MAKE_DIRECTORY ${GNSSPP_PYTHON_PACKAGE_DIR})
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/libgnsspp/__init__.py
    ${GNSSPP_PYTHON_PACKAGE_DIR}/__init__.py
    COPYONLY
)

pybind11_add_module(_libgnsspp MODULE libgnsspp/bindings.cpp)
target_link_libraries(_libgnsspp PRIVATE gnss_lib gnss_lib_noopt)
target_include_directories(_libgnsspp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libgnsspp)
set_target_properties(_libgnsspp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${GNSSPP_PYTHON_PACKAGE_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${GNSSPP_PYTHON_PACKAGE_DIR}
)
add_custom_command(
    TARGET _libgnsspp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/libgnsspp/__init__.py
            ${GNSSPP_PYTHON_PACKAGE_DIR}/__init__.py
)

install(
    TARGETS _libgnsspp
    LIBRARY DESTINATION ${GNSSPP_PYTHON_INSTALL_DIR}
    RUNTIME DESTINATION ${GNSSPP_PYTHON_INSTALL_DIR}
)
install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/libgnsspp/__init__.py
    DESTINATION ${GNSSPP_PYTHON_INSTALL_DIR}
)
