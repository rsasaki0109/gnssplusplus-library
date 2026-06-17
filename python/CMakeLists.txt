set(GNSSPP_PYTHON_PACKAGE_DIR ${CMAKE_BINARY_DIR}/python/libgnsspp)
set(GNSSPP_PYTHON_INSTALL_DIR
    ${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/libgnsspp
)
set(GNSSPP_PYTHON_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/libgnsspp/__init__.py
    ${CMAKE_CURRENT_SOURCE_DIR}/libgnsspp/artifacts.py
)

file(MAKE_DIRECTORY ${GNSSPP_PYTHON_PACKAGE_DIR})
foreach(GNSSPP_PYTHON_FILE ${GNSSPP_PYTHON_FILES})
    get_filename_component(GNSSPP_PYTHON_FILE_NAME ${GNSSPP_PYTHON_FILE} NAME)
    configure_file(
        ${GNSSPP_PYTHON_FILE}
        ${GNSSPP_PYTHON_PACKAGE_DIR}/${GNSSPP_PYTHON_FILE_NAME}
        COPYONLY
    )
endforeach()

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
            ${GNSSPP_PYTHON_FILES}
            ${GNSSPP_PYTHON_PACKAGE_DIR}
)

install(
    TARGETS _libgnsspp
    LIBRARY DESTINATION ${GNSSPP_PYTHON_INSTALL_DIR}
    RUNTIME DESTINATION ${GNSSPP_PYTHON_INSTALL_DIR}
)
install(
    FILES ${GNSSPP_PYTHON_FILES}
    DESTINATION ${GNSSPP_PYTHON_INSTALL_DIR}
)
