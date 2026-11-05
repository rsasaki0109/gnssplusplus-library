# WebAssembly builds of the batch CLI tools for the browser GUI in web/.
#
# Only reached from the root CMakeLists.txt when the Emscripten toolchain is
# active (see wasm/README.md):
#
#   emcmake cmake -S . -B build-wasm -DCMAKE_BUILD_TYPE=Release \
#       -DEigen3_DIR=/usr/share/eigen3/cmake
#   cmake --build build-wasm --target gnsspp_wasm -j$(nproc)
#
# Outputs land in build-wasm/wasm/dist/. Each tool's main() is compiled
# unchanged from apps/native/, so a browser run is the same program as the
# native CLI. web/ drives it with Module.callMain() after mounting the input
# files in the Emscripten virtual filesystem; every run gets a fresh module
# instance, so global state never leaks between runs.

set(GNSSPP_NATIVE_APP_DIR ${CMAKE_SOURCE_DIR}/apps/native)

set(GNSSPP_WASM_LINK_OPTIONS
    # ES module factory (createModule()) that does not run main() on load.
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sINVOKE_RUN=0
    # Normal process semantics for each run: atexit handlers, static
    # destructors and a final stdio flush happen when main() returns.
    -sEXIT_RUNTIME=1
    -sENVIRONMENT=web,worker,node
    -sEXPORTED_RUNTIME_METHODS=callMain,FS
    -sFORCE_FILESYSTEM=1
    # Lets the worker mount browser File objects read-only instead of
    # copying multi-hundred-MB RINEX files into the wasm heap.
    -lworkerfs.js
    -sALLOW_MEMORY_GROWTH=1
    -sINITIAL_MEMORY=64MB
    -sMAXIMUM_MEMORY=4GB
    # The solvers keep Eigen fixed-size matrices on the stack; Emscripten's
    # 64 KiB default is far below the native 8 MiB the tools are tested with.
    -sSTACK_SIZE=8MB
)

set(GNSSPP_WASM_DIST_DIR ${CMAKE_BINARY_DIR}/wasm/dist)
file(WRITE ${GNSSPP_WASM_DIST_DIR}/package.json "{ \"type\": \"module\" }\n")

set(GNSSPP_WASM_TARGETS)

# gnsspp_add_wasm_tool(<name> SOURCES <files...> LIBS <targets...>)
# <name> is the native tool name; the output is <name>.js (an ES module) +
# <name>.wasm. The .js suffix (not .mjs) keeps every static host, GitHub Pages
# included, serving it with a JavaScript MIME type; the package.json written
# above tells Node to load the same files as ES modules.
function(gnsspp_add_wasm_tool name)
    cmake_parse_arguments(ARG "" "" "SOURCES;LIBS" ${ARGN})
    set(target ${name}_wasm)
    add_executable(${target} ${ARG_SOURCES})
    # Deliberately no Threads::Threads: a -pthread build needs
    # SharedArrayBuffer (cross-origin isolation headers), which static hosts
    # such as GitHub Pages cannot send. std::thread still links (and is never
    # started on the batch file paths these tools take).
    target_link_libraries(${target} PRIVATE ${ARG_LIBS})
    target_link_options(${target} PRIVATE
        ${GNSSPP_WASM_LINK_OPTIONS}
        "-sEXPORT_NAME=create_${name}")
    set_target_properties(${target} PROPERTIES
        OUTPUT_NAME ${name}
        SUFFIX ".js"
        RUNTIME_OUTPUT_DIRECTORY ${GNSSPP_WASM_DIST_DIR})
    set(GNSSPP_WASM_TARGETS ${GNSSPP_WASM_TARGETS} ${target} PARENT_SCOPE)
endfunction()

# Link sets mirror apps/CMakeLists.txt for the same executables.
gnsspp_add_wasm_tool(gnss_convert
    SOURCES ${GNSSPP_NATIVE_APP_DIR}/gnss_convert.cpp
    LIBS gnss_lib gnss_lib_solvers)
gnsspp_add_wasm_tool(gnss_rinex_info
    SOURCES ${GNSSPP_NATIVE_APP_DIR}/gnss_rinex_info.cpp
    LIBS gnss_lib gnss_lib_solvers)
gnsspp_add_wasm_tool(gnss_spp
    SOURCES ${GNSSPP_NATIVE_APP_DIR}/gnss_spp.cpp
    LIBS gnss_lib gnss_lib_solvers)
gnsspp_add_wasm_tool(gnss_visibility
    SOURCES ${GNSSPP_NATIVE_APP_DIR}/gnss_visibility.cpp
    LIBS gnss_lib gnss_lib_solvers)
gnsspp_add_wasm_tool(gnss_solve
    SOURCES
        ${GNSSPP_NATIVE_APP_DIR}/gnss_solve.cpp
        ${GNSSPP_NATIVE_APP_DIR}/cli_toml_config.cpp
    LIBS gnss_lib gnss_rtk_runtime)
gnsspp_add_wasm_tool(gnss_fuse
    SOURCES
        ${GNSSPP_NATIVE_APP_DIR}/gnss_fuse.cpp
        ${GNSSPP_NATIVE_APP_DIR}/cli_toml_config.cpp
    LIBS gnss_lib gnss_rtk_runtime)
gnsspp_add_wasm_tool(gnss_ppp
    SOURCES
        ${GNSSPP_NATIVE_APP_DIR}/gnss_ppp.cpp
        ${GNSSPP_NATIVE_APP_DIR}/cli_toml_config.cpp
    LIBS gnss_lib gnss_lib_solvers)

add_custom_target(gnsspp_wasm)
add_dependencies(gnsspp_wasm ${GNSSPP_WASM_TARGETS})
