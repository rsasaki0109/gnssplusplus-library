#!/usr/bin/env bash
# Build the WebAssembly CLI tools for the browser GUI (see wasm/README.md).
#
#   bash wasm/build.sh [build-dir]        # default: build-wasm
#
# Needs emcmake on PATH (an activated Emscripten SDK, or the emscripten/emsdk
# Docker image) and Eigen 3 headers (EIGEN3_DIR, default the Debian/Ubuntu
# libeigen3-dev location). Outputs land in <build-dir>/wasm/dist.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${1:-${repo_root}/build-wasm}"
eigen_dir="${EIGEN3_DIR:-/usr/share/eigen3/cmake}"

if ! command -v emcmake >/dev/null 2>&1; then
    echo "emcmake not found: activate an Emscripten SDK (emsdk_env.sh) first" >&2
    exit 1
fi
if [ ! -f "${eigen_dir}/Eigen3Config.cmake" ]; then
    echo "Eigen3Config.cmake not found in ${eigen_dir}; set EIGEN3_DIR" >&2
    exit 1
fi

emcmake cmake -S "${repo_root}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release -DEigen3_DIR="${eigen_dir}"
cmake --build "${build_dir}" --target gnsspp_wasm -j"$(nproc 2>/dev/null || echo 2)"
ls -l "${build_dir}/wasm/dist"
