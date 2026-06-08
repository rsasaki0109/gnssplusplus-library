#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ]; then
    echo "usage: $(basename "$0") <label> [ctest args...]" >&2
    exit 2
fi

label="$1"
shift

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${repo_root}"

build_dir="${BUILD_DIR:-build}"

list_output="$(ctest --test-dir "${build_dir}" -N -L "${label}" "$@" 2>&1)"
printf '%s\n' "${list_output}"
test_count="$(awk '/Total Tests:/ {print $3}' <<< "${list_output}" | tail -n 1)"
if [ -z "${test_count}" ]; then
    echo "failed to determine CTest count for label '${label}'" >&2
    exit 2
fi
if [ "${test_count}" -eq 0 ]; then
    echo "no CTest tests matched label '${label}'" >&2
    exit 1
fi

ctest --test-dir "${build_dir}" --output-on-failure -L "${label}" "$@"
