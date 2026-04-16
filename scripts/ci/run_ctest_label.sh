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

ctest --test-dir "${build_dir}" --output-on-failure -L "${label}" "$@"
