#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${repo_root}"

if ! command -v docker >/dev/null 2>&1; then
    echo "docker is required to run actionlint hygiene locally" >&2
    exit 1
fi

python_bin="${PYTHON:-python3}"

docker run --rm -v "${repo_root}:/repo" -w /repo rhysd/actionlint:latest
"${python_bin}" -m compileall -q apps python scripts tests
