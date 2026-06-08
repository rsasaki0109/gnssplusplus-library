#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
RTKLIB_BIN=${1:-${RTKLIB_RNX2RTKP:-/tmp/RTKLIB/app/rnx2rtkp/gcc/rnx2rtkp}}

if [[ ! -x "${RTKLIB_BIN}" ]]; then
    echo "RTKLIB rnx2rtkp not found: ${RTKLIB_BIN}" >&2
    echo "Pass the binary path as the first argument or set RTKLIB_RNX2RTKP." >&2
    exit 1
fi

cd "${ROOT_DIR}/data/driving"
ln -sf Tokyo_Data/Odaiba/rover_trimble.obs rover.obs
ln -sf Tokyo_Data/Odaiba/base_trimble.obs base.obs
ln -sf Tokyo_Data/Odaiba/base.nav navigation.nav

cd "${ROOT_DIR}"
python3 "${ROOT_DIR}/apps/gnss.py" odaiba-benchmark --rtklib-bin "${RTKLIB_BIN}"
