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
./build/examples/rtk_positioning data/driving

"${RTKLIB_BIN}" \
    -k "${SCRIPT_DIR}/rtklib_odaiba.conf" \
    -o "${ROOT_DIR}/output/driving_rtklib_rtk.pos" \
    "${ROOT_DIR}/data/driving/rover.obs" \
    "${ROOT_DIR}/data/driving/base.obs" \
    "${ROOT_DIR}/data/driving/navigation.nav"

python3 "${SCRIPT_DIR}/generate_driving_comparison.py" \
    --lib-pos "${ROOT_DIR}/output/rtk_solution.pos" \
    --rtklib-pos "${ROOT_DIR}/output/driving_rtklib_rtk.pos" \
    --reference-csv "${ROOT_DIR}/data/driving/Tokyo_Data/Odaiba/reference.csv" \
    --output "${ROOT_DIR}/docs/driving_odaiba_comparison.png" \
    --title "Urban Driving Comparison: libgnss++ vs RTKLIB on UrbanNav Odaiba"
