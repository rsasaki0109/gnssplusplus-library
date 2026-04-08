#!/usr/bin/env bash
# Run gnss_ppp twice (dd-iflc vs dd-wlnl), then compare outputs and optional ECEF reference.
# Usage:
#   ./tools/run_ppp_ar_pair_and_compare.sh rover.obs rover.nav /tmp/ppp_cmp [optional_ssr.csv]
# Optional env:
#   REF_X REF_Y REF_Z  — reference ECEF (m); set to compare vs known truth
#   MAX_EPOCHS         — default 300
#   LAST_N             — default 100 (epochs for RMS block in compare_ppp_solutions.py)
#   QUIET=1            — pass --quiet to gnss_ppp
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="${ROOT}/build/apps/gnss_ppp"
OBS="${1:?obs}"
NAV="${2:?nav}"
OUTDIR="${3:?outdir}"
SSR_ARG=()
if [[ "${4:-}" != "" ]]; then
  SSR_ARG=(--ssr "$4")
fi
MAX_EPOCHS="${MAX_EPOCHS:-300}"
LAST_N="${LAST_N:-100}"
QUIET_FLAG=()
if [[ "${QUIET:-}" == "1" ]]; then
  QUIET_FLAG=(--quiet)
fi

mkdir -p "$OUTDIR"
if [[ ! -x "$BIN" ]]; then
  echo "Build gnss_ppp first: cmake --build build --target gnss_ppp" >&2
  exit 1
fi

run_one() {
  local method="$1"
  local out="$2"
  "$BIN" \
    --obs "$OBS" \
    --nav "$NAV" \
    "${SSR_ARG[@]}" \
    --out "$out" \
    --static \
    --estimate-troposphere \
    --enable-ar \
    --ar-method "$method" \
    --ar-ratio-threshold 2.0 \
    --max-epochs "$MAX_EPOCHS" \
    "${QUIET_FLAG[@]}"
}

echo "=== Run dd-iflc -> ${OUTDIR}/ppp_dd_iflc.pos"
run_one dd-iflc "${OUTDIR}/ppp_dd_iflc.pos"
echo "=== Run dd-wlnl -> ${OUTDIR}/ppp_dd_wlnl.pos"
run_one dd-wlnl "${OUTDIR}/ppp_dd_wlnl.pos"

echo "=== Compare: dd-iflc vs dd-wlnl (matched epochs)"
python3 "${ROOT}/tools/compare_ppp_solutions.py" \
  --candidate "${OUTDIR}/ppp_dd_iflc.pos" \
  --reference "${OUTDIR}/ppp_dd_wlnl.pos" \
  --last-n "$LAST_N" \
  --json-out "${OUTDIR}/compare_iflc_vs_wlnl_trajectory.json"

if [[ -n "${REF_X:-}" && -n "${REF_Y:-}" && -n "${REF_Z:-}" ]]; then
  echo "=== Compare: dd-wlnl vs reference ECEF"
  python3 "${ROOT}/tools/compare_ppp_solutions.py" \
    --candidate "${OUTDIR}/ppp_dd_wlnl.pos" \
    --ecef-x "$REF_X" --ecef-y "$REF_Y" --ecef-z "$REF_Z" \
    --last-n "$LAST_N" \
    --json-out "${OUTDIR}/compare_wlnl_vs_ref_ecef.json"
else
  echo "=== Skip REF_X/Y/Z (set env to compare vs known ECEF)"
fi

echo "Done. Outputs under ${OUTDIR}/"
