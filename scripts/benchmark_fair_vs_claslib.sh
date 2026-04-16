#!/usr/bin/env bash
#
# 日本語要約:
#   「同じ基準座標 REF に対する誤差」と「LibGNSS と他バイナリの軌道差」をJSON付きで出す。
#   CLASLIB/RTKLIB 側は ECEF の .pos（日時+XYZ）を想定。libgnss 側は gnss_ppp 標準 .pos。
#
# Fair LibGNSS++ vs CLASLIB / rnx2rtkp comparison:
# - Same reference ECEF for RMS (field truth or survey marker).
# - LibGNSS .pos from gnss_ppp; CLASLIB side should export ECEF XYZ .pos from rnx2rtkp
#   (pos1-outsolformat = xyz in the RTKLIB .conf).
#
# CLASLIB-oriented LibGNSS runs: generate the first .pos with gnss_ppp using --claslib-parity
# (see scripts/run_gnss_ppp_claslib_parity.sh), then compare here.
#
# Usage:
#   export REF_X=... REF_Y=... REF_Z=...
#   export LAST_N=100              # optional
#   export JSON_OUT_DIR=/tmp/bench # optional; default /tmp (writes gnsspp_bench_*.json)
#   ./scripts/benchmark_fair_vs_claslib.sh /path/to/libgnss.pos /path/to/claslib_ecef.pos
#
# Single file vs truth only:
#   ./scripts/benchmark_fair_vs_claslib.sh /path/to/libgnss.pos
#
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CMP="$ROOT/tools/compare_ppp_solutions.py"
LAST_N="${LAST_N:-100}"
JSON_OUT_DIR="${JSON_OUT_DIR:-/tmp}"
mkdir -p "$JSON_OUT_DIR"
J_LIB="$JSON_OUT_DIR/gnsspp_bench_libgnss.json"
J_CL="$JSON_OUT_DIR/gnsspp_bench_claslib.json"
J_DF="$JSON_OUT_DIR/gnsspp_bench_diff.json"

if [[ -z "${REF_X:-}" || -z "${REF_Y:-}" || -z "${REF_Z:-}" ]]; then
  echo "Set REF_X, REF_Y, REF_Z (reference ECEF meters) for RMS vs benchmark." >&2
  exit 1
fi

ref_args=(--ecef-x "$REF_X" --ecef-y "$REF_Y" --ecef-z "$REF_Z")

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <libgnss.pos> [rnx2rtkp_ecef.pos]" >&2
  exit 1
fi

cand="$1"
echo "=== LibGNSS++ candidate vs REF ==="
python3 "$CMP" --candidate "$cand" "${ref_args[@]}" --last-n "$LAST_N" --json-out "$J_LIB"

if [[ $# -ge 2 ]]; then
  ref_pos="$2"
  echo ""
  echo "=== CLASLIB / RTKLIB .pos vs REF ==="
  python3 "$CMP" --candidate "$ref_pos" --candidate-format rtklib-ecef "${ref_args[@]}" --last-n "$LAST_N" --json-out "$J_CL"
  echo ""
  echo "=== Trajectory difference (LibGNSS vs CLASLIB .pos, time-aligned) ==="
  python3 "$CMP" --candidate "$cand" --reference "$ref_pos" --reference-format rtklib-ecef --last-n "$LAST_N" --json-out "$J_DF"
  echo "JSON: $J_LIB $J_CL $J_DF"
fi
