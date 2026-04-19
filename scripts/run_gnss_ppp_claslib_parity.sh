#!/usr/bin/env bash
#
# 日本語要約:
#   「CLASLIB寄りのPPP設定」を一発で付けた gnss_ppp ラッパ。既定は native CLASNAT path。
#   以降の引数はそのまま gnss_ppp に渡る（--obs / --out / --ssr などは呼び出し側で付ける）。
#
# Run gnss_ppp with --claslib-parity (strict CLAS OSR preset, native CLASNAT by default).
# Pass all solver arguments after this wrapper. Add --legacy-strict-parity or --no-ported-clasnat
# after the wrapper arguments only when comparing against the older strict path.
#
# Bin lookup (first set wins):
#   GNSS_PPP, GNSS_PPP_BIN, or <repo>/build/gnss_ppp
#
# Example:
#   export GNSS_PPP="$PWD/build/gnss_ppp"
#   ./scripts/run_gnss_ppp_claslib_parity.sh \
#     --obs rover.obs --nav brdc.nav --ssr corrections.csv \
#     --out parity.pos --summary-json parity_summary.json
#
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="${GNSS_PPP:-${GNSS_PPP_BIN:-$ROOT/build/gnss_ppp}}"
if [[ ! -x "$BIN" && -x "$ROOT/build/apps/gnss_ppp" ]]; then
  BIN="$ROOT/build/apps/gnss_ppp"
fi
if [[ ! -x "$BIN" ]]; then
  echo "gnss_ppp not executable: $BIN (set GNSS_PPP or build the target)" >&2
  exit 1
fi
exec "$BIN" --claslib-parity "$@"
