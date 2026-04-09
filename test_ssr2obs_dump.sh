#!/bin/bash
#
# Test SSR2OBS dump functionality
#
set -euo pipefail

echo "Testing SSR2OBS dump functionality..."

# Test parameters
OBS_FILE=""
NAV_FILE=""
SSR_FILE=""
OUTPUT_FILE="output/test_ssr2obs_dump.pos"
DUMP_EPOCH=50

echo "Running gnss_ppp with SSR2OBS dump for epoch $DUMP_EPOCH..."

./build/apps/gnss_ppp \
  --claslib-parity \
  --obs "$OBS_FILE" \
  --nav "$NAV_FILE" \
  --ssr "$SSR_FILE" \
  --out "$OUTPUT_FILE" \
  --dump-ssr2obs-epoch $DUMP_EPOCH \
  --max-epochs 100 \
  --quiet

echo "SSR2OBS dump test completed."
echo "Output written to: $OUTPUT_FILE"