#!/bin/bash
# OSR補正値の詳細デバッグ（1エポック）
# Usage: ./scripts/debug_osr_values.sh [max_epochs]
set -e
GNSSPP=
CLAS=
MAX=${1:-1}

cd $GNSSPP
GNSS_PPP_DEBUG=1 ./build_local/apps/gnss_ppp \
  --obs $CLAS/0627239Q.obs \
  --nav $CLAS/sept_2019239.nav \
  --ssr /tmp/clas_expanded4.csv \
  --out /tmp/ppp_debug.pos \
  --static --estimate-troposphere --no-ionosphere-free --estimate-ionosphere \
  --max-epochs $MAX 2>&1 | grep -E "OSR-SSR|OSR\]|CLAS-OBS|CLAS-PPP"
