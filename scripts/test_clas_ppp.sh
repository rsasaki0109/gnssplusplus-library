#!/bin/bash
set -e
GNSSPP=/media/sasaki/aiueo/ai_coding_ws/rtklib_v2_ws/gnssplusplus-library
THESIS=/media/sasaki/aiueo/ai_coding_ws/gnssplusplus_thesis_ws
CLAS=$THESIS/data/clas/claslib/data
MAX_EPOCHS=${1:-3600}

cd $GNSSPP

# 1. ビルド
echo "=== Build ==="
rm -f build_local/CMakeFiles/gnss_lib.dir/src/algorithms/ppp*.o \
      build_local/CMakeFiles/gnss_lib.dir/src/core/navigation.cpp.o \
      build_local/libgnss_lib.a build_local/apps/gnss_ppp 2>/dev/null
cmake --build build_local --target gnss_ppp -j$(nproc) 2>&1 | tail -3
if [ ! -f build_local/apps/gnss_ppp ]; then
    echo "BUILD FAILED"
    exit 1
fi

# 2. CSVが存在しなければ再生成
if [ ! -f /tmp/clas_expanded4.csv ]; then
    echo "=== Regenerating CLAS CSV ==="
    python3 -c "
import sys; sys.path.insert(0, 'apps')
from gnss_qzss_l6_info import decode_source, decode_cssr_messages, write_compact_corrections
from pathlib import Path
from apps.gnss_clas_ppp import expand_compact_ssr_text
frames, subframes, _ = decode_source('$CLAS/2019239Q.l6')
messages, corrections, _ = decode_cssr_messages(subframes, gps_week=2068)
compact = Path('/tmp/clas_compact.csv')
expanded = Path('/tmp/clas_expanded4.csv')
write_compact_corrections(compact, corrections)
expand_compact_ssr_text(compact.read_text(encoding='ascii'), expanded)
print(f'CSV: {expanded} ({expanded.stat().st_size} bytes)')
"
fi

# 3. CLAS-PPP実行
echo "=== Run CLAS-PPP (max_epochs=$MAX_EPOCHS) ==="
./build_local/apps/gnss_ppp \
  --obs $CLAS/0627239Q.obs \
  --nav $CLAS/sept_2019239.nav \
  --ssr /tmp/clas_expanded4.csv \
  --out /tmp/ppp_test.pos \
  --static --estimate-troposphere --no-ionosphere-free --estimate-ionosphere \
  --enable-ar --ar-ratio-threshold 2.0 \
  --max-epochs $MAX_EPOCHS 2>&1 | grep -E "float|fixed|converge|fallback|solution rate" -i | tail -5

# 4. 精度計算
echo "=== Accuracy ==="
python3 -c "
import math, numpy as np
ref_x, ref_y, ref_z = -3957240.1233, 3310370.8778, 3737527.7041
xs, ys, zs, sts = [], [], [], []
with open('/tmp/ppp_test.pos') as f:
    for line in f:
        if line.startswith('%'): continue
        parts = line.split()
        if len(parts) >= 9:
            xs.append(float(parts[2])); ys.append(float(parts[3])); zs.append(float(parts[4]))
            sts.append(int(parts[8]))
errors_3d = [math.sqrt((xs[i]-ref_x)**2+(ys[i]-ref_y)**2+(zs[i]-ref_z)**2) for i in range(len(xs))]
fixed = [errors_3d[i] for i in range(len(sts)) if sts[i]==6]
flt = [errors_3d[i] for i in range(len(sts)) if sts[i]==5]
print(f'Epochs: {len(xs)}, Fixed: {len(fixed)}, Float: {len(flt)}, Fallback: {sum(1 for s in sts if s<5)}')
if errors_3d:
    print(f'All 3D: median={np.median(errors_3d):.4f}m, last50={np.median(errors_3d[-50:]):.4f}m, min={np.min(errors_3d):.4f}m')
if fixed:
    print(f'Fixed 3D: median={np.median(fixed):.4f}m, min={np.min(fixed):.4f}m')
print('--- Target: CLASLIB Float=4.9m, Fix=0.06m ---')
"
