#!/bin/bash
# libgnss++ vs CLASLIB Float精度比較
# Usage: ./scripts/compare_with_claslib.sh [max_epochs]
set -e
GNSSPP=/workspace/ai_coding_ws/rtklib_v2_ws/gnssplusplus-library
THESIS=/workspace/ai_coding_ws/gnssplusplus_thesis_ws
CLAS=$THESIS/data/clas/claslib/data
CLASLIB=$THESIS/data/clas/claslib/util/rnx2rtkp
MAX=${1:-3600}
REF_X=-3957240.1233; REF_Y=3310370.8778; REF_Z=3737527.7041

cd $GNSSPP

# 1. libgnss++ CLAS-PPP
echo "=== libgnss++ CLAS-PPP ==="
./build_local/apps/gnss_ppp \
  --obs $CLAS/0627239Q.obs --nav $CLAS/sept_2019239.nav \
  --ssr /tmp/clas_expanded4.csv --out /tmp/ppp_libgnss.pos \
  --static --estimate-troposphere --no-ionosphere-free --estimate-ionosphere \
  --max-epochs $MAX 2>&1 | grep -E "float|fixed|converge" -i | tail -3

# 2. CLASLIB Float
echo "=== CLASLIB Float ==="
sed 's/pos2-armode.*=.*/pos2-armode        =off/' /tmp/claslib_linux.conf > /tmp/claslib_float.conf 2>/dev/null || true
cd $CLASLIB
./rnx2rtkp -ti 1 -ts 2019/08/27 16:00:00 -te 2019/08/27 16:59:59 \
  -x 2 -k /tmp/claslib_float.conf \
  ../../data/0627239Q.obs ../../data/sept_2019239.nav ../../data/2019239Q.l6 \
  -o /tmp/claslib_float.nmea 2>&1 | tail -1

# 3. CLASLIB Fix
echo "=== CLASLIB Fix ==="
cd $CLASLIB
./rnx2rtkp -ti 1 -ts 2019/08/27 16:00:00 -te 2019/08/27 16:59:59 \
  -x 2 -k /tmp/claslib_linux.conf \
  ../../data/0627239Q.obs ../../data/sept_2019239.nav ../../data/2019239Q.l6 \
  -o /tmp/claslib_fix.nmea 2>&1 | tail -1

# 4. 精度比較
echo "=== Accuracy Comparison ==="
python3 -c "
import math, numpy as np

ref_x, ref_y, ref_z = $REF_X, $REF_Y, $REF_Z
a = 6378137.0; f = 1/298.257223563; e2 = 2*f - f*f
p = math.sqrt(ref_x**2 + ref_y**2)
lon = math.atan2(ref_y, ref_x)
lat = math.atan2(ref_z, p*(1-e2))
for _ in range(10):
    N = a / math.sqrt(1 - e2*math.sin(lat)**2)
    lat = math.atan2(ref_z + e2*N*math.sin(lat), p)
ref_lat, ref_lon = math.degrees(lat), math.degrees(lon)

def read_pos(path):
    xs, ys, zs = [], [], []
    with open(path) as f:
        for line in f:
            if line.startswith('%'): continue
            parts = line.split()
            if len(parts) >= 6:
                xs.append(float(parts[2])); ys.append(float(parts[3])); zs.append(float(parts[4]))
    return [math.sqrt((xs[i]-ref_x)**2+(ys[i]-ref_y)**2+(zs[i]-ref_z)**2) for i in range(len(xs))]

def read_nmea(path):
    lats, lons = [], []
    with open(path) as f:
        for line in f:
            if '\$GPGGA' not in line: continue
            parts = line.split(',')
            if len(parts) < 10: continue
            try:
                r = float(parts[2])
                la = int(r/100) + (r - int(r/100)*100)/60.0
                r = float(parts[4])
                lo = int(r/100) + (r - int(r/100)*100)/60.0
                lats.append(la); lons.append(lo)
            except: pass
    return [math.sqrt(((lats[i]-ref_lat)*111000)**2+((lons[i]-ref_lon)*111000*math.cos(math.radians(ref_lat)))**2) for i in range(len(lats))]

results = {}
try: e = read_pos('/tmp/ppp_libgnss.pos'); results['libgnss++ CLAS'] = e
except: pass
try: e = read_nmea('/tmp/claslib_float.nmea'); results['CLASLIB Float'] = e
except: pass
try: e = read_nmea('/tmp/claslib_fix.nmea'); results['CLASLIB Fix'] = e
except: pass

print(f\"{'Method':<20} {'Epochs':>7} {'Median':>8} {'Last50':>8} {'Min':>8}\")
print('-' * 55)
for name, errors in results.items():
    if errors:
        print(f\"{name:<20} {len(errors):>7} {np.median(errors):>8.3f}m {np.median(errors[-50:]):>8.3f}m {np.min(errors):>8.3f}m\")
"
