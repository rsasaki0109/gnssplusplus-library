#!/usr/bin/env python3
"""SSR CSVの内容を衛星ごとにサマリー表示。
orbit/clock/bias/atmosの有無をTOW別に確認。

Usage: python3 scripts/debug_ssr_csv.py [csv_path] [satellite]
  csv_path: default=/tmp/clas_expanded4.csv
  satellite: e.g. G14, E07 (default=all, summary only)
"""
import sys, csv
from collections import defaultdict

csv_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/clas_expanded4.csv"
target_sat = sys.argv[2] if len(sys.argv) > 2 else None

stats = defaultdict(lambda: {"orbit_nonzero": 0, "clock_nonzero": 0,
                              "cbias": 0, "pbias": 0, "atmos": 0, "total": 0})

with open(csv_path) as f:
    reader = csv.reader(f)
    for row in reader:
        if row[0].startswith("#"):
            continue
        if len(row) < 7:
            continue
        tow = float(row[1])
        sat = row[2]
        dx, dy, dz = float(row[3]), float(row[4]), float(row[5])
        clk = float(row[6]) if row[6] != "nan" else 0.0

        s = stats[sat]
        s["total"] += 1
        if abs(dx) > 0 or abs(dy) > 0 or abs(dz) > 0:
            s["orbit_nonzero"] += 1
        if abs(clk) > 0:
            s["clock_nonzero"] += 1
        extras = row[7:]
        for e in extras:
            if e.startswith("cbias:"):
                s["cbias"] += 1
            elif e.startswith("pbias:"):
                s["pbias"] += 1
            elif e.startswith("atmos_"):
                s["atmos"] += 1

        if target_sat and sat == target_sat and s["total"] <= 10:
            orbit_str = f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f}" if abs(dx)+abs(dy)+abs(dz) > 0 else "orbit=0"
            clk_str = f"clk={clk:.4f}" if abs(clk) > 0 else "clk=0"
            extra_types = set()
            for e in extras:
                if "=" in e:
                    extra_types.add(e.split("=")[0].split(":")[0])
            print(f"  tow={tow:.0f} {orbit_str} {clk_str} extras={sorted(extra_types)}")

# Summary
print(f"\n{'Sat':<6} {'Total':>6} {'Orbit≠0':>8} {'Clk≠0':>7} {'CBias':>6} {'PBias':>6} {'Atmos':>6}")
print("-" * 50)
for sat in sorted(stats.keys()):
    s = stats[sat]
    print(f"{sat:<6} {s['total']:>6} {s['orbit_nonzero']:>8} {s['clock_nonzero']:>7} {s['cbias']:>6} {s['pbias']:>6} {s['atmos']:>6}")
