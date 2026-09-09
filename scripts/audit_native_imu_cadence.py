#!/usr/bin/env python3
"""Read raw IMU timestamp columns only; emit aggregate interpolation support.

Diagnostic, not an inference input or a substitute for native loader checks.
Matches native UTC-key deduplication for explicit wall-clock fallback only.
"""
import argparse
import bisect
import csv
import hashlib
import json
from pathlib import Path


def distribution(values):
    values = sorted(values)
    if not values:
        return {"count": 0}
    return {"count": len(values), "min_ms": values[0],
            "median_ms": values[(len(values) - 1) // 2],
            "p95_ms": values[(95 * (len(values) - 1)) // 100],
            "max_ms": values[-1]}


def support(accel, gyro):
    accel, gyro = sorted(set(accel)), sorted(set(gyro))
    spans, nearest = [], []
    exact = outside = 0
    for t in gyro:
        i = bisect.bisect_left(accel, t)
        if i < len(accel) and accel[i] == t:
            exact += 1
        elif 0 < i < len(accel):
            spans.append(accel[i] - accel[i - 1])
            nearest.append(min(t - accel[i - 1], accel[i] - t))
        else:
            outside += 1
    return {"accel_unique": len(accel), "gyro_unique": len(gyro),
            "exact": exact, "outside_accel_range": outside,
            "accel_cadence": distribution([b-a for a,b in zip(accel, accel[1:])]),
            "gyro_cadence": distribution([b-a for a,b in zip(gyro, gyro[1:])]),
            "interpolation_span": distribution(spans),
            "interpolation_nearest_offset": distribution(nearest),
            "interpolation_spans_over_100ms": sum(s > 100 for s in spans)}


def audit(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    clocks = {"UncalAccel": [], "UncalGyro": []}
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            kind = row['MessageType'].strip()
            if kind not in clocks:
                continue
            if row['elapsedRealtimeNanos'].strip():
                raise ValueError('This audit only supports explicit UTC fallback')
            clocks[kind].append(int(row['utcTimeMillis']))
    result = support(clocks['UncalAccel'], clocks['UncalGyro'])
    result['sha256'] = digest.hexdigest()
    result['duplicate_utc_rows'] = {
        k: len(v) - len(set(v)) for k, v in clocks.items()}
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw_imu', type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.raw_imu), sort_keys=True, indent=2))
