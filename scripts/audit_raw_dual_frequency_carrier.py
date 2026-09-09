"""Raw-only L1/L5 carrier-difference increments; aggregate diagnostic only.

Not a native factor-admission replica or an ionosphere estimate. Uses no
enriched satellite/position/pseudorange fields and emits no epoch series.
"""
import argparse
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


def increment(previous, current):
    return (current[0] - current[1]) - (previous[0] - previous[1])


def percentile(values, fraction):
    v = sorted(values)
    if not v:
        return None
    index = fraction * (len(v) - 1)
    low = int(index)
    return v[low] + (index - low) * (v[min(low + 1, len(v)-1)] - v[low])


def audit(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    epochs = defaultdict(dict)
    counts = Counter()
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            if row['MessageType'] != 'Raw' or row['ConstellationType'] not in ('1', '6'):
                continue
            counts['gps_gal_raw_rows'] += 1
            frequency = float(row['CarrierFrequencyHz'])
            band = 'L1' if abs(frequency - 1575420000) < 1e6 else (
                'L5' if abs(frequency - 1176450000) < 1e6 else None)
            if band is None:
                counts['unsupported_frequency'] += 1
                continue
            state = int(row['AccumulatedDeltaRangeState'])
            adr = float(row['AccumulatedDeltaRangeMeters'])
            valid = bool(state & 1) and not bool(state & 6) and math.isfinite(adr)
            key = (int(row['ConstellationType']), int(row['Svid']))
            epoch = int(row['utcTimeMillis'])
            identity = (*key, band)
            if identity in epochs[epoch]:
                raise ValueError('Duplicate raw epoch/satellite/band')
            epochs[epoch][identity] = (
                adr, valid, int(row['TimeNanos']), float(row['TimeOffsetNanos'] or 0),
                int(row['HardwareClockDiscontinuityCount']))
    previous = {}
    values = defaultdict(list)
    for epoch, rows in sorted(epochs.items()):
        current = {}
        for system, sv in sorted({(k[0], k[1]) for k in rows}):
            a, b = rows.get((system, sv, 'L1')), rows.get((system, sv, 'L5'))
            if a is None or b is None:
                counts['missing_dual_band'] += 1
                continue
            if not a[1] or not b[1]:
                counts['invalid_adr_pair'] += 1
                continue
            if a[2:] != b[2:]:
                counts['different_clock_or_time_offset'] += 1
                continue
            current[(system, sv)] = (epoch, a, b)
            old = previous.get((system, sv))
            if old is None:
                continue
            utc_dt = (epoch-old[0])/1000
            # UTC milliseconds identify rows, but the measurement interval is
            # Android's raw receiver clock plus each observation's offset.
            # Subtract integer TimeNanos before converting to floating point.
            dt = ((a[2]-old[1][2]) + (a[3]-old[1][3])) * 1e-9
            if (not 0 < utc_dt <= 1.5 or not math.isfinite(dt) or
                    not 0 < dt <= 1.5 or a[4] != old[1][4]):
                counts['gap_or_clock_reset'] += 1
                continue
            values[system].append(increment((old[1][0], old[2][0]), (a[0], b[0])) / dt)
        # Reset across missing/invalid adjacent epochs, never bridge a bad pair.
        previous = current
    groups = {}
    for system, v in values.items():
        absolute = [abs(x) for x in v]
        groups[str(system)] = {
            'count': len(v), 'signed_median_mps': percentile(v, .5),
            'abs_p50_mps': percentile(absolute, .5),
            'abs_p95_mps': percentile(absolute, .95), 'abs_max_mps': max(absolute),
            'abs_over_0_1_mps': sum(x > .1 for x in absolute),
            'abs_over_1_mps': sum(x > 1 for x in absolute)}
    return {'raw_sha256': digest.hexdigest(), 'counts': dict(counts), 'groups': groups}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw_gnss', type=Path)
    print(json.dumps(audit(parser.parse_args().raw_gnss), indent=2, sort_keys=True))
