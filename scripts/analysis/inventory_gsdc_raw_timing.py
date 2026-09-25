"""Read only raw GNSS clock fields from a pinned GSDC archive.

This describes raw UTC groups, not epochs accepted by the native adapter.
It uses no position fields and reads no truth, IMU or MAT payloads.
"""
import argparse
import csv
import hashlib
import io
import json
import zipfile
from pathlib import Path, PurePosixPath


def inventory(stream):
    previous = None
    seen = set()
    gaps = []
    reversals = []
    reappeared = []
    clocks = set()
    rows = groups = invalid = zero_time = 0
    for row in csv.DictReader(stream):
        rows += 1
        try:
            utc = int(row['utcTimeMillis'])
            nanos = int(row['TimeNanos'])
        except (KeyError, ValueError):
            invalid += 1
            continue
        if nanos == 0:
            zero_time += 1
            continue
        clocks.add(row.get('HardwareClockDiscontinuityCount', ''))
        if utc == previous:
            continue
        groups += 1
        if utc in seen:
            reappeared.append(utc)
        if previous is not None:
            delta = utc - previous
            if delta > 2000:
                gaps.append({'before_utc_ms': previous, 'after_utc_ms': utc,
                             'delta_ms': delta})
            if delta < 0:
                reversals.append({'before_utc_ms': previous, 'after_utc_ms': utc})
        seen.add(utc)
        previous = utc
    return {'raw_rows': rows, 'invalid_time_rows': invalid,
            'zero_TimeNanos_rows': zero_time, 'utc_groups': groups,
            'unique_utc_groups': len(seen), 'gaps_above_2s': gaps,
            'reverse_utc_transitions': reversals, 'reappeared_utc_groups': reappeared,
            'hardware_clock_values': sorted(clocks)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--expected-sha256', required=True)
    parser.add_argument('--split', choices=['train', 'test'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.archive.open('rb') as handle:
        digest = hashlib.file_digest(handle, 'sha256').hexdigest()
    if digest != args.expected_sha256:
        raise ValueError('Archive hash mismatch; no payload read')
    results = {}
    with zipfile.ZipFile(args.archive) as archive:
        for name in sorted(archive.namelist()):
            parts = PurePosixPath(name).parts
            if len(parts) != 5 or parts[:2] != ('dataset_2023', args.split) or parts[-1] != 'device_gnss.csv':
                continue
            key = '/'.join(parts[2:4])
            with archive.open(name) as handle:
                results[key] = inventory(io.TextIOWrapper(handle, encoding='utf-8-sig', newline=''))
            print(f"{key}: {results[key]['utc_groups']} raw UTC groups, "
                  f"{len(results[key]['gaps_above_2s'])} gaps above 2 s", flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'archive_sha256': digest, 'split': args.split,
        'basis': 'raw UTC groups with nonzero TimeNanos; not native accepted GPS epochs',
        'runs': results}, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
