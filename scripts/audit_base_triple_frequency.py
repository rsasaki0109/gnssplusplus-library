"""Read-only GPS triple-frequency raw RINEX coverage, no position estimation."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path


def audit(path):
    digest = hashlib.sha256()
    codes = []
    epochs = 0
    gps_rows = 0
    counts = collections.Counter()
    satellites = collections.defaultdict(set)
    in_header = True
    with path.open('rb') as stream:
        for payload in stream:
            digest.update(payload)
            line = payload.decode('ascii').rstrip('\r\n')
            if in_header:
                if 'SYS / # / OBS TYPES' in line and line.startswith('G'):
                    fields = line.split('SYS / # / OBS TYPES')[0].split()
                    expected = int(fields[1])
                    codes = fields[2:]
                    if len(codes) != expected:
                        raise ValueError('multiline GPS header requires native parser')
                if 'END OF HEADER' in line:
                    in_header = False
                continue
            if line.startswith('>'):
                epochs += 1
                continue
            if not line.startswith('G'):
                continue
            if not codes or len(line) < 3 + 16*len(codes)-2:
                raise ValueError('short/continued GPS row requires native parser')
            gps_rows += 1
            values = {}
            for i, code in enumerate(codes):
                text = line[3+16*i:3+16*i+14].strip()
                if text:
                    value = float(text)
                    if math.isfinite(value) and value != 0:
                        values[code] = value
            for middle in ('2W', '2X'):
                required = [kind+band for band in ('1C', middle, '5X') for kind in ('C', 'L')]
                if all(code in values for code in required):
                    counts[middle] += 1
                    satellites[middle].add(line[:3])
    if in_header or not epochs:
        raise ValueError('incomplete RINEX')
    return dict(path=str(path), sha256=digest.hexdigest(), epochs=epochs, gps_rows=gps_rows,
                simultaneous_code_carrier_rows=dict(counts),
                satellite_counts={k: len(v) for k, v in satellites.items()},
                note='coverage only; no slip screening, model, solver, truth, or positioning inputs')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw_base', type=Path)
    print(json.dumps(audit(parser.parse_args().raw_base), indent=2))
