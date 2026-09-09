"""Raw quality metadata only; never access derived satellite/WLS columns."""
import csv
from collections import Counter
import json
import math
from run_phase486_nhc_monitor import ROOT, digest

def main():
    manifest = json.loads((ROOT / 'output/smartphone-r5/phase493-h-code-monitor-v1/mtv-h/manifest.json').read_text())
    pin = manifest['inputs']['android_gnss']
    path = ROOT / pin['path']
    assert digest(path) == pin['sha256']
    fields = ('MultipathIndicator', 'AgcDb', 'BasebandCn0DbHz',
              'FullInterSignalBiasNanos', 'FullInterSignalBiasUncertaintyNanos',
              'SatelliteInterSignalBiasNanos', 'SatelliteInterSignalBiasUncertaintyNanos')
    values = {k: [] for k in fields}
    missing = Counter()
    multipath = Counter()
    rows = 0
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        assert all(k in reader.fieldnames for k in fields)
        for row in reader:
            rows += 1
            for key in fields:
                try:
                    value = float(row[key])
                except (TypeError, ValueError):
                    missing[key] += 1
                    continue
                if not math.isfinite(value):
                    missing[key] += 1
                    continue
                values[key].append(value)
                if key == 'MultipathIndicator': multipath[value] += 1
    summary = {}
    for key, samples in values.items():
        summary[key] = dict(finite=len(samples), missing=missing[key],
                            nonzero=sum(x != 0 for x in samples),
                            minimum=min(samples) if samples else None,
                            maximum=max(samples) if samples else None)
    result = dict(phase=497, raw_rows=rows, raw_sha256=pin['sha256'],
                  fields=summary, multipath_counts=dict(multipath),
                  selected_columns=list(fields), truth_reads=0,
                  derived_position_columns_used=False, retained_factor_claim=False)
    target = ROOT / 'docs/use_cases/records/smartphone_r5_phase497_raw_quality_result_v1.json'
    with target.open('x') as handle: json.dump(result, handle, indent=2)
    print(json.dumps(result, indent=2))

if __name__ == '__main__':
    main()
