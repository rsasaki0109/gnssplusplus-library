"""Evaluation-only output comparison; never invoked by native inference."""
import csv
import json
import math
from pathlib import Path
from verify_native_imu_bias_diagnostic import digest

ROOT = Path(__file__).resolve().parents[1]


def read_output(path):
    rows = {}
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        assert reader.fieldnames == ['phone', 'UnixTimeMillis', 'LatitudeDegrees', 'LongitudeDegrees']
        for row in reader:
            key = row['phone'], int(row['UnixTimeMillis'])
            assert key not in rows, 'duplicate output identity'
            lat, lon = float(row['LatitudeDegrees']), float(row['LongitudeDegrees'])
            assert math.isfinite(lat) and math.isfinite(lon)
            assert -90 <= lat <= 90 and -180 <= lon <= 180
            rows[key] = lat, lon
    assert rows
    return rows


def separation(a, b):
    lat1, lon1, lat2, lon2 = map(math.radians, (*a, *b))
    h = math.sin((lat2-lat1)/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin((lon2-lon1)/2)**2
    return 2*6371008.8*math.asin(math.sqrt(max(0., min(1., h))))


def main():
    specs = [(472, 'd2c619121951d1a322ab39e15fcf16dbf7a6d61d2c651d03a4f68ea357c61e7d'),
             (476, '255318bdad6c396deb5a6a634a177022defefc1edd4655c039b4686891288df4')]
    outputs, summaries = [], []
    for phase, expected in specs:
        directory = ROOT / f'output/smartphone-r5/phase{phase}-lax-t-robust-diagnostic-v1/lax-t'
        path = directory / 'opaque_solution_output.csv'
        assert digest(path) == expected
        outputs.append(read_output(path))
        summaries.append(json.loads((directory / 'native_summary.json').read_text()))
    a, b = outputs
    assert a.keys() == b.keys() and len(a) == 1466
    distances = sorted(separation(a[key], b[key]) for key in a)
    def percentile(p):
        index = (len(distances)-1)*p
        lo, hi = math.floor(index), math.ceil(index)
        return distances[lo] + (distances[hi]-distances[lo])*(index-lo)
    fields = ('factors', 'values', 'imu_intervals', 'upstream_stop_pose_factors',
              'upstream_stop_velocity_factors', 'upstream_stop_epochs')
    graph = {}
    for key in fields:
        left, right = (s['graph'][key] for s in summaries)
        assert left == right, key
        graph[key] = right
    print(json.dumps(dict(rows=len(a), exact_identity_alignment=True,
        separation_m=dict(mean=sum(distances)/len(distances), p50=percentile(.5),
                          p95=percentile(.95), maximum=distances[-1]),
        unchanged_graph_counts=graph, truth_reads=0, accuracy_evaluations=0,
        inference_input=False, promoted=False), indent=2))


if __name__ == '__main__':
    main()
