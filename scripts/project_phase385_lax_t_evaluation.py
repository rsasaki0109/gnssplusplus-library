"""Evaluation-only first-epoch projection; never an inference input."""
import csv
import hashlib
import io
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'docs/use_cases/records/smartphone_r5_phase385_projection_manifest_v1.json'


def main():
    m = json.loads(MANIFEST.read_text())
    def read(pin):
        b = (ROOT / pin['path']).read_bytes()
        assert hashlib.sha256(b).hexdigest() == pin['sha256']
        if 'bytes' in pin:
            assert len(b) == pin['bytes']
        return b
    raw = read(m['raw'])
    keys = sorted({int(r['utcTimeMillis']) for r in
                   csv.DictReader(io.StringIO(raw.decode())) if r['MessageType'] == 'Raw'})
    assert len(keys) == 1466
    prepared = []
    for pin in m['candidates']:
        b = read(pin)
        lines = b.splitlines(keepends=True)
        rows = list(csv.DictReader(io.StringIO(b.decode())))
        assert len(lines) == 1467 and len(rows) == 1466
        assert [int(r['UnixTimeMillis']) for r in rows] == keys
        assert all(r['phone'] == m['route'] for r in rows)
        # Preserve all retained CSV bytes; no coordinate parsing or rewriting.
        projected = lines[0] + b''.join(lines[2:])
        prepared.append((pin, projected))
    target = ROOT / m['output_directory']
    target.mkdir(parents=True, exist_ok=False)
    results = []
    for pin, b in prepared:
        path = target / pin['output_name']
        with path.open('xb') as f:
            f.write(b)
        results.append({'path': str(path.relative_to(ROOT)),
                        'sha256': hashlib.sha256(b).hexdigest(), 'bytes': len(b), 'rows': 1465,
                        'original': pin, 'removed_rows': 1})
    result = {'phase': 385, 'truth_reads': 0, 'native_invocations': 0,
              'role': 'evaluation/output only; forbidden as inference input',
              'manifest_sha256': hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
              'projected': results}
    with (target / 'projection_record.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
