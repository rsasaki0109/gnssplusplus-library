"""One raw-only seed audit; metadata output only, no FGO or truth."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    prior = json.loads((ROOT / 'docs/use_cases/records/smartphone_r5_phase455_lax_t_comparison_manifest_v1.json').read_text())
    for group in ('source_pins', 'algorithm_source_pins'):
        for path, expected in prior[group].items():
            assert digest(ROOT / path) == expected, path
    for item in (prior['binary'], prior['inputs']['android_gnss'], prior['inputs']['nav']):
        assert digest(ROOT / item['path']) == item['sha256']
    directory = ROOT / 'output/smartphone-r5/phase459-lax-seed-audit-v1'
    directory.mkdir(exist_ok=False)
    summary = directory / 'seed_summary.json'
    argv = [prior['binary']['path'], '--dataset-id', prior['dataset_id'],
            '--android-gnss', prior['inputs']['android_gnss']['path'],
            '--nav', prior['inputs']['nav']['path'], '--all-epochs',
            '--android-raw-utc-keys', '--android-raw-clock-only',
            '--android-utc-wall-clock-fallback', '--android-include-first-native-epoch',
            '--native-phase149-raw-p-seed-stage', '--native-phase157-raw-p-bootstrap',
            '--summary-json', str(summary.relative_to(ROOT))]
    manifest = dict(phase=459, argv=argv, binary=prior['binary'],
                    inputs={k: prior['inputs'][k] for k in ('android_gnss', 'nav')},
                    source_pins={**prior['source_pins'], **prior['algorithm_source_pins']},
                    truth_reads=0, fgo_invocations=0)
    for path in ('src/algorithms/raw_p_seed.cpp', 'src/algorithms/spp.cpp',
                 'include/libgnss++/algorithms/raw_p_seed.hpp',
                 'include/libgnss++/algorithms/spp.hpp', 'scripts/run_phase459_seed_audit.py'):
        manifest['source_pins'][path] = digest(ROOT / path)
    with (directory / 'manifest.json').open('x') as stream:
        json.dump(manifest, stream, indent=2)
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH', '')
    start = time.monotonic()
    with (directory / 'stdout.log').open('x') as out, (directory / 'stderr.log').open('x') as err:
        process = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=out, stderr=err)
        print(json.dumps(dict(pid=process.pid, directory=str(directory))), flush=True)
        code = process.wait()
    with (directory / 'completed.json').open('x') as stream:
        json.dump(dict(return_code=code, elapsed_seconds=time.monotonic()-start,
                       manifest_sha256=digest(directory / 'manifest.json')), stream)
    assert code == 0, f'native exit {code}; do not retry automatically'
    report = json.loads(summary.read_text())
    assert not report['truth_used'] and not report['mat_used'] and not report['fgo_entered']
    keys = ('input_epoch_index', 'raw_source_index', 'status',
            'native_used_pseudorange_rows', 'degrees_of_freedom',
            'gdop', 'residual_rms_m', 'max_abs_residual_m')
    rows = report['epochs']
    print(json.dumps(dict(epochs=len(rows), selected=[{k: row[k] for k in keys}
        for row in rows if 850 <= row['input_epoch_index'] <= 853]), indent=2))


if __name__ == '__main__':
    main()
