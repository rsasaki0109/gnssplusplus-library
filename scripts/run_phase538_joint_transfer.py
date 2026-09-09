"""Frozen raw U/LAX recipes, current diagnostic binary; no truth or coordinate reads."""
import json
import os
import subprocess
import sys
import time
from run_phase486_nhc_monitor import ROOT, digest
from verify_phase522_ionosphere_monitor import parse_monitor, parse_residual
from verify_phase479_lambda_floor import check_stage


def main(route):
    old_name = {'u': '480_u', 'lax': '476_lax_t'}[route]
    old_path = ROOT / f'docs/use_cases/records/smartphone_r5_phase{old_name}_comparison_manifest_v1.json'
    old = json.loads(old_path.read_text())
    current_path = ROOT / 'output/smartphone-r5/phase536-h-joint-ionosphere-v1/mtv-h/manifest.json'
    current = json.loads(current_path.read_text())
    for name, sha in current['source_pins'].items():
        assert digest(ROOT / name) == sha, name
    assert digest(ROOT / current['argv'][0]) == current['binary_sha256']
    for pin in old['inputs'].values():
        path = ROOT / pin['path']
        assert digest(path) == pin['sha256'] and path.stat().st_size == pin['bytes']
    argv = list(old['argv'])
    assert '--native-joint-ionosphere' not in argv
    argv += ['--native-joint-ionosphere', '3', '0.02', '1.5']
    baseline_output = ROOT / argv[argv.index('--out')+1]
    baseline_summary = ROOT / argv[argv.index('--summary-json')+1]
    directory = ROOT / f'output/smartphone-r5/phase538-joint-transfer-v1/{route}'
    expected_hash = digest(baseline_output)  # opaque byte comparison only
    for flag, filename in (('--out','opaque_solution_output.csv'),('--summary-json','native_summary.json')):
        argv[argv.index(flag)+1] = str((directory / filename).relative_to(ROOT))
    directory.mkdir(parents=True, exist_ok=False)
    record = dict(route=route, argv=argv, inputs=old['inputs'], source_pins=current['source_pins'],
                  binary_sha256=current['binary_sha256'], old_manifest_sha256=digest(old_path),
                  current_manifest_sha256=digest(current_path), expected_output_sha256=expected_hash,
                  baseline_summary_sha256=digest(baseline_summary),
                  launcher_sha256=digest(ROOT / 'scripts/run_phase538_joint_transfer.py'))
    with (directory/'manifest.json').open('x') as f: json.dump(record,f,indent=2)
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = '/home/sasaki/.local/lib:' + env.get('LD_LIBRARY_PATH','')
    start = time.monotonic()
    with (directory/'stdout.log').open('x') as out, (directory/'stderr.log').open('x') as err:
        process = subprocess.Popen(argv,cwd=ROOT,env=env,stdout=out,stderr=err)
        with (directory/'started.json').open('x') as f: json.dump({'pid':process.pid},f)
        print(json.dumps(dict(route=route,pid=process.pid)),flush=True)
        code = process.wait()
    result = dict(return_code=code,elapsed_seconds=time.monotonic()-start)
    if code == 0:
        result['output_identical'] = digest(directory/'opaque_solution_output.csv') == expected_hash
        log = (directory/'stderr.log').read_text()
        result['irls'] = parse_monitor(log)
        result['residual'] = parse_residual(log,result['irls']['epochs'])
        s = json.loads((directory/'native_summary.json').read_text())
        b = json.loads(baseline_summary.read_text())
        n = s['epochs']['problem']
        result['graph_increments_valid'] = all(
            s['graph'][k] == b['graph'][k] + n for k in ('factors', 'values'))
        result['epochs_identical'] = s['epochs'] == b['epochs']
        result['converged'] = s['graph']['converged']
        assert s['native_joint_ionosphere'] == dict(enabled=True, main_only=True,
            anchor_sigma_m=3.0, density_m_sqrt_s=0.02, max_gap_s=1.5)
        import re
        rows = re.findall(r'^\[native-joint-ionosphere\] states=(\d+) priors=(\d+) code=(\d+) tdcp=(\d+) anchor_sigma=3 density=0.02 max_gap=1.5$', log, re.M)
        assert len(rows) == 1
        assert tuple(map(int, rows[0])) == (n, n, s['epochs']['pseudorange_factors'], s['epochs']['tdcp_factors_built'])
        result['joint_solved'] = re.findall(r'^\[native-joint-ionosphere-solved\].*$', log, re.M)
        assert len(result['joint_solved']) == 1
        result['output_sha256'] = digest(directory/'opaque_solution_output.csv')
        result['gnss_first_identical'] = s['gnss_first'] == b['gnss_first']
        assert result['irls']['rows'] == s['epochs']['pseudorange_factors']
        assert result['irls']['epochs'] == s['epochs']['problem']
        for stage in ('main','gnss_first'): check_stage(s['phase143_termination'][stage],stage)
    with (directory/'completed.json').open('x') as f: json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
    assert code == 0
    assert result['graph_increments_valid'] and result['gnss_first_identical']
    assert result['epochs_identical'] and result['converged']


if __name__ == '__main__':
    main(sys.argv[1])
