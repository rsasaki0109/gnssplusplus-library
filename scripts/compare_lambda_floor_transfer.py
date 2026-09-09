"""Evaluation-only H/U comparison, no trajectory enters inference."""
import json
import sys
from compare_lambda_floor_outputs import ROOT, read_output, separation
from verify_native_imu_bias_diagnostic import digest


def main():
    phase = int(sys.argv[1])
    tag, baseline, route, expected_rows = {
        479: ('h_baseline_replay', 'phase438-h-robust-diagnostic-v1', 'mtv-h', 3139),
        480: ('u_comparison', 'phase249-u-native-first-epoch-v1', 'mtv-u', 1102),
    }[phase]
    manifest = json.loads((ROOT / f'docs/use_cases/records/smartphone_r5_phase{phase}_{tag}_manifest_v1.json').read_text())
    candidate_dir = (ROOT / manifest['launcher_contract']['execution_metadata_path']).parent
    completed = json.loads((candidate_dir / 'launcher_completed.json').read_text())
    assert completed['return_code'] == 0
    base_dir = ROOT / 'output/smartphone-r5' / baseline / route
    base = base_dir / 'opaque_solution_output.csv'
    candidate = candidate_dir / 'opaque_solution_output.csv'
    assert digest(base) == manifest['expected_candidate_sha256']
    a, b = read_output(base), read_output(candidate)
    assert a.keys() == b.keys() and len(a) == expected_rows
    delta = sorted(separation(a[k], b[k]) for k in a)
    summaries = [json.loads((d / 'native_summary.json').read_text()) for d in (base_dir, candidate_dir)]
    counts = {}
    for key in ('factors', 'values', 'imu_intervals', 'upstream_stop_pose_factors',
                'upstream_stop_velocity_factors', 'upstream_stop_epochs'):
        counts[key] = [s['graph'][key] for s in summaries]
        assert counts[key][0] == counts[key][1], key
    def percentile(p):
        index = (len(delta)-1)*p
        lo = int(index)
        return delta[lo] + (delta[min(lo+1,len(delta)-1)]-delta[lo])*(index-lo)
    print(json.dumps(dict(phase=phase, rows=len(a), candidate_sha256=digest(candidate),
        separation_m=dict(mean=sum(delta)/len(delta), p50=percentile(.5),
                          p95=percentile(.95), maximum=delta[-1]),
        unchanged_graph_counts=counts, truth_reads=0, accuracy_evaluations=0,
        inference_input=False, promoted=False), indent=2))


if __name__ == '__main__':
    main()
