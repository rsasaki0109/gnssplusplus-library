"""Audit post-solve IMU export against a completed frozen reference; no truth input."""
import argparse
import json
from pathlib import Path

import numpy as np

from compare_gsdc_doppler_diagnostic import audited_run, normalized_argv
from run_gsdc_doppler_segment_diagnostic import sha


def inspect_states(summary, keys):
    export = summary['optimized_imu_states']
    assert export['schema_version'] == 1
    assert export['truth_used'] is False and export['estimator_feedback'] is False
    assert export['bias_axes'] == 'body FLU'
    assert export['attitude_convention'] == 'body FLU to nav ENU Rot3::rpy radians'
    rows = export['epochs']
    assert [r['index'] for r in rows] == list(range(len(keys)))
    assert [r['raw_utc_time_millis'] for r in rows] == sorted(keys)
    vectors = {}
    for field in ['rpy_rad', 'velocity_enu_mps', 'accel_bias_mps2', 'gyro_bias_radps']:
        values = np.asarray([r[field] for r in rows], dtype=float)
        assert values.shape == (len(keys), 3) and np.isfinite(values).all()
        vectors[field] = values
    result = {}
    for field, aggregate in [('accel_bias_mps2', 'optimized_accel_bias_max_norm_mps2'),
                             ('gyro_bias_radps', 'optimized_gyro_bias_max_norm_radps')]:
        norms = np.linalg.norm(vectors[field], axis=1)
        assert np.isclose(norms.max(), summary['tdcp_contract'][aggregate], rtol=1e-12, atol=1e-12)
        maximum = int(np.argmax(norms))
        result[field] = dict(first_norm=float(norms[0]), last_norm=float(norms[-1]),
            maximum_norm=float(norms[maximum]), maximum_epoch=maximum,
            maximum_utc=rows[maximum]['raw_utc_time_millis'])
    norms = np.linalg.norm(vectors['accel_bias_mps2'], axis=1)
    result['accel_bias_thresholds_mps2'] = {}
    for threshold in [0.1, 1.0, 5.0, 10.0]:
        selected = np.flatnonzero(norms > threshold)
        result['accel_bias_thresholds_mps2'][str(threshold)] = dict(
            count=len(selected), first_epoch=int(selected[0]) if len(selected) else None,
            last_epoch=int(selected[-1]) if len(selected) else None)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--diagnostic', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    a, b = audited_run(args.reference), audited_run(args.diagnostic)
    aa, bb = normalized_argv(a[0]['argv']), normalized_argv(b[0]['argv'])
    # The diagnostic binary is a separate build; all inference options must match.
    assert bb[1:] == aa[1:] + ['--native-imu-state-diagnostic']
    assert a[0]['inputs_sha256'] == b[0]['inputs_sha256']
    assert b[0]['source_run_sha256'] == sha(args.reference/'run.json')
    assert a[1]['dataset_id'] == b[1]['dataset_id']
    assert (args.reference/'solution.csv').read_bytes() == (args.diagnostic/'solution.csv').read_bytes(), 'diagnostic changed frozen trajectory'
    report = dict(status='audited-imu-state-export', truth_consumed=False,
        accuracy_evaluated=False, trajectory_byte_parity=True,
        reference_binary_sha256=a[0]['binary_sha256'], diagnostic_binary_sha256=b[0]['binary_sha256'],
        reference_run_sha256=sha(args.reference/'run.json'),
        summary_sha256=b[0]['summary_sha256'], states=inspect_states(b[1], b[2]))
    args.out.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
