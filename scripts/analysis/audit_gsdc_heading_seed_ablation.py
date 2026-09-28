"""Audit a same-binary attitude-seed ablation without reading evaluation truth."""
import argparse
import json
from pathlib import Path

from audit_gsdc_imu_state_diagnostic import inspect_states
from assemble_gsdc_frozen_test_pairs import displacement
from compare_gsdc_doppler_diagnostic import audited_run, normalized_argv
from run_gsdc_doppler_segment_diagnostic import sha


def audit(reference, candidate):
    a, b = audited_run(reference), audited_run(candidate)
    ar, sa, pa = a
    br, sb, pb = b
    assert ar['binary_sha256'] == br['binary_sha256']
    assert ar['inputs_sha256'] == br['inputs_sha256']
    assert br['source_run_sha256'] == sha(reference / 'run.json')
    assert normalized_argv(br['argv']) == normalized_argv(ar['argv']) + ['--native-epoch-heading-attitude-seeds']
    assert '--native-imu-state-diagnostic' in ar['argv']
    assert sa['dataset_id'] == sb['dataset_id'] and set(pa) == set(pb)
    ta, tb = sa['tdcp_contract'], sb['tdcp_contract']
    assert ta['epoch_heading_attitude_seeds_requested'] is False
    assert ta['epoch_heading_attitude_seeds_inserted'] == 0
    assert tb['epoch_heading_attitude_seeds_requested'] is True
    assert tb['epoch_heading_attitude_seeds_inserted'] == len(pb)
    for s in [sa, sb]:
        assert s['imu_initialization']['source_velocity_attitude_zero_bias_initialization'] is False
    # These are actual graph/measurement invariants, not an accuracy gate.
    invariant_keys = {
        'graph': ['factors', 'upstream_stop_velocity_factors', 'upstream_stop_pose_factors', 'phase205_bias_density'],
        'tdcp_contract': ['first_imu_bias_priors_inserted', 'first_imu_bias_priors_omitted',
                         'factors_built', 'factors_inserted', 'sigma_m',
                         'height_map_factors_inserted', 'relative_height_factors_inserted'],
    }
    invariants = {}
    for section, fields in invariant_keys.items():
        invariants[section] = {}
        for field in fields:
            assert sa[section][field] == sb[section][field], (section, field)
            invariants[section][field] = sa[section][field]
    seed = 'summary.json.initialization.json'
    assert (reference / seed).read_bytes() == (candidate / seed).read_bytes()
    states = {label: inspect_states(s, p) for label, s, p in [('reference', sa, pa), ('heading', sb, pb)]}
    return dict(status='audited-heading-seed-ablation', truth_consumed=False, accuracy_evaluated=False,
                submitted=False, dataset_id=sa['dataset_id'], keys=len(pa),
                binary_sha256=ar['binary_sha256'], reference_run_sha256=sha(reference / 'run.json'),
                candidate_run_sha256=sha(candidate / 'run.json'),
                summaries_sha256=[ar['summary_sha256'], br['summary_sha256']],
                solutions_sha256=[ar['solution_sha256'], br['solution_sha256']],
                raw_initialization_sha256=sha(reference / seed), invariants=invariants,
                displacement_m=displacement(pa, pb), states=states,
                scope='One CLI option changes attitude/lever-arm seeds and low-speed heading filling. Same factors/noise, no zero-bias initialization. This does not establish test accuracy.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--candidate', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    result = audit(args.reference, args.candidate)
    args.out.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
