"""Require trajectory and exported-state parity for a native binary replay."""
import argparse
import json
from pathlib import Path

from audit_gsdc_imu_state_diagnostic import inspect_states
from compare_gsdc_doppler_diagnostic import audited_run, normalized_argv
from run_gsdc_doppler_segment_diagnostic import sha


def audit(reference, replay):
    ar, sa, pa = audited_run(reference)
    br, sb, pb = audited_run(replay)
    assert normalized_argv(ar['argv'])[1:] == normalized_argv(br['argv'])[1:]
    assert ar['inputs_sha256'] == br['inputs_sha256']
    assert br['source_run_sha256'] == sha(reference / 'run.json')
    assert sa['dataset_id'] == sb['dataset_id'] and set(pa) == set(pb)
    assert '--native-imu-state-diagnostic' in ar['argv']
    assert '--native-epoch-heading-attitude-seeds' not in ar['argv']
    assert (reference / 'solution.csv').read_bytes() == (replay / 'solution.csv').read_bytes(), 'trajectory changed'
    seed = 'summary.json.initialization.json'
    assert (reference / seed).read_bytes() == (replay / seed).read_bytes(), 'raw initialization changed'
    states_a = inspect_states(sa, pa)
    states_b = inspect_states(sb, pb)
    assert sa['optimized_imu_states'] == sb['optimized_imu_states'], 'optimized states changed'
    assert states_a == states_b
    return dict(status='audited-imu-baseline-replay', truth_consumed=False,
                accuracy_evaluated=False, trajectory_byte_parity=True,
                optimized_state_exact_parity=True, dataset_id=sa['dataset_id'], keys=len(pa),
                reference_run_sha256=sha(reference / 'run.json'),
                replay_run_sha256=sha(replay / 'run.json'),
                binaries_sha256=[ar['binary_sha256'], br['binary_sha256']],
                summaries_sha256=[ar['summary_sha256'], br['summary_sha256']],
                solution_sha256=br['solution_sha256'], raw_initialization_sha256=sha(replay / seed),
                states=states_b)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.reference, args.replay)
    args.out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
