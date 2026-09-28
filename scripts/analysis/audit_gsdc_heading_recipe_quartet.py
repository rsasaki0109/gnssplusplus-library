"""Compare a frozen recipe pair before/after heading seeds, without truth."""
import argparse
import json
from pathlib import Path

import numpy as np

from audit_gsdc_heading_seed_ablation import audit
from compare_gsdc_doppler_diagnostic import audited_run, normalized_argv
from assemble_gsdc_frozen_test_pairs import displacement
from run_gsdc_doppler_segment_diagnostic import sha


def compare(root):
    ablations = {}
    runs = {}
    for arm in ['control', 'candidate']:
        baseline = root / f'{arm}_baseline/control'
        heading = root / f'{arm}_heading/heading'
        ablations[arm] = audit(baseline, heading)
        for setting, folder in [('baseline', baseline), ('heading', heading)]:
            runs[arm, setting] = (folder, audited_run(folder))
    flags = ['--native-source-tdcp-meter-sigma',
             '--native-phase184-source-tdcp-huber-k',
             '--native-phase213-main-doppler']
    pair_displacements = {}
    metrics = {}
    for setting in ['baseline', 'heading']:
        ca, cb = (runs[arm, setting][1] for arm in ['control', 'candidate'])
        a, b = ca[0], cb[0]
        assert a['binary_sha256'] == b['binary_sha256']
        assert a['inputs_sha256'] == b['inputs_sha256']
        assert ca[1]['dataset_id'] == cb[1]['dataset_id']
        assert set(ca[2]) == set(cb[2])
        aa, bb = normalized_argv(a['argv']), normalized_argv(b['argv'])
        assert all(flag not in aa and bb.count(flag) == 1 for flag in flags)
        assert aa == [arg for arg in bb if arg not in flags]
        pair_displacements[setting] = displacement(ca[2], cb[2])
        for arm in ['control', 'candidate']:
            folder, (record, summary, positions) = runs[arm, setting]
            velocity = np.asarray([row['velocity_enu_mps']
                                   for row in summary['optimized_imu_states']['epochs']])
            metrics[f'{arm}_{setting}'] = dict(
                run_sha256=sha(folder / 'run.json'),
                solution_sha256=record['solution_sha256'],
                summary_sha256=record['summary_sha256'],
                keys=len(positions),
                max_absolute_vertical_velocity_mps=float(np.abs(velocity[:, 2]).max()),
                max_horizontal_velocity_mps=float(np.linalg.norm(velocity[:, :2], axis=1).max()),
                tdcp_sigma_m=summary['tdcp_contract']['sigma_m'])
    return dict(status='audited-heading-recipe-quartet', truth_consumed=False,
                accuracy_evaluated=False, submitted=False, recipe_flags=flags,
                pair_displacement_m=pair_displacements, runs=metrics, ablations=ablations,
                scope='Trajectory differences and optimized states are not truth errors. '
                      'Heading changes attitude/lever-arm seeds and low-speed heading filling.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.root)
    args.out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({key: value for key, value in result.items() if key != 'ablations'}, indent=2))


if __name__ == '__main__':
    main()
