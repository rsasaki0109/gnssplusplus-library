"""Freeze a uniform Pixel5 development comparison against the height recipe."""
import argparse
import json
from pathlib import Path

from run_gsdc_doppler_segment_diagnostic import sha
from gsdc_development_plan_audit import audit_height_independence, evaluation_group


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parent-plan', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--train-gt', type=Path, required=True)
    parser.add_argument('--map-builder', type=Path, required=True)
    args = parser.parse_args()
    parent = json.loads(args.parent_plan.read_text())
    maps = json.loads((args.root / 'maps/report.json').read_text())
    selected = {k: v for k, v in parent['runs'].items() if k.endswith('/pixel5')}
    assert len(selected) == 15 and set(maps) == set(selected)
    gt = {str(p.resolve()): sha(p) for p in sorted(args.train_gt.glob('*ground_truth.csv'))}
    assert len(gt) == 156
    extra = ['--native-source-tdcp-meter-sigma', '--native-phase184-source-tdcp-huber-k',
             '--native-phase213-main-doppler']
    common = ['--native-upstream-position-offset',
              '--native-base-pseudorange-preserve-additional-frequency-bands']
    plan = dict(schema='gsdc-pixel5-source-recipe-development.v1',
                official_score=False, heldout=False, status='prepared-not-started',
                parent_plan_sha256=sha(args.parent_plan), binary=str(args.binary.resolve()),
                binary_sha256=sha(args.binary), candidate_flags=extra, common_flags=common,
                height_policy='same in both arms; map if leave-course-out coverage >= 10%, otherwise relative pairs',
                selection='all 15 Pixel5 cases in the frozen corrected train40 plan; no per-route winner selection',
                map_builder_sha256=sha(args.map_builder), train_gt_sha256=gt, runs={})
    for index, (case, source) in enumerate(selected.items()):
        course = case.split('/')[0]
        m = maps[case]
        assert m['excluded_course'] == course
        group = evaluation_group(course)
        assert m.get('excluded_evaluation_group', m['excluded_course']) == group
        seed = Path(source['folder']) / 'solution.csv'
        seed_run = json.loads((Path(source['folder']) / 'run.json').read_text())
        assert seed_run['returncode'] == 0 and seed.is_file()
        # The builder removes every phone's GT belonging to this course.
        excluded = [p for p in gt if evaluation_group(Path(p).name.split('__')[0]) == group]
        included = [p for p in gt if p not in excluded]
        assert all(Path(p).name.split('__')[0] != course for p in included)
        height = ['--native-height-map', m['map']] if m['coverage_15m'] >= .10 and m['points'] else ['--native-relative-height-pairs']
        argv = [str(args.binary.resolve())] + source['argv'][1:]
        assert all(flag not in argv for flag in extra + common)
        argv += common + height
        inputs = {v['path']: v['sha256'] for v in source['inputs']}
        assert all(Path(p).is_file() and sha(p) == digest for p, digest in inputs.items())
        if height[0] == '--native-height-map':
            inputs[height[1]] = sha(height[1])
        record = dict(dataset_id=case, route_group=course, inputs_sha256=inputs,
                      map=dict(**m, excluded_gt_files=excluded, included_gt_files=included,
                               seed_solution_sha256=sha(seed)), argv={})
        for mode in ['control', 'candidate']:
            folder = args.root / 'runs' / f'{index:02d}' / mode
            command = argv.copy()
            for flag, filename in [('--out', 'solution.csv'), ('--summary-json', 'summary.json')]:
                command[command.index(flag) + 1] = str((folder / filename).resolve())
            if mode == 'candidate':
                command += extra
            record['argv'][mode] = command
        plan['runs'][case] = record
        audit_height_independence(plan, case)
    with (args.root / 'plan.json').open('x', encoding='utf-8') as stream:
        json.dump(plan, stream, indent=2)
    print(json.dumps(dict(cases=len(plan['runs']), plan_sha256=sha(args.root / 'plan.json'))))


if __name__ == '__main__':
    main()
