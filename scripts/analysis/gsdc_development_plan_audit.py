"""Audit height-map exclusions and explicitly repaired experiment provenance."""
import json
from pathlib import Path

from run_gsdc_doppler_segment_diagnostic import sha


def evaluation_group(course):
    # Established from overlapping raw UTC intervals, without reading truth:
    # docs/use_cases/records/gsdc2023_coincident_route_group_audit.json.
    aliases = {'2023-05-16-19-54-us-ca-mtv-xe1',
               '2023-05-16-19-55-us-ca-mtv-xe1'}
    return '2023-05-16-us-ca-mtv-xe1' if course in aliases else course


def audit_height_independence(plan, case):
    entry = plan['runs'][case]
    if not any('--native-height-map' in argv for argv in entry['argv'].values()):
        return
    metadata = entry['map']
    universe = set(plan['train_gt_sha256'])
    included, excluded = map(set, (metadata['included_gt_files'], metadata['excluded_gt_files']))
    assert universe and included.isdisjoint(excluded) and included | excluded == universe
    group = evaluation_group(case.split('/')[0])
    same_group = {p for p in universe if evaluation_group(Path(p).name.split('__')[0]) == group}
    assert same_group and same_group <= excluded, f'{case}: height map includes evaluation-group truth'
    for argv in entry['argv'].values():
        assert '--native-height-map' in argv
        assert Path(argv[argv.index('--native-height-map') + 1]).resolve() == Path(metadata['map']).resolve()


def provenance_hash(plan_path, plan, case, require_done=False):
    entry = plan['runs'][case]
    if plan.get('schema') == 'gsdc-repaired-frozen-development.v1':
        original_path = Path(plan['original_plan'])
        assert sha(original_path) == plan['original_plan_sha256']
        original = json.loads(original_path.read_text())
        assert set(plan['runs']) == set(original['runs'])
        assert plan['candidate_flags'] == original['candidate_flags']
        assert plan['binary_sha256'] == original['binary_sha256']
        source_path = Path(entry['provenance_plan'])
        expected = entry['provenance_plan_sha256']
        assert sha(source_path) == expected
        source = json.loads(source_path.read_text())
        if case != plan['repair_case']:
            assert source_path.resolve() == original_path.resolve()
        else:
            assert source_path.resolve() != original_path.resolve()
            assert set(source['runs']) == {case}
        assert source['binary_sha256'] == plan['binary_sha256']
        assert source['candidate_flags'] == plan['candidate_flags']
        for key in ['argv', 'inputs_sha256', 'map']:
            assert entry.get(key) == source['runs'][case].get(key)
    else:
        source_path, expected = plan_path, sha(plan_path)
    if require_done:
        done = json.loads((source_path.parent / 'execution.done.json').read_text())
        assert done['plan_sha256'] == expected
        selected = done['runs'][case]
        assert len(selected) == 2 and {r['mode'] for r in selected} == {'control', 'candidate'}
        assert all(r['state'] == 'complete' for r in selected)
    return expected
