"""Prepare, but do not execute, a Pixel5 comparison against the submitted recipe."""
import argparse
import csv
import json
from pathlib import Path

from run_gsdc_doppler_segment_diagnostic import sha
from gsdc_development_plan_audit import audit_height_independence, provenance_hash


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-root', type=Path, required=True)
    parser.add_argument('--development-plan', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    root = args.native_root
    published = root / 'all40_height_submission_v1'
    manifest = read(published / 'manifest.json')
    receipt = read(published / 'kaggle_receipt.json')
    submission = published / 'submission.csv'
    digest = sha(submission)
    assert digest == manifest['submission_sha256'] == receipt['submission_sha256']
    assert receipt['submitted'] and receipt['submission_ref'] == 56536540
    assert manifest['all_output_rows_native'] and not manifest['sample_coordinates_used']
    with submission.open(newline='', encoding='utf-8-sig') as stream:
        submitted_rows = list(csv.DictReader(stream))
    assert len(submitted_rows) == manifest['rows'] == 71936
    keys = [(r['tripId'], int(r['UnixTimeMillis'])) for r in submitted_rows]
    assert len(set(keys)) == len(keys)
    cases = set(r['tripId'] for r in submitted_rows)
    assert len(cases) == manifest['drives'] == 40
    assert cases == set(manifest['height_modes'])
    extra_root = root / 'test40_offset_extra_bands_v1'
    audit = read(extra_root / 'audit.json')['runs']
    assert set(audit) == cases
    development = read(args.development_plan)
    assert len(development['runs']) == 15
    for case in development['runs']:
        assert case.endswith('/pixel5')
        audit_height_independence(development, case)
        provenance_hash(args.development_plan, development, case)
    binary = Path(development['binary'])
    assert sha(binary) == development['binary_sha256']
    flags = development['candidate_flags']
    assert flags == ['--native-source-tdcp-meter-sigma',
                     '--native-phase184-source-tdcp-huber-k',
                     '--native-phase213-main-doppler']
    plan = dict(schema='gsdc-pixel5-submitted-test-recipe.v1',
                status='prepared-not-started', submitted=False, official_score=False,
                execution_gate='Review the complete fixed 15-case development comparison first; no automatic promotion.',
                reference_submission_sha256=digest, reference_submission_ref=receipt['submission_ref'],
                development_plan=str(args.development_plan.resolve()),
                development_plan_sha256=sha(args.development_plan),
                binary=str(binary), binary_sha256=sha(binary), candidate_flags=flags,
                selection='All 17 Pixel5 test phones; same three candidate flags on every phone.',
                height_policy='Exactly the per-phone recipe used by submission 56536540, including lax-p without height.',
                control_gate='Verify each replayed control against its published native source before assembling any candidate.',
                retained_sources={}, runs={})
    source_positions = {}
    for case in sorted(cases):
        extra = audit[case]
        name = Path(extra['argv'][extra['argv'].index('--out') + 1]).parent.name
        mode = manifest['height_modes'][case]
        folder = (extra_root if mode == 'fallback-extra-bands-no-height'
                  else root / 'test40_height_v1') / name
        record_path = folder / 'run.json'
        record = read(record_path)
        assert record['returncode'] == 0 and record['dataset_id'] == case
        summary = read(folder / 'summary.json')
        assert summary['truth_used'] is False and summary['native_upstream_position_offset'] is True
        with (folder / 'solution.csv').open(newline='', encoding='utf-8-sig') as stream:
            rows = list(csv.DictReader(stream))
        positions = {int(r['UnixTimeMillis']): (r['LatitudeDegrees'], r['LongitudeDegrees']) for r in rows}
        assert len(positions) == len(rows) and all(r['phone'] == case for r in rows)
        source_positions[case] = positions
        provenance = dict(run=str(record_path), run_sha256=sha(record_path),
                          solution=str(folder / 'solution.csv'), solution_sha256=sha(folder / 'solution.csv'),
                          summary_sha256=sha(folder / 'summary.json'), height_mode=mode)
        if not case.endswith('/pixel5'):
            plan['retained_sources'][case] = provenance
            continue
        native_source = Path(extra['source_run_record'])
        source_record = read(native_source)
        assert source_record['returncode'] == 0
        inputs = {entry['path']: entry['sha256'] for entry in source_record['inputs']}
        assert all(sha(p) == expected for p, expected in inputs.items())
        argv = [str(binary)] + record['argv'][1:]
        assert all(flag not in argv for flag in flags)
        if mode == 'map':
            path = argv[argv.index('--native-height-map') + 1]
            inputs[path] = sha(path)
        elif mode == 'relative':
            assert '--native-relative-height-pairs' in argv
        else:
            assert mode == 'fallback-extra-bands-no-height'
            assert '--native-height-map' not in argv and '--native-relative-height-pairs' not in argv
        entry = dict(dataset_id=case, published_source=provenance,
                     historical_input_manifest=str(native_source),
                     historical_input_manifest_sha256=sha(native_source), inputs_sha256=inputs, argv={})
        index = len(plan['runs'])
        for arm in ['control', 'candidate']:
            command = argv.copy()
            folder = args.out_dir / 'runs' / f'{index:02d}' / arm
            for flag, filename in [('--out', 'solution.csv'), ('--summary-json', 'summary.json')]:
                command[command.index(flag) + 1] = str((folder / filename).resolve())
            if arm == 'candidate':
                command += flags
            entry['argv'][arm] = command
        plan['runs'][case] = entry
    # Recover every published coordinate from its declared completed native run.
    # The published coordinates are comparison evidence, never solver input.
    for row in submitted_rows:
        assert source_positions[row['tripId']][int(row['UnixTimeMillis'])] == (
            row['LatitudeDegrees'], row['LongitudeDegrees'])
    assert len(plan['runs']) == 17 and len(plan['retained_sources']) == 23
    plan['published_rows_reconciled'] = len(submitted_rows)
    args.out_dir.mkdir(parents=True, exist_ok=False)
    with (args.out_dir / 'plan.json').open('x', encoding='utf-8') as stream:
        json.dump(plan, stream, indent=2)
    print(json.dumps(dict(selected=17, retained=23, published_rows_reconciled=len(submitted_rows),
                          plan_sha256=sha(args.out_dir / 'plan.json'), executed=False)))


if __name__ == '__main__':
    main()
