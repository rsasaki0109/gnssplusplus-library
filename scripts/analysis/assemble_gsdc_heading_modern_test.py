"""Assemble the complete Pixel5 heading + modern recipe candidate locally."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from assemble_gsdc_frozen_test_pairs import native_rows
from audit_gsdc_pixel5_heading_test import audit_native
from compare_gsdc_doppler_diagnostic import audited_run
from run_gsdc_doppler_segment_diagnostic import sha


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def folder(entry, mode):
    argv = entry['argv'][mode]
    return Path(argv[argv.index('--out') + 1]).parent


def assemble(pixel_plan, modern_plan, reference_path, out_dir):
    # Check completeness before expensive audits or creating any output directory.
    for path in (pixel_plan, modern_plan):
        done = read(path.parent / 'execution.done.json')
        assert done['native_execution_complete'] is True
        assert done['plan_sha256'] == sha(path)
    assert not out_dir.exists(), 'output directory already exists'
    pixel, pixel_audit, _, pixel_done_hash = audit_native(pixel_plan)
    modern = read(modern_plan)
    auditor = Path(__file__).with_name('audit_gsdc_modern_test_recipe.py')
    with tempfile.TemporaryDirectory(prefix='gsdc-modern-assembly-audit-') as temporary:
        report = Path(temporary) / 'audit.json'
        subprocess.run([sys.executable, str(auditor), '--plan', str(modern_plan),
                        '--out', str(report)], check=True)
        modern_audit = read(report)
    parent_path = Path(pixel['parent_plan'])
    parent = read(parent_path)
    reference_hash = sha(reference_path)
    assert reference_hash == pixel['reference_submission_sha256']
    assert reference_hash == parent['reference_submission_sha256']
    assert reference_hash == modern['reference_submission_sha256']
    assert sha(modern['binary']) == modern['binary_sha256']
    pixel_cases, modern_cases = set(pixel['runs']), set(modern['runs'])
    assert len(pixel_cases) == 17 and len(modern_cases) == 9
    assert not pixel_cases & modern_cases
    assert modern_cases <= set(parent['retained_sources'])
    retained = set(parent['retained_sources']) - modern_cases
    assert len(retained) == 14
    with reference_path.open(newline='', encoding='utf-8-sig') as stream:
        reference = list(csv.DictReader(stream))
    reference_keys = [(r['tripId'], int(r['UnixTimeMillis'])) for r in reference]
    assert len(reference_keys) == len(set(reference_keys)) == 71936
    expected_cases = pixel_cases | modern_cases | retained
    assert len(expected_cases) == 40
    assert expected_cases == {case for case, _ in reference_keys}
    output, evidence = {}, {}
    for label, plan, plan_path in [('pixel5-heading', pixel, pixel_plan),
                                   ('modern-clock-recipe', modern, modern_plan)]:
        for case, entry in plan['runs'].items():
            directory = folder(entry, 'candidate')
            record, summary, points = audited_run(directory)
            assert record['argv'] == entry['argv']['candidate']
            assert record['source_run_sha256'] == sha(plan_path)
            assert record['binary_sha256'] == plan['binary_sha256']
            assert record['inputs_sha256'] == entry['inputs_sha256']
            assert summary['dataset_id'] == case
            output[case] = native_rows(directory / 'solution.csv', case)
            assert set(output[case]) == set(points)
            evidence[case] = dict(mode=label, native_run=str(directory / 'run.json'),
                run_sha256=sha(directory / 'run.json'),
                solution_sha256=record['solution_sha256'], summary_sha256=record['summary_sha256'])
    # Heading controls reproduce the earlier recipe candidates; also verify the
    # earlier controls against the native source of the official submission.
    for case in pixel_cases:
        entry = parent['runs'][case]
        published = entry['published_source']
        control_dir = folder(entry, 'control')
        record, summary, _ = audited_run(control_dir)
        assert record['argv'] == entry['argv']['control']
        assert record['source_run_sha256'] == sha(parent_path)
        assert record['binary_sha256'] == parent['binary_sha256']
        assert record['inputs_sha256'] == entry['inputs_sha256']
        assert summary['dataset_id'] == case
        assert sha(published['solution']) == published['solution_sha256']
        assert record['solution_sha256'] == published['solution_sha256']
        evidence[case]['published_control_solution_sha256'] = published['solution_sha256']
    for case in retained:
        entry = parent['retained_sources'][case]
        path = Path(entry['solution'])
        assert sha(path) == entry['solution_sha256']
        assert sha(entry['run']) == entry['run_sha256']
        assert sha(path.parent / 'summary.json') == entry['summary_sha256']
        record, summary = read(entry['run']), read(path.parent / 'summary.json')
        assert record['returncode'] == 0 and summary['truth_used'] is False
        assert summary['dataset_id'] == case
        assert summary['native_pdc_imu_tdcp_no_bridge'] is True
        assert summary['graph']['converged']
        assert all(summary['raw_utc_key_contract'][key] == 0 for key in
                   ['interpolated_epochs', 'edge_hold_epochs', 'unresolved_epochs'])
        output[case] = native_rows(path, case)
        evidence[case] = dict(mode='retained-native', **entry)
    rows = []
    for original in reference:
        case, utc = original['tripId'], int(original['UnixTimeMillis'])
        lat, lon = output[case][utc]  # Missing native keys must fail; no interpolation.
        if case in retained:
            assert (lat, lon) == (original['LatitudeDegrees'], original['LongitudeDegrees'])
        rows.append(dict(tripId=case, UnixTimeMillis=original['UnixTimeMillis'],
                         LatitudeDegrees=lat, LongitudeDegrees=lon))
    manifest = dict(status='assembled-locally-not-submitted', submitted=False,
        official_score=False, accuracy_evaluated=False, evaluation_truth_used=False,
        reference_coordinates_used_for_inference=False, all_output_rows_native=True,
        rows=len(rows), drives=40, replaced_drives=26, retained_drives=14,
        pixel5_plan_sha256=sha(pixel_plan), modern_plan_sha256=sha(modern_plan),
        pixel5_execution_done_sha256=pixel_done_hash,
        modern_execution_done_sha256=modern_audit['execution_done_sha256'],
        reference_submission_sha256=reference_hash, assembler_sha256=sha(__file__),
        source_evidence=evidence, pixel5_audit=pixel_audit, modern_audit=modern_audit)
    out_dir.mkdir(parents=True, exist_ok=False)
    destination = out_dir / 'submission.csv'
    with destination.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=['tripId', 'UnixTimeMillis',
            'LatitudeDegrees', 'LongitudeDegrees'], lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    manifest['submission_sha256'] = sha(destination)
    (out_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pixel5-plan', type=Path, required=True)
    parser.add_argument('--modern-plan', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    result = assemble(args.pixel5_plan, args.modern_plan, args.reference, args.out_dir)
    print(json.dumps({key: value for key, value in result.items()
                      if key not in ['source_evidence', 'pixel5_audit', 'modern_audit']}, indent=2))


if __name__ == '__main__':
    main()
