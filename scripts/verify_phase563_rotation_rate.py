"""No-truth structural gate for the frozen rotation-rate experiment."""
import argparse
import json
import math
from run_phase563_rotation_rate import DIRECTORY, ROOT, EXPECTED, validate


def verify(stage):
    record = json.loads((DIRECTORY / 'manifest.json').read_text())
    validate(record)
    from run_phase486_nhc_monitor import digest
    directory = DIRECTORY / stage
    done = json.loads((directory / 'completed.json').read_text())
    if done['return_code'] != 0 or not done.get('pins_verified'):
        raise ValueError('native completion/pins failed')
    if digest(directory / 'opaque_solution_output.csv') != done['output_sha256']:
        raise ValueError('output changed')
    summary = json.loads((directory / 'native_summary.json').read_text())
    previous = json.loads((ROOT / 'output/smartphone-r5/phase535-h-ionosphere-monitor-v1/mtv-h/native_summary.json').read_text())
    if stage == 'baseline':
        if done['output_sha256'] != EXPECTED or 'native_doppler_rotation_rate' in summary:
            raise ValueError('disabled output/metadata mismatch')
        for field in ('epochs', 'graph', 'gnss_first'):
            if summary[field] != previous[field]:
                raise ValueError(f'disabled {field} mismatch')
    else:
        verify('baseline')
        if summary.get('native_doppler_rotation_rate') != dict(enabled=True, gnss_first_and_main=True):
            raise ValueError('missing enabled metadata')
        # Admission can change through same-run GNSS-first seeds. Require
        # reporting such changes instead of mislabeling them as fixed rows.
        for section in ('graph', 'gnss_first'):
            if not summary[section]['converged']:
                raise ValueError(f'{section} unconverged')
            for key in ('initial_cost', 'final_cost'):
                if not math.isfinite(summary[section][key]):
                    raise ValueError(f'{section} nonfinite cost')
        if summary['epochs']['problem'] != 3140:
            raise ValueError('epoch retention changed')
        first = summary['gnss_first']
        if first['coordinates_source'] != 'in-memory GNSS-first result only' or not first['epoch_identity_alignment_valid']:
            raise ValueError('invalid handoff')
        if first['optimized_d_nonfinite_count'] or not first['optimized_d_export_valid']:
            raise ValueError('invalid drift export')
        if summary['graph']['phase213_main_doppler']['factors'] != first['doppler_factors_inserted']:
            raise ValueError('D transfer count mismatch')
    report = dict(stage=stage, verified=True, truth_reads=0,
                  main_doppler=summary['graph']['phase213_main_doppler']['factors'],
                  gnss_first_doppler=summary['gnss_first']['doppler_factors_inserted'],
                  epochs=summary['epochs'],
                  epoch_counter_changes={key: [value, summary['epochs'].get(key)]
                                         for key, value in previous['epochs'].items()
                                         if summary['epochs'].get(key) != value})
    print(json.dumps(report), flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('baseline', 'candidate'))
    verify(parser.parse_args().stage)
