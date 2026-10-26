"""phase_audit.py reference.csv truth_dd.csv out.json NAME=lib:POS|NAME=rtklib:POS ...

Carrier-phase audit of wrong-FIX labels (docs/rtk_phase_audit.md).

A FIX epoch more than 0.5 m from the reference is either a wrong FIX or a
reference error. The DD carrier phases of export_truth_dd decide between the
two. The ambiguity function

    AF(x) = mean over DD pairs of cos(2*pi * (DD phase - DD range(x)) / lambda)

is 1 when every DD is an integer number of cycles from position x and about 0
for a position that does not fit the phases. DD range(x) is moved from the
reference position with the rover line of sight (linear in dx). AFn(x) is
the maximum of AF on a 3D grid within +-0.10 m of x, applied to the reference
and to the solver position alike.

Labels for a FIX epoch whose 3D error exceeds 0.5 m:
  n_DD < 10                          -> undetermined_few_dd
  AFn(sol) >= 0.8 and AFn(ref) < 0.5 -> reference_inconsistent
  AFn(ref) >= 0.8 and AFn(sol) < 0.5 -> wrong_fix_confirmed
  otherwise                          -> undetermined

The reference is used only for scoring; nothing here is a solver input.
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'scripts'))
from generate_driving_comparison import (  # noqa: E402
    ecef_to_enu,
    read_libgnss_pos,
    read_reference_csv,
    read_rtklib_pos,
)

WRONG_M = 0.5
CONTROL_M = 0.10
MIN_ELEVATION_DEG = 15.0
MIN_DD = 10
AF_CONSISTENT = 0.8
AF_INCONSISTENT = 0.5
_STEPS = np.arange(-0.10, 0.10001, 0.02)
NEIGHBOURHOOD = np.array([(a, b, c) for a in _STEPS for b in _STEPS for c in _STEPS])


def epoch_key(tow):
    return round(tow, 1)


def dd_by_epoch(truth_dd_csv, min_elevation_deg=MIN_ELEVATION_DEG):
    """DD phase minus DD reference range (m), LOS difference and wavelength per epoch.

    Pairs are formed within each (system, observation type) group against its
    highest-elevation satellite; rows with a rover LLI flag are skipped.
    """
    groups = defaultdict(lambda: defaultdict(list))
    with open(truth_dd_csv) as f:
        head = f.readline().strip().split(',')
        ix = {c: i for i, c in enumerate(head)}
        for line in f:
            c = line.rstrip('\n').split(',')
            if c[ix['rov_lli']] not in ('0', '') or float(c[ix['el_deg']]) < min_elevation_deg:
                continue
            lam = float(c[ix['lambda']])
            d = (lam * (float(c[ix['rov_L']]) - float(c[ix['base_L']]))
                 - (float(c[ix['rho_rov']]) - float(c[ix['rho_base']])))
            los = (float(c[ix['ux']]), float(c[ix['uy']]), float(c[ix['uz']]))
            groups[epoch_key(float(c[ix['tow']]))][(c[ix['sat']][0], c[ix['obs_type']])].append(
                (float(c[ix['el_deg']]), d, los, lam))
    epochs = {}
    for key, by_signal in groups.items():
        dd, du, lam = [], [], []
        for rows in by_signal.values():
            if len(rows) < 2:
                continue
            rows.sort(key=lambda r: -r[0])
            ref = rows[0]
            for row in rows[1:]:
                dd.append(row[1] - ref[1])
                du.append(np.subtract(row[2], ref[2]))
                lam.append(row[3])
        epochs[key] = (np.array(dd), np.array(du).reshape(-1, 3), np.array(lam))
    return epochs


def ambiguity_function(dd, du, lam, dx):
    """(AFn, AF) at offset dx (ECEF, m) from the reference position."""
    pts = np.asarray(dx, dtype=float)[None, :] + NEIGHBOURHOOD
    cycles = (dd[None, :] + pts @ du.T) / lam[None, :]
    af_near = float(np.cos(2 * np.pi * cycles).mean(axis=1).max())
    af = float(np.cos(2 * np.pi * (dd + du @ np.asarray(dx, dtype=float)) / lam).mean())
    return af_near, af


def label(n_dd, af_solution, af_reference):
    if n_dd < MIN_DD:
        return 'undetermined_few_dd'
    if af_solution >= AF_CONSISTENT and af_reference < AF_INCONSISTENT:
        return 'reference_inconsistent'
    if af_reference >= AF_CONSISTENT and af_solution < AF_INCONSISTENT:
        return 'wrong_fix_confirmed'
    return 'undetermined'


def audit(reference, epochs, rows, fixed_status, control_stride=10):
    """Label every FIX epoch beyond WRONG_M; sample correct FIX epochs as controls."""
    wrong, controls = [], []
    for row in rows:
        key = epoch_key(row.tow)
        truth = reference.get(key)
        if row.status != fixed_status or abs(row.tow - key) > 0.011 or truth is None:
            continue
        dx = row.ecef - truth.ecef
        err = float(np.linalg.norm(ecef_to_enu(dx, truth.lat_deg, truth.lon_deg)))
        if err > WRONG_M:
            wrong.append((key, dx, err))
        elif err < CONTROL_M:
            controls.append((key, dx))
    labelled = []
    counts = defaultdict(int)
    for key, dx, err in wrong:
        if key not in epochs or len(epochs[key][0]) == 0:
            lab, n, af_sol, af_ref = 'undetermined_no_dd', 0, None, None
        else:
            dd, du, lam = epochs[key]
            n = len(dd)
            af_sol = ambiguity_function(dd, du, lam, dx)[0]
            af_ref = ambiguity_function(dd, du, lam, np.zeros(3))[0]
            lab = label(n, af_sol, af_ref)
        counts[lab] += 1
        labelled.append({'tow': key, 'err_m': round(err, 3), 'n_dd': n,
                         'afn_solution': af_sol, 'afn_reference': af_ref, 'label': lab})
    sampled = both = as_reference_inconsistent = 0
    for i, (key, dx) in enumerate(controls):
        if i % control_stride or key not in epochs or len(epochs[key][0]) < MIN_DD:
            continue
        dd, du, lam = epochs[key]
        af_sol = ambiguity_function(dd, du, lam, dx)[0]
        af_ref = ambiguity_function(dd, du, lam, np.zeros(3))[0]
        sampled += 1
        both += af_sol >= AF_CONSISTENT and af_ref >= AF_CONSISTENT
        as_reference_inconsistent += label(len(dd), af_sol, af_ref) == 'reference_inconsistent'
    return {'n_wrong': len(wrong), 'labels': dict(counts), 'wrong': labelled,
            'controls': {'sampled': sampled, 'both_afn_consistent': both,
                         'labelled_reference_inconsistent': as_reference_inconsistent}}


def main(argv):
    if len(argv) < 4:
        print(__doc__.splitlines()[0], file=sys.stderr)
        return 2
    reference = {epoch_key(r.tow): r for r in read_reference_csv(Path(argv[0]))}
    epochs = dd_by_epoch(argv[1])
    result = {'reference': argv[0], 'truth_dd': argv[1], 'solvers': {}}
    for spec in argv[3:]:
        name, kind_path = spec.split('=', 1)
        kind, path = kind_path.split(':', 1)
        rows = read_libgnss_pos(Path(path)) if kind == 'lib' else read_rtklib_pos(Path(path))
        rec = audit(reference, epochs, rows, 4 if kind == 'lib' else 1)
        result['solvers'][name] = rec
        print(name, 'wrong', rec['n_wrong'], rec['labels'], 'controls', rec['controls'])
    Path(argv[2]).write_text(json.dumps(result, indent=1) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
