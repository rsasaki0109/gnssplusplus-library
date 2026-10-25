"""Shared loaders for the truth-DD analyses (docs/rtk_long_lock_overconfidence.md).

The reference trajectory is used only to label solutions and to compute
analysis residuals; it is never a solver input.
"""
import csv
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'scripts'))
from generate_driving_comparison import ecef_to_enu, read_libgnss_pos, read_reference_csv  # noqa: E402


def load(drive, pos, debug_csv, data_root=REPO / 'data/driving/Tokyo_Data'):
    """3D error per POS epoch (status, error_m) keyed by TOW, and debug rows keyed by TOW."""
    ref = {(r.week, round(r.tow, 3)): r for r in read_reference_csv(Path(data_root) / drive / 'reference.csv')}
    err = {}
    for row in read_libgnss_pos(Path(pos)):
        truth = ref.get((row.week, round(row.tow, 3)))
        if truth is None:
            continue
        enu = ecef_to_enu(row.ecef - truth.ecef, truth.lat_deg, truth.lon_deg)
        err[round(row.tow, 3)] = (row.status, float(np.linalg.norm(enu)))
    rows = {}
    with open(debug_csv) as f:
        for r in csv.DictReader(f):
            rows[round(float(r['tow']), 3)] = r
    return err, rows


def num(row, key):
    try:
        return float(row.get(key, ''))
    except ValueError:
        return None


def quantile(values, p):
    values = sorted(values)
    return values[min(len(values) - 1, int(p * len(values)))] if values else None
