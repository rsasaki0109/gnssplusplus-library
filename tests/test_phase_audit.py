"""Carrier-phase audit of wrong-FIX labels (scripts/analysis/truth_dd/phase_audit.py)."""

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "phase_audit", ROOT / "scripts/analysis/truth_dd/phase_audit.py"
)
phase_audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(phase_audit)

LAT, LON = math.radians(35.69), math.radians(139.70)
ENU_TO_ECEF = np.array(
    [
        [-math.sin(LON), -math.sin(LAT) * math.cos(LON), math.cos(LAT) * math.cos(LON)],
        [math.cos(LON), -math.sin(LAT) * math.sin(LON), math.cos(LAT) * math.sin(LON)],
        [0.0, math.cos(LAT), math.sin(LAT)],
    ]
)
WAVELENGTHS = (0.19029367, 0.24421021, 0.25480000)


def scene(offset_enu, n_sats=10, seed=7):
    """DDs whose phases are integer cycles at offset_enu from the reference."""
    rng = np.random.default_rng(seed)
    az = rng.uniform(0, 2 * math.pi, n_sats)
    el = np.sort(rng.uniform(math.radians(20), math.radians(85), n_sats))[::-1]
    los = np.array(
        [ENU_TO_ECEF @ [math.cos(e) * math.sin(a), math.cos(e) * math.cos(a), math.sin(e)] for a, e in zip(az, el)]
    )
    shift = ENU_TO_ECEF @ np.asarray(offset_enu, dtype=float)
    dd, du, lam = [], [], []
    for wavelength in WAVELENGTHS:
        for k in range(1, n_sats):
            u = los[k] - los[0]
            integer = rng.integers(-50, 50)
            dd.append(wavelength * integer - u @ shift + rng.normal(0, 0.003))
            du.append(u)
            lam.append(wavelength)
    return np.array(dd), np.array(du), np.array(lam), shift


class AmbiguityFunctionTest(unittest.TestCase):
    def test_phases_that_fit_the_reference_confirm_a_wrong_fix(self):
        dd, du, lam, _ = scene((0, 0, 0))
        solution = ENU_TO_ECEF @ [0.0, 0.41, 0.0]
        af_ref = phase_audit.ambiguity_function(dd, du, lam, np.zeros(3))[0]
        af_sol = phase_audit.ambiguity_function(dd, du, lam, solution)[0]
        self.assertGreater(af_ref, 0.95)
        self.assertLess(af_sol, 0.5)
        self.assertEqual(phase_audit.label(len(dd), af_sol, af_ref), "wrong_fix_confirmed")

    def test_phases_that_fit_the_fix_mark_the_reference_inconsistent(self):
        # Shinjuku TOW 283198-283247: FIX 0.41 m north of the reference while stopped.
        dd, du, lam, shift = scene((0, 0.41, 0))
        af_ref = phase_audit.ambiguity_function(dd, du, lam, np.zeros(3))[0]
        af_sol = phase_audit.ambiguity_function(dd, du, lam, shift)[0]
        self.assertGreater(af_sol, 0.95)
        self.assertLess(af_ref, 0.5)
        self.assertEqual(phase_audit.label(len(dd), af_sol, af_ref), "reference_inconsistent")

    def test_neighbourhood_tolerates_a_few_centimetres(self):
        dd, du, lam, _ = scene((0.04, -0.03, 0.05))
        af_near, af_point = phase_audit.ambiguity_function(dd, du, lam, np.zeros(3))
        self.assertGreater(af_near, 0.95)
        self.assertLess(af_point, af_near)

    def test_few_pairs_are_undetermined(self):
        self.assertEqual(phase_audit.label(9, 1.0, 0.0), "undetermined_few_dd")
        self.assertEqual(phase_audit.label(20, 0.7, 0.6), "undetermined")


class TruthDdParsingTest(unittest.TestCase):
    def test_pairs_use_the_highest_satellite_and_skip_lli_and_low_elevation(self):
        header = "tow,sat,obs_type,lambda,rov_L,base_L,rov_P,base_P,rov_lli,base_lli,rov_snr,el_deg,rho_rov,rho_base,ux,uy,uz,base_dt\n"
        rows = [
            # sat, el, lli, rov_L, rho_rov, ux
            ("G01", 60.0, 0, 110.0, 20.0, 1.0),
            ("G02", 40.0, 0, 205.0, 30.0, 0.0),
            ("G03", 35.0, 1, 300.0, 40.0, 0.0),
            ("G04", 10.0, 0, 400.0, 50.0, 0.0),
            ("E05", 50.0, 0, 500.0, 60.0, 0.0),
        ]
        with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
            f.write(header)
            for sat, el, lli, rov_l, rho, ux in rows:
                f.write(f"100.0,{sat},L1C,0.2,{rov_l},0.0,0,0,{lli},0,40,{el},{rho},0.0,{ux},0.0,0.0,0\n")
            path = f.name
        try:
            epochs = phase_audit.dd_by_epoch(path)
        finally:
            Path(path).unlink()
        dd, du, lam = epochs[100.0]
        # Only G02 against G01: G03 has LLI, G04 is below 15 deg, E05 is alone in its group.
        self.assertEqual(len(dd), 1)
        self.assertAlmostEqual(dd[0], (0.2 * 205.0 - 30.0) - (0.2 * 110.0 - 20.0))
        np.testing.assert_allclose(du[0], [-1.0, 0.0, 0.0])
        self.assertAlmostEqual(lam[0], 0.2)


if __name__ == "__main__":
    unittest.main()
