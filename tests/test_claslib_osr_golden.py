#!/usr/bin/env python3
"""Golden CLAS parity integration tests for SSR-facing and fixed-phase debug values."""

from __future__ import annotations

import os
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
WRAPPER = ROOT_DIR / "scripts" / "run_gnss_ppp_claslib_parity.sh"
THESIS_ROOT = Path("")
CLAS_ROOT = THESIS_ROOT / "data" / "clas" / "claslib"
CLAS_DATA = CLAS_ROOT / "data"
OBS_2019 = CLAS_DATA / "0627239Q.obs"
NAV_2019 = CLAS_DATA / "sept_2019239.nav"
SSR_2019 = CLAS_DATA / "2019239Q.l6"
REF_X = -3957235.3717
REF_Y = 3310368.2257
REF_Z = 3737529.7179

BIAS_SEL_RE = re.compile(
    r"^\[SSR-BIAS-SEL\]\s+(?P<sat>[A-Z0-9]+)\s+tow=(?P<tow>\d+)\s+"
    r"pref_net=(?P<pref>-?\d+)\s+atmos_net=(?P<atmos>-?\d+)\s+"
    r"phase_net=(?P<phase>-?\d+)\s+code_net=(?P<code>-?\d+)$"
)
OSR_SSR_RE = re.compile(
    r"^\[OSR-SSR\]\s+(?P<sat>[A-Z0-9]+)\s+orbit_ecef=\s*"
    r"(?P<x>[-+0-9.eE]+)\s+(?P<y>[-+0-9.eE]+)\s+(?P<z>[-+0-9.eE]+)\s+"
    r"clk_m=(?P<clk>[-+0-9.eE]+).*\s+cbias_n=(?P<cbias_n>\d+)\s+pbias_n=(?P<pbias_n>\d+)$"
)
OSR_FULL_RE = re.compile(r"^\[OSR\]\s+(?P<sat>[A-Z0-9]+)\s+(?P<body>.*)$")
CLAS_PHASE_ROW_RE = re.compile(r"^\[CLAS-PHASE-ROW\]\s+(?:tow=[-+0-9.eE]+\s+)?sat=(?P<sat>[A-Z0-9]+)\s+f=(?P<f>\d+)\s+(?P<body>.*)$")
CLAS_IF_PHASE_RE = re.compile(r"^\[CLAS-IF-PHASE\]\s+(?:tow=[-+0-9.eE]+\s+)?sat=(?P<sat>[A-Z0-9]+)\s+(?P<body>.*)$")
CLAS_WLNL_FIX_RE = re.compile(r"^\[CLAS-WLNL-FIX\]\s+(?P<body>.*)$")
KV_RE = re.compile(r"(?P<key>[A-Za-z_][A-Za-z0-9_]*)=(?P<value>[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)")

GOLDEN_SSR = {
    "G25": {
        "tow": 230420,
        "pref_net": 7,
        "atmos_net": 7,
        "phase_net": 7,
        "code_net": 0,
        "orbit_ecef": (1.91341, 0.742095, 0.708304),
        "clk_m": -0.4592,
        "cbias_n": 4,
        "pbias_n": 4,
    },
    "G26": {
        "tow": 230420,
        "pref_net": 7,
        "atmos_net": 7,
        "phase_net": 7,
        "code_net": 0,
        "orbit_ecef": (0.495233, -0.764205, 0.699193),
        "clk_m": -0.1808,
        "cbias_n": 4,
        "pbias_n": 4,
    },
    "G31": {
        "tow": 230420,
        "pref_net": 7,
        "atmos_net": 7,
        "phase_net": 7,
        "code_net": 0,
        "orbit_ecef": (0.74301, -1.20127, -0.329653),
        "clk_m": 0.872,
        "cbias_n": 3,
        "pbias_n": 3,
    },
}

GOLDEN_PHASE_ROWS = {
    ("G25", 0): {
        "phase_corr": -1.56670509681,
        "trop": 3.55029865104,
        "iono_term": 0.0000227787,
        "amb": -1.72117961486,
        "residual": 9.36700389907,
    },
    ("G26", 0): {
        "phase_corr": -2.14570345466,
        "trop": 4.22192148185,
        "iono_term": 0.000859381,
        "amb": 19.8843318702,
        "residual": 9.41576893255,
    },
    ("G31", 0): {
        "phase_corr": 1.99113312887,
        "trop": 3.01859747629,
        "iono_term": -0.000356353,
        "amb": -5.17807993257,
        "residual": 9.44992027804,
    },
}

GOLDEN_IF_PHASE = {
    "G25": {
        "if_raw": 0.136859308368,
        "amb_gap": 0.0,
        "beta_amb": -8.10210163286,
        "base_if_amb": -8.10210163286,
        "if_residual": 0.136859308368,
    },
    "G26": {
        "if_raw": 0.0872934011318,
        "amb_gap": 0.0,
        "beta_amb": 31.2021028156,
        "base_if_amb": 31.2021028156,
        "if_residual": 0.0872934011318,
    },
    "G31": {
        "if_raw": 0.0547902206796,
        "amb_gap": 0.0,
        "beta_amb": -9.29335230904,
        "base_if_amb": -9.29335230904,
        "if_residual": 0.0547902206796,
    },
}

GOLDEN_WLNL_FIX = {
    "rows": 12,
    "constraints": 8,
    "solved": 1,
    "pos_shift": 0.188436447086,
    "phase_rms": 0.0549538265232,
    "dd_mean": 0.16176716528,
    "dd_max": 0.485454163651,
}

GOLDEN_OSR_EPOCH2 = {
    "G25": {
        "trop": 3.79638,
        "iono_l1": 5.44928,
        "cbias1": 0.66,
        "pbias1": 11.182,
        "orb_los": -0.523482,
        "clk_corr": -0.46336,
        "PRC0": 7.89213218156,
        "CPC0": 2.22967208924,
        "CPC1": 7.85350350422,
    },
    "G26": {
        "trop": 4.51478,
        "iono_l1": 8.27746,
        "cbias1": 0.72,
        "pbias1": 6.126,
        "orb_los": -0.711566,
        "clk_corr": -0.18464,
        "PRC0": 10.8728798142,
        "CPC0": 2.36907623938,
        "CPC1": -0.0529261396889,
    },
    "G31": {
        "trop": 3.22771,
        "iono_l1": 0.108782,
        "cbias1": 0.28,
        "pbias1": 6.461,
        "orb_los": -0.979298,
        "clk_corr": 0.86784,
        "PRC0": 3.15682823654,
        "CPC0": 5.21884641637,
        "CPC1": 9.38283172691,
    },
}

GOLDEN_30_SUMMARY = {
    "processed_epochs": 30,
    "valid_solutions": 30,
    "ppp_float_solutions": 0,
    "ppp_fixed_solutions": 30,
    "fallback_solutions": 0,
    "clas_hybrid_fallback_epochs": 0,
    "ppp_solution_rate_pct": 100.0,
    "converged": False,
}

GOLDEN_30_LAST_CLAS_PPP = {
    "rows": 43,
    "sats": 11,
    "pos_delta": 0.00574770360806,
    "code_rms": 0.304835660391,
    "phase_rms": 0.00435561082115,
}

GOLDEN_30_LAST_WLNL_FIX = {
    "rows": 12,
    "constraints": 8,
    "solved": 1,
    "pos_shift": 0.00207144331552,
    "phase_rms": 0.0506760856094,
    "dd_mean": 0.0184477530032,
    "dd_max": 0.0309184880712,
}

GOLDEN_30_LAST_OSR = {
    "G25": {
        "trop": 3.81732,
        "iono_l1": 5.44344,
        "cbias1": 0.64,
        "pbias1": 11.182,
        "orb_los": -0.513384,
        "clk_corr": -0.39808,
        "PRC0": 7.90880283417,
        "CPC0": 2.25539689139,
        "CPC1": 7.88216629579,
    },
    "G26": {
        "trop": 4.50661,
        "iono_l1": 8.23314,
        "cbias1": 0.7,
        "pbias1": 6.063,
        "orb_los": -0.694577,
        "clk_corr": -0.1152,
        "PRC0": 10.8293888721,
        "CPC0": 2.34362227946,
        "CPC1": -0.0680501456595,
    },
    "G31": {
        "trop": 3.22915,
        "iono_l1": 0.0925156,
        "cbias1": 0.28,
        "pbias1": 6.429,
        "orb_los": -0.984987,
        "clk_corr": 0.9024,
        "PRC0": 3.1450512134,
        "CPC0": 5.20435447348,
        "CPC1": 9.37252203975,
    },
}


def _resolve_gnss_ppp_bin() -> Path:
    for candidate in (ROOT_DIR / "build" / "gnss_ppp", ROOT_DIR / "build" / "apps" / "gnss_ppp"):
        if candidate.is_file():
            return candidate
    return ROOT_DIR / "build" / "gnss_ppp"


PPP_BIN = _resolve_gnss_ppp_bin()


def _integration_ready() -> bool:
    required = (PPP_BIN, WRAPPER, OBS_2019, NAV_2019, SSR_2019)
    return all(path.is_file() for path in required)


def _run_parity_debug(max_epochs: int = 2) -> subprocess.CompletedProcess[str]:
    with tempfile.TemporaryDirectory(prefix="gnsspp_clas_osr_") as tmp_str:
        tmp = Path(tmp_str)
        out_pos = tmp / "osr_golden.pos"
        summary_json = tmp / "osr_golden_summary.json"
        env = {
            **os.environ,
            "GNSS_PPP": str(PPP_BIN),
            "GNSS_PPP_DEBUG": "1",
        }
        return subprocess.run(
            [
                str(WRAPPER),
                "--obs",
                str(OBS_2019),
                "--nav",
                str(NAV_2019),
                "--ssr",
                str(SSR_2019),
                "--out",
                str(out_pos),
                "--summary-json",
                str(summary_json),
                "--ref-x",
                str(REF_X),
                "--ref-y",
                str(REF_Y),
                "--ref-z",
                str(REF_Z),
                "--max-epochs",
                str(max_epochs),
                "--legacy-strict-parity",
            ],
            cwd=ROOT_DIR,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )


def _run_parity_debug_with_summary(max_epochs: int) -> tuple[subprocess.CompletedProcess[str], dict[str, object]]:
    with tempfile.TemporaryDirectory(prefix="gnsspp_clas_osr_") as tmp_str:
        tmp = Path(tmp_str)
        out_pos = tmp / "osr_golden.pos"
        summary_json = tmp / "osr_golden_summary.json"
        env = {
            **os.environ,
            "GNSS_PPP": str(PPP_BIN),
            "GNSS_PPP_DEBUG": "1",
        }
        run = subprocess.run(
            [
                str(WRAPPER),
                "--obs",
                str(OBS_2019),
                "--nav",
                str(NAV_2019),
                "--ssr",
                str(SSR_2019),
                "--out",
                str(out_pos),
                "--summary-json",
                str(summary_json),
                "--ref-x",
                str(REF_X),
                "--ref-y",
                str(REF_Y),
                "--ref-z",
                str(REF_Z),
                "--max-epochs",
                str(max_epochs),
                "--legacy-strict-parity",
            ],
            cwd=ROOT_DIR,
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        import json

        summary = json.loads(summary_json.read_text(encoding="utf-8"))
        return run, summary


def _parse_kv(body: str) -> dict[str, float]:
    return {m.group("key"): float(m.group("value")) for m in KV_RE.finditer(body)}


def _collect_first_epoch_ssr_debug(stderr_text: str) -> tuple[dict[str, dict[str, int]], dict[str, dict[str, object]]]:
    bias_rows: dict[str, dict[str, int]] = {}
    osr_rows: dict[str, dict[str, object]] = {}
    for raw_line in stderr_text.splitlines():
        line = raw_line.strip()
        bias_match = BIAS_SEL_RE.match(line)
        if bias_match is not None:
            sat = bias_match.group("sat")
            if sat in GOLDEN_SSR and sat not in bias_rows:
                bias_rows[sat] = {
                    "tow": int(bias_match.group("tow")),
                    "pref_net": int(bias_match.group("pref")),
                    "atmos_net": int(bias_match.group("atmos")),
                    "phase_net": int(bias_match.group("phase")),
                    "code_net": int(bias_match.group("code")),
                }
            continue
        osr_match = OSR_SSR_RE.match(line)
        if osr_match is not None:
            sat = osr_match.group("sat")
            if sat in GOLDEN_SSR and sat not in osr_rows:
                osr_rows[sat] = {
                    "orbit_ecef": (
                        float(osr_match.group("x")),
                        float(osr_match.group("y")),
                        float(osr_match.group("z")),
                    ),
                    "clk_m": float(osr_match.group("clk")),
                    "cbias_n": int(osr_match.group("cbias_n")),
                    "pbias_n": int(osr_match.group("pbias_n")),
                }
        if len(bias_rows) == len(GOLDEN_SSR) and len(osr_rows) == len(GOLDEN_SSR):
            break
    return bias_rows, osr_rows


def _collect_fixed_epoch_debug(
    stderr_text: str,
) -> tuple[dict[tuple[str, int], dict[str, float]], dict[str, dict[str, float]], dict[str, float] | None]:
    phase_rows: dict[tuple[str, int], dict[str, float]] = {}
    if_rows: dict[str, dict[str, float]] = {}
    fix_row: dict[str, float] | None = None
    for raw_line in stderr_text.splitlines():
        line = raw_line.strip()
        phase_match = CLAS_PHASE_ROW_RE.match(line)
        if phase_match is not None:
            key = (phase_match.group("sat"), int(phase_match.group("f")))
            if key in GOLDEN_PHASE_ROWS:
                phase_rows[key] = _parse_kv(phase_match.group("body"))
            continue
        if_match = CLAS_IF_PHASE_RE.match(line)
        if if_match is not None:
            sat = if_match.group("sat")
            if sat in GOLDEN_IF_PHASE:
                if_rows[sat] = _parse_kv(if_match.group("body"))
            continue
        fix_match = CLAS_WLNL_FIX_RE.match(line)
        if fix_match is not None:
            fix_row = _parse_kv(fix_match.group("body"))
    return phase_rows, if_rows, fix_row


def _collect_epoch_osr(stderr_text: str, epoch_index: int) -> tuple[int, dict[str, dict[str, float]]]:
    current_epoch = -1
    rows: dict[str, dict[str, float]] = {}
    for raw_line in stderr_text.splitlines():
        line = raw_line.strip()
        if line.startswith("[PPP-ATMOS] "):
            current_epoch += 1
            continue
        if current_epoch != epoch_index:
            continue
        osr_match = OSR_FULL_RE.match(line)
        if osr_match is None:
            continue
        sat = osr_match.group("sat")
        if sat in GOLDEN_OSR_EPOCH2 and sat not in rows:
            rows[sat] = _parse_kv(osr_match.group("body"))
        if len(rows) == len(GOLDEN_OSR_EPOCH2):
            break
    return current_epoch, rows


def _collect_last_named_row(stderr_text: str, prefix: str) -> dict[str, float] | None:
    last: dict[str, float] | None = None
    for raw_line in stderr_text.splitlines():
        line = raw_line.strip()
        if line.startswith(prefix):
            body = line[len(prefix):].strip()
            last = _parse_kv(body)
    return last


def _collect_last_osr_rows(stderr_text: str) -> dict[str, dict[str, float]]:
    rows: dict[str, dict[str, float]] = {}
    for raw_line in stderr_text.splitlines():
        line = raw_line.strip()
        osr_match = OSR_FULL_RE.match(line)
        if osr_match is None:
            continue
        sat = osr_match.group("sat")
        if sat in GOLDEN_30_LAST_OSR:
            rows[sat] = _parse_kv(osr_match.group("body"))
    return rows


@unittest.skipUnless(_integration_ready(), "local 2019 CLAS parity data or gnss_ppp binary missing")
class ClaslibOsrGoldenIntegrationTest(unittest.TestCase):
    def test_first_epoch_ssr_debug_matches_clas_answer_values(self) -> None:
        run = _run_parity_debug(max_epochs=2)
        self.assertEqual(run.returncode, 0, msg=run.stderr + run.stdout)
        bias_rows, osr_rows = _collect_first_epoch_ssr_debug(run.stderr)
        self.assertEqual(set(bias_rows), set(GOLDEN_SSR), msg=run.stderr)
        self.assertEqual(set(osr_rows), set(GOLDEN_SSR), msg=run.stderr)
        for sat, expected in GOLDEN_SSR.items():
            bias = bias_rows[sat]
            self.assertEqual(bias["tow"], expected["tow"])
            self.assertEqual(bias["pref_net"], expected["pref_net"])
            self.assertEqual(bias["atmos_net"], expected["atmos_net"])
            self.assertEqual(bias["phase_net"], expected["phase_net"])
            self.assertEqual(bias["code_net"], expected["code_net"])

            osr = osr_rows[sat]
            for actual, golden in zip(osr["orbit_ecef"], expected["orbit_ecef"]):
                self.assertAlmostEqual(actual, golden, delta=5e-4)
            self.assertAlmostEqual(osr["clk_m"], expected["clk_m"], delta=5e-4)
            self.assertEqual(osr["cbias_n"], expected["cbias_n"])
            self.assertEqual(osr["pbias_n"], expected["pbias_n"])

    def test_fixed_epoch_phase_and_wlnl_debug_match_golden(self) -> None:
        run = _run_parity_debug(max_epochs=2)
        self.assertEqual(run.returncode, 0, msg=run.stderr + run.stdout)
        phase_rows, if_rows, fix_row = _collect_fixed_epoch_debug(run.stderr)
        self.assertEqual(set(phase_rows), set(GOLDEN_PHASE_ROWS), msg=run.stderr)
        self.assertEqual(set(if_rows), set(GOLDEN_IF_PHASE), msg=run.stderr)
        self.assertIsNotNone(fix_row, msg=run.stderr)

        for key, expected in GOLDEN_PHASE_ROWS.items():
            actual = phase_rows[key]
            for field, golden in expected.items():
                self.assertIn(field, actual, msg=f"missing {key} {field}: {run.stderr}")
                self.assertAlmostEqual(actual[field], golden, delta=1e-3)

        for sat, expected in GOLDEN_IF_PHASE.items():
            actual = if_rows[sat]
            for field, golden in expected.items():
                self.assertIn(field, actual, msg=f"missing {sat} {field}: {run.stderr}")
                self.assertAlmostEqual(actual[field], golden, delta=1e-3)

        assert fix_row is not None
        self.assertEqual(int(fix_row["rows"]), GOLDEN_WLNL_FIX["rows"])
        self.assertEqual(int(fix_row["constraints"]), GOLDEN_WLNL_FIX["constraints"])
        self.assertEqual(int(fix_row["solved"]), GOLDEN_WLNL_FIX["solved"])
        self.assertAlmostEqual(fix_row["pos_shift"], GOLDEN_WLNL_FIX["pos_shift"], delta=1e-3)
        self.assertAlmostEqual(fix_row["phase_rms"], GOLDEN_WLNL_FIX["phase_rms"], delta=1e-3)
        self.assertAlmostEqual(fix_row["dd_mean"], GOLDEN_WLNL_FIX["dd_mean"], delta=1e-3)
        self.assertAlmostEqual(fix_row["dd_max"], GOLDEN_WLNL_FIX["dd_max"], delta=1e-3)

    def test_second_epoch_full_osr_matches_golden(self) -> None:
        run = _run_parity_debug(max_epochs=2)
        self.assertEqual(run.returncode, 0, msg=run.stderr + run.stdout)
        epoch_index, rows = _collect_epoch_osr(run.stderr, epoch_index=7)
        self.assertGreaterEqual(epoch_index, 7, msg=run.stderr)
        self.assertEqual(set(rows), set(GOLDEN_OSR_EPOCH2), msg=run.stderr)
        for sat, expected in GOLDEN_OSR_EPOCH2.items():
            actual = rows[sat]
            for field, golden in expected.items():
                self.assertIn(field, actual, msg=f"missing {sat} {field}: {run.stderr}")
                self.assertAlmostEqual(actual[field], golden, delta=1e-3)

    def test_30_epoch_summary_and_late_fixed_residuals_match_golden(self) -> None:
        run, summary = _run_parity_debug_with_summary(max_epochs=30)
        self.assertEqual(run.returncode, 0, msg=run.stderr + run.stdout)

        for field, golden in GOLDEN_30_SUMMARY.items():
            self.assertIn(field, summary)
            self.assertEqual(summary[field], golden)
        self.assertTrue(summary.get("legacy_strict_parity"))
        self.assertFalse(summary.get("ported_clasnat"))

        last_clas_ppp = _collect_last_named_row(run.stderr, "[CLAS-PPP]")
        self.assertIsNotNone(last_clas_ppp, msg=run.stderr)
        assert last_clas_ppp is not None
        for field, golden in GOLDEN_30_LAST_CLAS_PPP.items():
            self.assertIn(field, last_clas_ppp, msg=run.stderr)
            delta = 1e-6 if field in ("rows", "sats") else 1e-3
            self.assertAlmostEqual(last_clas_ppp[field], golden, delta=delta)

        last_wlnl_fix = _collect_last_named_row(run.stderr, "[CLAS-WLNL-FIX]")
        self.assertIsNotNone(last_wlnl_fix, msg=run.stderr)
        assert last_wlnl_fix is not None
        for field, golden in GOLDEN_30_LAST_WLNL_FIX.items():
            self.assertIn(field, last_wlnl_fix, msg=run.stderr)
            delta = 1e-6 if field in ("rows", "constraints", "solved") else 1e-3
            self.assertAlmostEqual(last_wlnl_fix[field], golden, delta=delta)

        last_osr_rows = _collect_last_osr_rows(run.stderr)
        self.assertEqual(set(last_osr_rows), set(GOLDEN_30_LAST_OSR), msg=run.stderr)
        for sat, expected in GOLDEN_30_LAST_OSR.items():
            actual = last_osr_rows[sat]
            for field, golden in expected.items():
                self.assertIn(field, actual, msg=f"missing late {sat} {field}: {run.stderr}")
                self.assertAlmostEqual(actual[field], golden, delta=1e-3)


if __name__ == "__main__":
    unittest.main()
