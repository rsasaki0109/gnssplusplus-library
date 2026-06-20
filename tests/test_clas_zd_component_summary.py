#!/usr/bin/env python3
"""Tests for CLAS zero-difference component snapshot summaries."""

from __future__ import annotations

import csv
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "clas_zd_component_summary.py"

spec = importlib.util.spec_from_file_location("clas_zd_component_summary", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
summary_script = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = summary_script
spec.loader.exec_module(summary_script)


FIELDNAMES = [
    "week",
    "tow",
    "stage",
    "sat",
    "row_type",
    "freq",
    "pseudorange_rinex_code",
    "carrier_rinex_code",
    "bias_exact_identity",
    "observation_exact_identity_requested",
    "observation_exact_match",
    "observation_family_fallback",
    "code_bias_fallback",
    "phase_bias_fallback",
    "prc_m",
    "cpc_m",
    "code_bias_m",
    "phase_bias_m",
]


def zd_row(
    *,
    sat: str = "G14",
    tow: str = "172800.000",
    stage: str = "accepted",
    row_type: str = "code",
    freq: str = "1",
    pseudorange_code: str = "C2W",
    carrier_code: str = "",
    bias_exact_identity: str = "1",
    observation_exact_identity_requested: str = "1",
    observation_exact_match: str = "1",
    observation_family_fallback: str = "0",
    code_bias_fallback: str = "0",
    phase_bias_fallback: str = "0",
    prc_m: str = "0.100",
    cpc_m: str = "",
    code_bias_m: str = "0.020",
    phase_bias_m: str = "",
) -> dict[str, str]:
    return {
        "week": "2360",
        "tow": tow,
        "stage": stage,
        "sat": sat,
        "row_type": row_type,
        "freq": freq,
        "pseudorange_rinex_code": pseudorange_code,
        "carrier_rinex_code": carrier_code,
        "bias_exact_identity": bias_exact_identity,
        "observation_exact_identity_requested": observation_exact_identity_requested,
        "observation_exact_match": observation_exact_match,
        "observation_family_fallback": observation_family_fallback,
        "code_bias_fallback": code_bias_fallback,
        "phase_bias_fallback": phase_bias_fallback,
        "prc_m": prc_m,
        "cpc_m": cpc_m,
        "code_bias_m": code_bias_m,
        "phase_bias_m": phase_bias_m,
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


class ClasZdComponentSummaryTest(unittest.TestCase):
    def test_summary_reports_identity_and_components(self) -> None:
        rows = [
            zd_row(sat="G14", row_type="code", pseudorange_code="C2W"),
            zd_row(
                sat="J01",
                row_type="phase",
                tow="172830.000",
                freq="2",
                pseudorange_code="",
                carrier_code="L2L",
                prc_m="",
                cpc_m="0.200",
                code_bias_m="",
                phase_bias_m="-0.050",
            ),
        ]

        summary = summary_script.summarize_rows(rows)

        self.assertEqual(summary["schema"], "clas_zd_component_summary.v2")
        self.assertEqual(summary["status"], "passed")
        self.assertEqual(summary["rows"], 2)
        self.assertEqual(summary["systems"], {"GPS": 1, "QZSS": 1})
        self.assertEqual(summary["row_types"], {"code": 1, "phase": 1})
        self.assertEqual(summary["stages"], {"accepted": 2})
        self.assertEqual(summary["time_range"]["start_tow"], 172800.0)
        self.assertEqual(summary["time_range"]["end_tow"], 172830.0)
        observation_identity = summary["observation_identity"]
        self.assertEqual(observation_identity["rinex_codes"]["C2W"], 1)
        self.assertEqual(observation_identity["rinex_codes"]["L2L"], 1)
        self.assertEqual(observation_identity["frequency_indices"]["2"], 1)
        self.assertEqual(summary["component_presence"]["rows_with_component"]["prc_m"], 1)
        self.assertEqual(summary["component_presence"]["rows_with_component"]["phase_bias_m"], 1)
        self.assertEqual(summary["bias_identity"]["rows_with_code_bias"], 1)
        self.assertEqual(summary["bias_identity"]["rows_with_phase_bias"], 1)
        self.assertEqual(summary["identity_provenance"]["gps_l2w_rows"], 1)
        self.assertEqual(
            summary["identity_provenance"]["components"]["observation_exact_match"]["gps_l2w_true_rows"],
            1,
        )
        self.assertEqual(summary["row_key"]["duplicate_groups"], 0)

    def test_summary_detects_bad_rows_and_duplicate_keys(self) -> None:
        rows = [
            zd_row(prc_m="", code_bias_m=""),
            zd_row(prc_m="", code_bias_m=""),
            zd_row(
                sat="",
                row_type="",
                tow="bad",
                pseudorange_code="",
                bias_exact_identity="",
                observation_exact_identity_requested="",
                observation_exact_match="",
                observation_family_fallback="",
                code_bias_fallback="",
                phase_bias_fallback="",
                prc_m="nan",
                code_bias_m="",
            ),
        ]

        summary = summary_script.summarize_rows(rows)

        self.assertEqual(summary["status"], "failed")
        self.assertEqual(summary["row_key"]["duplicate_groups"], 1)
        self.assertEqual(summary["issue_counts"]["missing_satellite"], 1)
        self.assertEqual(summary["issue_counts"]["missing_row_type"], 1)
        self.assertEqual(summary["issue_counts"]["missing_observation_identity"], 1)
        self.assertEqual(summary["issue_counts"]["bad_row_key"], 1)
        self.assertEqual(summary["issue_counts"]["missing_numeric_component"], 1)

    def test_cli_writes_json_and_enforces_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="clas_zd_component_summary_cli_") as temp_dir:
            root = Path(temp_dir)
            snapshot = root / "clas_zd.csv"
            summary_json = root / "summary.json"
            write_csv(snapshot, [zd_row(row_type="code")])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(snapshot),
                    "--json-out",
                    str(summary_json),
                    "--require-rows-min",
                    "2",
                    "--require-system",
                    "QZSS",
                    "--require-stage",
                    "candidate",
                    "--require-row-type",
                    "phase",
                    "--require-rinex-code",
                    "L2W",
                    "--require-component",
                    "prc_m",
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 1)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema"], "clas_zd_component_summary.v2")
            self.assertEqual(payload["status"], "failed")
            self.assertIn("rows 1 < required minimum 2", payload["failures"])
            self.assertIn("required system QZSS is absent", payload["failures"])
            self.assertIn("required row type phase is absent", payload["failures"])

    def test_cli_passes_clean_snapshot_with_fail_on_issue(self) -> None:
        with tempfile.TemporaryDirectory(prefix="clas_zd_component_summary_pass_") as temp_dir:
            root = Path(temp_dir)
            snapshot = root / "clas_zd.csv"
            summary_json = root / "summary.json"
            write_csv(snapshot, [zd_row(row_type="code")])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(snapshot),
                    "--json-out",
                    str(summary_json),
                    "--fail-on-issue",
                    "--require-row-type",
                    "code",
                    "--require-rinex-code",
                    "C2W",
                    "--require-component",
                    "prc_m",
                    "--require-gps-l2w-rows-min",
                    "1",
                    "--require-gps-l2w-exact-bias",
                    "--require-gps-l2w-observation-exact-match",
                    "--require-gps-l2w-no-observation-family-fallback",
                    "--require-gps-l2w-no-code-bias-fallback",
                    "--require-gps-l2w-no-phase-bias-fallback",
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "passed")
            self.assertEqual(payload["failures"], [])

    def test_cli_rejects_gps_l2w_identity_contract_violations(self) -> None:
        with tempfile.TemporaryDirectory(prefix="clas_zd_component_summary_identity_") as temp_dir:
            root = Path(temp_dir)
            snapshot = root / "clas_zd.csv"
            summary_json = root / "summary.json"
            write_csv(
                snapshot,
                [
                    zd_row(
                        row_type="code",
                        bias_exact_identity="0",
                        observation_exact_match="0",
                        observation_family_fallback="1",
                        code_bias_fallback="1",
                        phase_bias_fallback="1",
                    )
                ],
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(snapshot),
                    "--json-out",
                    str(summary_json),
                    "--require-gps-l2w-rows-min",
                    "2",
                    "--require-gps-l2w-exact-bias",
                    "--require-gps-l2w-observation-exact-match",
                    "--require-gps-l2w-no-observation-family-fallback",
                    "--require-gps-l2w-no-code-bias-fallback",
                    "--require-gps-l2w-no-phase-bias-fallback",
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 1)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "failed")
            self.assertIn("GPS L2W rows 1 < required minimum 2", payload["failures"])
            self.assertIn("GPS L2W exact-bias rows 0 != GPS L2W rows 1", payload["failures"])
            self.assertIn(
                "GPS L2W observation exact-match rows 0 != GPS L2W rows 1",
                payload["failures"],
            )
            self.assertIn("GPS L2W observation-family fallback rows 1 != 0", payload["failures"])
            self.assertIn("GPS L2W code-bias fallback rows 1 != 0", payload["failures"])
            self.assertIn("GPS L2W phase-bias fallback rows 1 != 0", payload["failures"])


if __name__ == "__main__":
    unittest.main()
