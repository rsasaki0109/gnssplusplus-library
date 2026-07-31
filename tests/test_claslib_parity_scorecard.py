#!/usr/bin/env python3
"""Tests for the canonical CLASLIB parity scorecard."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "claslib_parity_scorecard.py"


def write_pos(path: Path, rows: list[tuple[float, float, int]]) -> None:
    lines = ["% test solution"]
    for tow, y_m, status in rows:
        lines.append(
            f"2068 {tow:.3f} 1000.000 {y_m:.6f} 3000.000 "
            f"0.0 0.0 0.0 {status} 10 1.0 0.0 0 1"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_zd_diff(path: Path, *, cpc_rms: float = 0.004) -> None:
    payload = {
        "schema": "clas_zd_component_diff.v1",
        "common_rows": 4,
        "base_only_rows": 0,
        "candidate_only_rows": 0,
        "per_component": [
            {"component": "prc_m", "rows": 4, "rms_delta_m": 0.003},
            {"component": "cpc_m", "rows": 4, "rms_delta_m": cpc_rms},
        ],
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def write_zd_shard(path: Path, component: str, rms: float, *, missing: int = 0) -> None:
    payload = {
        "schema": "clas_zd_component_diff.v1",
        "common_rows": 4,
        "base_only_rows": missing,
        "candidate_only_rows": 0,
        "per_component": [
            {"component": component, "rows": 4, "rms_delta_m": rms},
        ],
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def write_manifest(root: Path, *, candidate_commit: str = "native-abc") -> Path:
    manifest = {
        "schema": "claslib_parity_manifest.v1",
        "dataset": {
            "id": "synthetic-clas",
            "start": "2019-08-27T16:00:00 GPST",
            "end": "2019-08-27T16:00:03 GPST",
            "expected_epochs": 4,
            "warmup_seconds": 1.0,
            "artifacts": {
                "observation": "observation.rnx",
                "navigation": "navigation.nav",
                "l6": "correction.l6",
            },
        },
        "oracle": {
            "implementation": "CLASLIB",
            "version": "test",
            "commit": "claslib-abc",
            "solution_pos": "oracle.pos",
        },
        "candidate": {
            "implementation": "LibGNSS++",
            "commit": candidate_commit,
            "solution_pos": "native.pos",
        },
        "reference_ecef_m": [1000.0, 2000.0, 3000.0],
        "solution": {
            "match_tolerance_s": 0.01,
            "oracle_fix_statuses": [1],
            "candidate_fix_statuses": [4],
        },
        "layers": {"zd_diff_json": "zd.json"},
    }
    path = root / "manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


class ClaslibParityScorecardTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory(prefix="claslib_scorecard_test_")
        self.root = Path(self.temp.name)
        (self.root / "observation.rnx").write_text("synthetic\n", encoding="utf-8")
        (self.root / "navigation.nav").write_text("synthetic\n", encoding="utf-8")
        (self.root / "correction.l6").write_text("synthetic\n", encoding="utf-8")
        write_pos(
            self.root / "oracle.pos",
            [(230400.0, 2000.000, 2), (230401.0, 2000.000, 2),
             (230402.0, 2000.000, 1), (230403.0, 2000.000, 1)],
        )
        write_pos(
            self.root / "native.pos",
            [(230400.0, 2000.002, 5), (230401.0, 2000.002, 5),
             (230402.0, 2000.002, 4), (230403.0, 2000.002, 4)],
        )
        write_zd_diff(self.root / "zd.json")

    def tearDown(self) -> None:
        self.temp.cleanup()

    def run_scorecard(self, manifest: Path, *extra: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(SCRIPT_PATH),
                str(manifest),
                "--json-out",
                str(self.root / "scorecard.json"),
                *extra,
            ],
            check=False,
            capture_output=True,
            text=True,
        )

    def test_passes_and_records_reproducible_contract(self) -> None:
        result = self.run_scorecard(write_manifest(self.root), "--fail-on-gate")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        scorecard = json.loads((self.root / "scorecard.json").read_text(encoding="utf-8"))
        self.assertEqual(scorecard["schema"], "claslib_parity_scorecard.v1")
        self.assertEqual(scorecard["status"], "passed")
        self.assertEqual(scorecard["contract"]["dataset_id"], "synthetic-clas")
        self.assertEqual(scorecard["contract"]["oracle"]["commit"], "claslib-abc")
        self.assertEqual(len(scorecard["contract"]["oracle"]["solution"]["sha256"]), 64)
        solution = scorecard["layers"]["solution"]
        self.assertEqual(solution["evaluated_epochs"], 3)
        self.assertEqual(solution["status_agreement_ratio"], 1.0)
        self.assertEqual(solution["native_only_fix_epochs"], 0)
        self.assertEqual(solution["delta"]["full"]["epochs"], 4)
        self.assertEqual(solution["delta"]["post_warmup"]["epochs"], 3)
        self.assertAlmostEqual(solution["delta"]["all"]["rms_3d_m"], 0.002, places=6)
        self.assertEqual(scorecard["layers"]["zd"]["row_key_coverage"], 1.0)

    def test_fail_on_gate_reports_missing_cpc(self) -> None:
        payload = json.loads((self.root / "zd.json").read_text(encoding="utf-8"))
        payload["per_component"] = payload["per_component"][:1]
        (self.root / "zd.json").write_text(json.dumps(payload), encoding="utf-8")

        result = self.run_scorecard(write_manifest(self.root), "--fail-on-gate")

        self.assertEqual(result.returncode, 1, msg=result.stderr)
        scorecard = json.loads((self.root / "scorecard.json").read_text(encoding="utf-8"))
        self.assertEqual(scorecard["status"], "failed")
        self.assertIn("zd_cpc_rms_delta_m", scorecard["failed_gates"])
        gate = next(item for item in scorecard["gates"] if item["name"] == "zd_cpc_rms_delta_m")
        self.assertIsNone(gate["actual"])
        self.assertIn("absent", gate["reason"])

    def test_rejects_unpinned_oracle_commit(self) -> None:
        manifest_path = write_manifest(self.root)
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        payload["oracle"]["commit"] = ""
        manifest_path.write_text(json.dumps(payload), encoding="utf-8")

        result = self.run_scorecard(manifest_path)

        self.assertEqual(result.returncode, 2)
        self.assertIn("oracle.commit", result.stderr)

    def test_constellation_zd_shards_are_gated_independently(self) -> None:
        for system in ("gps", "qzss", "galileo"):
            write_zd_shard(self.root / f"{system}_code.json", "prc_m", 0.003)
            write_zd_shard(self.root / f"{system}_phase.json", "cpc_m", 0.004)
        manifest_path = write_manifest(self.root)
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["layers"] = {
            "zd_diff_jsons": {
                system: {
                    "code": f"{system}_code.json",
                    "phase": f"{system}_phase.json",
                }
                for system in ("gps", "qzss", "galileo")
            }
        }
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

        result = self.run_scorecard(manifest_path, "--fail-on-gate")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        scorecard = json.loads((self.root / "scorecard.json").read_text(encoding="utf-8"))
        self.assertEqual(set(scorecard["layers"]["zd"]["systems"]), {"gps", "qzss", "galileo"})
        gate_names = {gate["name"] for gate in scorecard["gates"]}
        self.assertIn("zd_gps_prc_rms_delta_m", gate_names)
        self.assertIn("zd_qzss_cpc_rms_delta_m", gate_names)
        self.assertIn("zd_galileo_row_key_coverage", gate_names)

    def test_constellation_row_gap_cannot_be_hidden_by_other_systems(self) -> None:
        write_zd_shard(self.root / "gps_code.json", "prc_m", 0.003)
        write_zd_shard(self.root / "gps_phase.json", "cpc_m", 0.004, missing=1)
        manifest_path = write_manifest(self.root)
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["layers"] = {
            "zd_diff_jsons": {
                "gps": {"code": "gps_code.json", "phase": "gps_phase.json"},
                "qzss": "zd.json",
            }
        }
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

        result = self.run_scorecard(manifest_path, "--fail-on-gate")

        self.assertEqual(result.returncode, 1, msg=result.stderr)
        scorecard = json.loads((self.root / "scorecard.json").read_text(encoding="utf-8"))
        self.assertIn("zd_gps_row_key_coverage", scorecard["failed_gates"])
        self.assertNotIn("zd_qzss_row_key_coverage", scorecard["failed_gates"])

    def test_all_fixed_post_warmup_makes_float_gate_not_applicable(self) -> None:
        write_pos(
            self.root / "oracle.pos",
            [(230400.0, 2000.0, 2), (230401.0, 2000.0, 1),
             (230402.0, 2000.0, 1), (230403.0, 2000.0, 1)],
        )
        write_pos(
            self.root / "native.pos",
            [(230400.0, 2000.0, 5), (230401.0, 2000.0, 4),
             (230402.0, 2000.0, 4), (230403.0, 2000.0, 4)],
        )

        result = self.run_scorecard(write_manifest(self.root), "--fail-on-gate")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        scorecard = json.loads((self.root / "scorecard.json").read_text(encoding="utf-8"))
        gate = next(item for item in scorecard["gates"] if item["name"] == "float_delta_rms_3d_m")
        self.assertEqual(gate["status"], "passed")
        self.assertIn("not applicable", gate["reason"])


if __name__ == "__main__":
    unittest.main()
