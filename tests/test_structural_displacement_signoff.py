"""Tests for the R7 multi-day structural-displacement scorer."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "apps" / "commands"))
sys.path.insert(0, str(ROOT_DIR / "apps" / "commands" / "benchmarks"))

import gnss_structural_displacement_signoff as signoff  # noqa: E402
import gnss_structural_displacement_workflow as workflow  # noqa: E402


TRUTH = (-3957184.9682679, 3310231.00877522, 3737703.81925572)


def fake_day(index: int, dx: float = 0.0) -> dict[str, object]:
    return {
        "date_utc": f"2024-01-0{index}",
        "coordinate_ecef_m": [TRUTH[0] + dx, TRUTH[1], TRUTH[2]],
        "truth": {"frame": "IGS20"},
        "antennas": {"TSK2": "TRM159900.00 NONE", "TSKB": "AOAD/M_T DOME"},
        "station_logs": {"rover_log": {"sha256": "a"}, "base_log": {"sha256": "b"}},
        "epochs": 2880,
        "mode": "full",
        "fix_rate_pct": 100.0,
        "gaps": {"max_gap_s": 30.0},
    }


class StructuralDisplacementSignoffTest(unittest.TestCase):
    def test_development_estimates_noise_before_selecting_witness(self) -> None:
        days = [fake_day(1, -0.001), fake_day(2, 0.0), fake_day(3, 0.001)]
        analysis = signoff.analyse_development(days)  # type: ignore[arg-type]
        self.assertEqual(analysis["empirical_daily_noise_floor"]["population_days"], 3)
        self.assertEqual(analysis["false_alert_count"], 0)
        self.assertTrue(analysis["synthetic_witness"]["detected"])
        profile = analysis["candidate_profile"]
        self.assertEqual(profile["development_dates_utc"], ["2024-01-01", "2024-01-02", "2024-01-03"])
        self.assertGreater(profile["witness_enu_m"][0], profile["horizontal_alert_m"])

    def test_contract_fails_closed_on_frame_antenna_or_continuity(self) -> None:
        days = [fake_day(1), fake_day(2), fake_day(3)]
        days[1]["truth"] = {"frame": "ambiguous"}
        days[2]["epochs"] = 2879
        days[2]["antennas"] = {"TSK2": "changed"}
        failures = signoff.enforce_contract(days)  # type: ignore[arg-type]
        self.assertIn("reference_frame_changed_or_ambiguous", failures)
        self.assertIn("antenna_identity_changed_or_ambiguous", failures)
        self.assertIn("observation_continuity_incomplete", failures)

    def test_holdout_uses_frozen_baseline_and_limits(self) -> None:
        day = fake_day(4)
        profile = {
            "baseline_ecef_m": list(TRUTH),
            "horizontal_alert_m": 0.02,
            "vertical_alert_m": 0.04,
            "minimum_fix_rate_pct": 99.0,
            "maximum_gap_s": 60.0,
            "maximum_false_alerts": 0,
            "witness_enu_m": [0.04, -0.02, 0.08],
        }
        result = signoff.analyse_holdout(day, profile)  # type: ignore[arg-type]
        self.assertTrue(result["gate_passed"])
        self.assertFalse(result["stable_site_alert"])

    def test_load_bundle_rejects_header_truth(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r7_bundle_") as temp_dir:
            root = Path(temp_dir)
            manifest = {
                "status": "passed",
                "acquisition": {
                    "observation_epoch_utc": "2024-01-01T00:00:00+00:00",
                    "dataset_role": "r7_development",
                },
            }
            summary = {"accuracy_reference": {"role": "rinex_header_approximate_non_independent"}}
            (root / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
            (root / "relative_summary.json").write_text(json.dumps(summary), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "header position"):
                signoff.load_bundle(root)

    def test_visual_evidence_hashed_and_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r7_snaps_") as temp_dir:
            root = Path(temp_dir)
            (root / "20240101T090000.jpg").write_bytes(b"day1")
            (root / "2024-01-03_morning.png").write_bytes(b"day3")
            (root / "unrelated.txt").write_bytes(b"ignored")
            days = [fake_day(1), fake_day(2), fake_day(3)]
            evidence = signoff.collect_visual_evidence(root, days)  # type: ignore[arg-type]
            self.assertEqual(evidence["declared_absent"], ["2024-01-02"])
            self.assertEqual(evidence["days"]["2024-01-01"][0]["name"], "20240101T090000.jpg")
            self.assertEqual(len(evidence["days"]["2024-01-03"]), 1)
            self.assertNotEqual(
                evidence["days"]["2024-01-01"][0]["sha256"],
                evidence["days"]["2024-01-03"][0]["sha256"],
            )
            self.assertEqual(
                signoff.enforce_visual_evidence_contract(evidence),
                ["visual_evidence_incomplete"],
            )
            complete = dict(
                evidence,
                declared_absent=[],
                days=dict(evidence["days"], **{
                    "2024-01-02": [{"name": "20240102T090000.jpg", "sha256": "x", "bytes": 4}]
                }),
            )
            self.assertEqual(signoff.enforce_visual_evidence_contract(complete), [])

    def test_visual_evidence_absent_is_disclosed_not_fatal(self) -> None:
        days = [fake_day(1), fake_day(2), fake_day(3)]
        evidence = signoff.collect_visual_evidence(None, days)  # type: ignore[arg-type]
        self.assertFalse(evidence["provided"])
        self.assertEqual(evidence["declared_absent"], ["2024-01-01", "2024-01-02", "2024-01-03"])
        self.assertEqual(signoff.enforce_visual_evidence_contract(evidence), [])

    def test_opened_holdout_is_not_rerun(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r7_holdout_") as temp_dir:
            root = Path(temp_dir)
            output = root / "opened"
            output.mkdir()
            (output / "workflow_manifest.json").write_text("{}\n", encoding="utf-8")
            args = workflow.parse_args(
                [
                    "--phase", "holdout", "--mode", "full", "--output-dir", str(output),
                    "--cache-dir", str(root / "cache"),
                ]
            )
            self.assertEqual(workflow.run(args), 2)

    def test_profile_and_guide_preserve_claim_boundaries(self) -> None:
        profile = json.loads(
            (ROOT_DIR / "configs/benchmarks/structural_displacement_r7_tsukuba.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(profile["release_state"], "closed_passed")
        self.assertEqual(profile["development_dates_utc"], ["2024-01-01", "2024-01-02", "2024-01-03"])
        guide = (ROOT_DIR / "docs/use_cases/structural_displacement_monitoring.md").read_text(
            encoding="utf-8"
        )
        for token in (
            "structural-displacement-workflow", "SOLUTION/EPOCHS", "SOLUTION/ESTIMATE",
            "APPROX POSITION XYZ", "synthetic witness", "usable", "degraded", "unusable",
            "station log", "weather", "closed_passed",
        ):
            self.assertIn(token, guide)
        self.assertIn(
            "use_cases/structural_displacement_monitoring.md",
            (ROOT_DIR / "mkdocs.yml").read_text(encoding="utf-8"),
        )


if __name__ == "__main__":
    unittest.main()
