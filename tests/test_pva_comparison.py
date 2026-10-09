import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

from unittest import mock
from test_pva import fixture, write, m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts/analysis"))
import compare_online_pva as comparison


class ComparisonTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.estimate, self.truth = fixture(self.root)

    def case(self, name, mutate=None, candidate="vehicle_nhc_latched_v1"):
        directory = self.root/name
        (directory/"replay").mkdir(parents=True)
        raw = m.read_rows(self.estimate)
        if mutate: mutate(raw)
        estimate = directory/"replay/pva.csv"
        write(estimate, raw)
        report, errors = m.score(estimate, self.truth)
        comparison.dump(directory/"score.json", report)
        m.write_errors(directory/"errors.csv", errors)
        meta = dict(max_epochs=0, candidate=candidate if name.startswith("candidate") else "none",
                    scenario="normal", scenario_start_s=60, scenario_duration_s=10, epochs=12, start_week=2200,
                    start_tow=1000, base_ecef=[6378137,0,0], lever_arm_flu_m=[0,0,0], navigation_policy="causal")
        comparison.dump(directory/"manifest.json", dict(state="passed", replay=meta,
            inputs={n: comparison.pin(self.truth) for n in ("rover.obs", "base.obs", "base.nav", "imu.csv", "reference.csv")},
            outputs={n: comparison.pin(directory/n) for n in ("score.json", "errors.csv")}, estimate=comparison.pin(estimate)))
        return directory

    def test_equal_control_has_no_false_regression(self):
        result, improved = comparison.compare(self.case("control"), self.case("candidate_v1"), "synthetic")
        self.assertFalse(improved)
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_candidate_name_is_checked_and_parity_gate_is_v1_only(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="velocity_consistency_v1")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic")
        result, _ = comparison.compare(control, candidate, "synthetic", "velocity_consistency_v1")
        names = {g["name"] for g in result["gates"]}
        self.assertNotIn("before_latch.numeric_parity", names)
        self.assertTrue(all(g["passed"] for g in result["gates"]))
        v1, _ = comparison.compare(control, self.case("candidate_v1"), "synthetic")
        self.assertIn("before_latch.numeric_parity", {g["name"] for g in v1["gates"]})

    def test_velocity_consistency_v2_is_a_distinct_checked_candidate(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="velocity_consistency_v2")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic", "velocity_consistency_v1")
        result, _ = comparison.compare(control, candidate, "synthetic", "velocity_consistency_v2")
        self.assertNotIn("before_latch.numeric_parity", {g["name"] for g in result["gates"]})
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_velocity_consistency_v3_is_a_distinct_checked_candidate(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="velocity_consistency_v3")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic", "velocity_consistency_v2")
        result, _ = comparison.compare(control, candidate, "synthetic", "velocity_consistency_v3")
        self.assertNotIn("before_latch.numeric_parity", {g["name"] for g in result["gates"]})
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_velocity_consistency_v4_is_a_distinct_checked_candidate(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="velocity_consistency_v4")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic", "velocity_consistency_v3")
        result, _ = comparison.compare(control, candidate, "synthetic", "velocity_consistency_v4")
        self.assertNotIn("before_latch.numeric_parity", {g["name"] for g in result["gates"]})
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_velocity_consistency_v5_is_a_distinct_checked_candidate(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="velocity_consistency_v5")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic", "velocity_consistency_v4")
        result, _ = comparison.compare(control, candidate, "synthetic", "velocity_consistency_v5")
        self.assertNotIn("before_latch.numeric_parity", {g["name"] for g in result["gates"]})
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_velocity_consistency_v6_is_a_distinct_checked_candidate(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="velocity_consistency_v6")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic", "velocity_consistency_v5")
        result, _ = comparison.compare(control, candidate, "synthetic", "velocity_consistency_v6")
        self.assertNotIn("before_latch.numeric_parity", {g["name"] for g in result["gates"]})
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_velocity_consistency_v7_is_a_distinct_checked_candidate(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="velocity_consistency_v7")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic", "velocity_consistency_v6")
        result, _ = comparison.compare(control, candidate, "synthetic", "velocity_consistency_v7")
        self.assertNotIn("before_latch.numeric_parity", {g["name"] for g in result["gates"]})
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_velocity_consistency_v9_is_a_distinct_checked_candidate(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="velocity_consistency_v9")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic", "velocity_consistency_v8")
        result, _ = comparison.compare(control, candidate, "synthetic", "velocity_consistency_v9")
        self.assertNotIn("before_latch.numeric_parity", {g["name"] for g in result["gates"]})
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_velocity_consistency_v10_is_a_distinct_checked_candidate(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="velocity_consistency_v10")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic", "velocity_consistency_v9")
        result, _ = comparison.compare(control, candidate, "synthetic", "velocity_consistency_v10")
        self.assertNotIn("before_latch.numeric_parity", {g["name"] for g in result["gates"]})
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_velocity_consistency_v8_is_a_distinct_checked_candidate(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="velocity_consistency_v8")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic", "velocity_consistency_v7")
        result, _ = comparison.compare(control, candidate, "synthetic", "velocity_consistency_v8")
        self.assertNotIn("before_latch.numeric_parity", {g["name"] for g in result["gates"]})
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_rtk_base_extrapolation_v1_is_a_distinct_checked_candidate(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="rtk_base_extrapolation_v1")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic", "velocity_consistency_v6")
        result, _ = comparison.compare(control, candidate, "synthetic", "rtk_base_extrapolation_v1")
        self.assertNotIn("before_latch.numeric_parity", {g["name"] for g in result["gates"]})
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_rtk_online_product_v1_is_a_distinct_checked_candidate(self):
        control = self.case("control")
        candidate = self.case("candidate", candidate="rtk_online_product_v1")
        with self.assertRaisesRegex(ValueError, "unexpected candidate"):
            comparison.compare(control, candidate, "synthetic", "rtk_base_extrapolation_v1")
        result, _ = comparison.compare(control, candidate, "synthetic", "rtk_online_product_v1")
        self.assertNotIn("before_latch.numeric_parity", {g["name"] for g in result["gates"]})
        self.assertTrue(all(g["passed"] for g in result["gates"]))

    def test_dropping_bad_attitude_cannot_pass_coverage_gate(self):
        control = self.case("control")
        candidate = self.case("candidate", lambda rows: rows[-1].update(attitude_available="0"))
        result, improved = comparison.compare(control, candidate, "synthetic")
        self.assertTrue(improved)  # Removing the error makes all-output RMSE look better.
        failures = {g["name"] for g in result["gates"] if not g["passed"]}
        self.assertIn("coverage.attitude_available", failures)
        self.assertIn("coverage.heading_available", failures)
        self.assertEqual(result["common_valid"]["rotation_deg"]["control"]["count"], 9)

    def test_tampered_score_or_mismatched_inputs_fail_before_decision(self):
        control, candidate = self.case("control"), self.case("candidate")
        manifest = json.loads((candidate/"manifest.json").read_text())
        manifest["inputs"]["imu.csv"]["sha256"] = "different"
        comparison.dump(candidate/"manifest.json", manifest)
        with self.assertRaisesRegex(ValueError, "different raw inputs"):
            comparison.compare(control, candidate, "synthetic")
        (candidate/"score.json").write_text("{}")
        with self.assertRaises((ValueError, KeyError)):
            comparison.compare(control, candidate, "synthetic")

    def decide(self, runs, *extra):
        """Build control/candidate trees for `runs` and invoke the CLI; returns (exit code, decision.json)."""
        for side in ("control", "candidate"):
            for name in runs:
                if (self.root/side/"normal"/name).exists(): continue  # trees are reusable across calls
                self.case(f"{side}/normal/{name}")
                for scenario in ("gnss_outage", "imu_gap"):
                    self.case(f"{side}/scenarios/{name}-{scenario}")
        out = self.root/f"decision{len(list(self.root.glob('decision*')))}"
        argv = ["compare_online_pva.py", "--baseline-dir", str(self.root/"control/normal"),
                "--candidate-dir", str(self.root/"candidate/normal"),
                "--baseline-scenario-dir", str(self.root/"control/scenarios"),
                "--candidate-scenario-dir", str(self.root/"candidate/scenarios"), "--output-dir", str(out), *extra]
        with mock.patch.object(sys, "argv", argv), contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            code = comparison.main()
        return code, json.loads((out/"decision.json").read_text())

    # Gate 8 (--attitude-integrity): absolute, candidate-only, opt-in.
    GATE8 = "attitude_integrity.rotation_gt_90deg_fraction"

    @staticmethod
    def repair_final_flip(raw):
        # The fixture's last epoch is 180 deg off (heading 180, truth 0); give it the truth heading.
        a = math.radians(90)/2
        raw[-1]["qw"], raw[-1]["qz"] = math.cos(a), math.sin(a)

    def test_attitude_integrity_gate_is_absent_by_default(self):
        control, candidate = self.case("control"), self.case("candidate_v1")
        default, _ = comparison.compare(control, candidate, "synthetic")
        explicit_off, _ = comparison.compare(control, candidate, "synthetic", attitude_integrity=False)
        self.assertEqual(default, explicit_off)
        self.assertNotIn(self.GATE8, {g["name"] for g in default["gates"]})

    def test_attitude_integrity_gate_fails_an_absolute_flip_even_when_the_control_flips_too(self):
        control, candidate = self.case("control"), self.case("candidate_v1")  # both flip at the last epoch
        result, _ = comparison.compare(control, candidate, "synthetic", attitude_integrity=True)
        gates = {g["name"]: g for g in result["gates"]}
        self.assertFalse(gates[self.GATE8]["passed"])
        self.assertAlmostEqual(gates[self.GATE8]["candidate"], 1/10)
        self.assertAlmostEqual(gates[self.GATE8]["control"], 1/10)
        others = [g for g in result["gates"] if g["name"] != self.GATE8]
        self.assertTrue(all(g["passed"] for g in others))
        self.assertEqual(others, comparison.compare(control, candidate, "synthetic")[0]["gates"])

    def test_attitude_integrity_gate_passes_a_candidate_without_flips_regardless_of_the_control(self):
        control = self.case("control")
        candidate = self.case("candidate_v1", mutate=self.repair_final_flip)
        result, _ = comparison.compare(control, candidate, "synthetic", attitude_integrity=True)
        gate = next(g for g in result["gates"] if g["name"] == self.GATE8)
        self.assertTrue(gate["passed"])
        self.assertEqual(gate["candidate"], 0.0)
        # Absolute, not relative: a clean control and a flipping candidate fail.
        clean_control = self.case("control2", mutate=self.repair_final_flip)
        flipping = self.case("candidate_v1b")
        failed, _ = comparison.compare(clean_control, flipping, "synthetic", attitude_integrity=True)
        self.assertFalse(next(g for g in failed["gates"] if g["name"] == self.GATE8)["passed"])

    def test_attitude_integrity_threshold_is_one_percent_of_scored_epochs(self):
        rows = [dict(rotation_deg="") for _ in range(5)] + [dict(rotation_deg="1.0") for _ in range(99)]
        self.assertEqual(comparison.rotation_flip_fraction(rows), (0, 99, 0.0))
        at_limit = rows + [dict(rotation_deg="90.5")]  # 1 of 100 scored epochs
        above, scored, fraction = comparison.rotation_flip_fraction(at_limit)
        self.assertEqual((above, scored), (1, 100))
        self.assertLessEqual(fraction, comparison.ATTITUDE_INTEGRITY_MAX_FRACTION)
        self.assertEqual(comparison.rotation_flip_fraction([dict(rotation_deg="90.0")] * 3)[0], 0)  # not strictly above
        self.assertEqual(comparison.rotation_flip_fraction([])[2], 0.0)

    def test_attitude_integrity_flag_adds_the_gate_to_every_run_and_changes_nothing_else(self):
        runs = ("Odaiba_ublox", "Shinjuku_ublox")
        code0, plain = self.decide(runs, "--runs", *runs)
        code1, strict = self.decide(runs, "--runs", *runs, "--attitude-integrity")
        self.assertEqual((code0, code1), (0, 0))
        self.assertNotIn("attitude_integrity", plain)
        self.assertTrue(strict["attitude_integrity"])
        for before, after in zip(plain["runs"], strict["runs"]):
            self.assertEqual(after["gates"][:-1], before["gates"])
            self.assertEqual(after["gates"][-1]["name"], self.GATE8)
        self.assertEqual(plain["adoption"], "No-Go" if plain["failures"] else "Go")
        gate8_failures = [f for f in strict["failures"] if f["gate"]["name"] == self.GATE8]
        self.assertEqual(len(gate8_failures), 6)  # the fixture flips in all 6 runs/scenarios
        self.assertEqual(strict["adoption"], "No-Go")

    def test_default_runs_are_the_six_ppc_runs_in_the_original_order(self):
        self.assertEqual(comparison.PPC_RUNS, ("tokyo1", "tokyo2", "tokyo3", "nagoya1", "nagoya2", "nagoya3"))
        code, decision = self.decide(comparison.PPC_RUNS)
        self.assertEqual(code, 0)
        expected = [n for name in comparison.PPC_RUNS for n in (name, name+"-gnss_outage", name+"-imu_gap")]
        self.assertEqual([r["name"] for r in decision["runs"]], expected)

    def test_runs_argument_selects_other_run_names_without_changing_gates(self):
        code, decision = self.decide(("Odaiba_ublox", "Shinjuku_ublox"), "--runs", "Odaiba_ublox", "Shinjuku_ublox")
        self.assertEqual(code, 0)
        self.assertEqual([r["name"] for r in decision["runs"]],
                         ["Odaiba_ublox", "Odaiba_ublox-gnss_outage", "Odaiba_ublox-imu_gap",
                          "Shinjuku_ublox", "Shinjuku_ublox-gnss_outage", "Shinjuku_ublox-imu_gap"])
        reference = comparison.compare(self.root/"control/normal/Odaiba_ublox", self.root/"candidate/normal/Odaiba_ublox",
                                       "Odaiba_ublox")[0]
        self.assertEqual([g["name"] for g in decision["runs"][0]["gates"]], [g["name"] for g in reference["gates"]])
        self.assertEqual(decision["runs"][0]["gates"], reference["gates"])

    def test_missing_run_fails_closed(self):
        code, decision = self.decide(("Odaiba_ublox",), "--runs", "Odaiba_ublox", "Shinjuku_ublox")
        self.assertEqual(code, 2)
        self.assertEqual(decision["state"], "failed")
        self.assertEqual(decision["adoption"], "No-Go")


if __name__ == "__main__":
    unittest.main()
