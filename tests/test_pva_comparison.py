import contextlib
import io
import json
import math
import shutil
from pathlib import Path
import sys
import tempfile
import unittest

from unittest import mock
from test_pva import fixture, write, m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts/analysis"))
import compare_online_pva as comparison


class Fixture(unittest.TestCase):
    """Synthetic control/candidate replay directories; has no tests of its own."""
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.estimate, self.truth = fixture(self.root)

    def case(self, name, mutate=None, candidate="vehicle_nhc_latched_v1", scenario="normal", start_s=60, duration_s=10,
             patch=None, is_candidate=None):
        directory = self.root/name
        (directory/"replay").mkdir(parents=True)
        raw = m.read_rows(self.estimate)
        if mutate: mutate(raw)
        estimate = directory/"replay/pva.csv"
        write(estimate, raw)
        report, errors = m.score(estimate, self.truth)
        if scenario in ("gnss_outage", "imu_gap"):  # as pva-evaluate does; this 12-epoch fixture has no epoch after 60 s
            report["scenario"] = m.scenario_summary(errors, scenario, start_s, duration_s)
        if patch: patch(report)
        comparison.dump(directory/"score.json", report)
        m.write_errors(directory/"errors.csv", errors)
        if is_candidate is None: is_candidate = name.startswith("candidate")
        meta = dict(max_epochs=0, candidate=candidate if is_candidate else "none",
                    scenario=scenario, scenario_start_s=start_s, scenario_duration_s=duration_s, epochs=12, start_week=2200,
                    start_tow=1000, base_ecef=[6378137,0,0], lever_arm_flu_m=[0,0,0], navigation_policy="causal")
        comparison.dump(directory/"manifest.json", dict(state="passed", replay=meta,
            inputs={n: comparison.pin(self.truth) for n in ("rover.obs", "base.obs", "base.nav", "imu.csv", "reference.csv")},
            outputs={n: comparison.pin(directory/n) for n in ("score.json", "errors.csv")}, estimate=comparison.pin(estimate),
            binary=dict(sha256="a"*64)))
        return directory


class ComparisonTest(Fixture):

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

    def decide(self, runs, *extra, build=True):
        """Build control/candidate trees for `runs` and invoke the CLI; returns (exit code, decision.json)."""
        for side in ("control", "candidate") if build else ():
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


    def test_default_gate_set_is_selected_by_omission_and_keeps_the_original_schema(self):
        code, omitted = self.decide(("Odaiba_ublox",), "--runs", "Odaiba_ublox")
        explicit_code, explicit = self.decide(("Odaiba_ublox",), "--runs", "Odaiba_ublox", "--gate-set", "default", build=False)
        self.assertEqual((code, explicit_code), (0, 0))
        for decision in (omitted, explicit):
            self.assertEqual(decision["schema"], "libgnsspp.pva_candidate_decision.v1")
            self.assertNotIn("gate_set", decision)
            self.assertEqual(decision["candidate"], "vehicle_nhc_latched_v1")
            self.assertEqual([r["name"] for r in decision["runs"]], ["Odaiba_ublox", "Odaiba_ublox-gnss_outage", "Odaiba_ublox-imu_gap"])
            self.assertTrue(decision["runs"][0]["gates"][0]["name"].startswith("all.rtk_position_m"))
            self.assertNotIn("hypothesis", decision["runs"][0]["gates"][0])
        for key in ("runs", "failures", "adoption", "improved_rotation"):
            self.assertEqual(omitted[key], explicit[key])

    def test_missing_run_fails_closed(self):
        code, decision = self.decide(("Odaiba_ublox",), "--runs", "Odaiba_ublox", "Shinjuku_ublox")
        self.assertEqual(code, 2)
        self.assertEqual(decision["state"], "failed")
        self.assertEqual(decision["adoption"], "No-Go")


class HoldoutHelpers(Fixture):
    """Synthetic three-scenario trees and mutations shared by the holdout gate-set tests; has no tests of its own."""
    CANDIDATE = "velocity_consistency_v9"
    GATE_SET = "holdout_v2"
    RUN = "HKDeepUrban1_novatel"
    SCENARIOS = (("normal", 60, 10), ("gnss_outage", 60, 10), ("imu_gap", 60, 4))

    def tree(self, run=None, control=None, candidate=None, control_patch=None, candidate_patch=None):
        """Write control/candidate replays of one run. `control`/`candidate` map scenario -> row mutation
        of the native CSV, `*_patch` scenario -> mutation of score.json. Returns the comparator arguments."""
        run = run or self.RUN
        self.generation = getattr(self, "generation", 0)+1
        top = f"t{self.generation}"
        for side, mutates, patches in (("control", control or {}, control_patch or {}), ("candidate", candidate or {}, candidate_patch or {})):
            for scenario, start_s, duration_s in self.SCENARIOS:
                where = f"{top}/{side}/normal/{run}" if scenario == "normal" else f"{top}/{side}/scenarios/{run}-{scenario}"
                self.case(where, mutates.get(scenario), self.CANDIDATE, scenario, start_s, duration_s, patches.get(scenario),
                          is_candidate=side == "candidate")
        self.top = self.root/top
        return ["--baseline-dir", str(self.top/"control/normal"), "--candidate-dir", str(self.top/"candidate/normal"),
                "--baseline-scenario-dir", str(self.top/"control/scenarios"), "--candidate-scenario-dir", str(self.top/"candidate/scenarios"),
                "--gate-set", self.GATE_SET, "--contract", str(self.truth), "--runs", run]

    def decide_v2(self, args, *extra):
        out = self.root/f"decision{len(list(self.root.glob('decision*')))}"
        with mock.patch.object(sys, "argv", ["compare_online_pva.py", *args, *extra, "--output-dir", str(out)]), \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            code = comparison.main()
        return code, json.loads((out/"decision.json").read_text())

    @staticmethod
    def failed(decision):
        return {f["gate"]["name"] for f in decision["failures"]}

    def edit_manifest(self, relative, change):
        path = self.top/relative/"manifest.json"
        manifest = json.loads(path.read_text())
        change(manifest)
        comparison.dump(path, manifest)

    @staticmethod
    def shifted(prefix, dx, velocity=0.0):
        """Constant ECEF-x offset: at lat=lon=0 an x error is a pure Up error of the same size."""
        def mutate(rows):
            for row in rows:
                row[prefix+"_x_m"] = 6378137+dx
                row[prefix+"_vx_mps"] = velocity
        return mutate

    @staticmethod
    def yaw_error(degrees):
        """The last epoch has truth heading 0; give the estimate a heading error of `degrees`."""
        def mutate(rows):
            a = math.radians(90-degrees)/2
            rows[-1].update(qw=math.cos(a), qz=math.sin(a))
        return mutate

    def everywhere(self, *mutates):
        def mutate(rows):
            for f in mutates: f(rows)
        return {name: mutate for name, _, _ in self.SCENARIOS}

class HoldoutV2Test(HoldoutHelpers):
    """Pooled gate set of docs/online_pva_default_switch_holdout_v2.md, on synthetic replays only."""

    def test_equal_arms_pass_every_gate(self):
        code, decision = self.decide_v2(self.tree())
        self.assertEqual(code, 0)
        self.assertEqual((decision["adoption"], decision["failures"], decision["gate_set"]), ("Go", [], "holdout_v2"))
        self.assertEqual(decision["schema"], "libgnsspp.pva_candidate_decision.v2")
        self.assertEqual(decision["candidate"], self.CANDIDATE)
        self.assertEqual(decision["binary_sha256"], "a"*64)
        gates = decision["runs"][0]["gates"]
        counts = {h: sum(g["hypothesis"] == h for g in gates) for h in ("H1", "H2", "H3", "H4", "H5", "H6")}
        self.assertEqual(counts, dict(H1=8, H2=12, H3=2, H4=6, H5=8, H6=3))
        self.assertEqual(len({g["name"] for g in gates}), len(gates))
        self.assertEqual(decision["runs"][0]["pooled_epochs"], dict(control=36, candidate=36))
        self.assertEqual([r["scenario"] for r in decision["runs"][0]["replays"]], ["normal", "gnss_outage", "imu_gap"])

    def test_fixed_definitions(self):
        self.assertEqual(comparison.GATE_SETS[:2], ("default", "holdout_v2"))
        self.assertEqual(comparison.HOLDOUT_V2_CANDIDATE, "velocity_consistency_v9")
        self.assertEqual(comparison.HOLDOUT_V2_CONTRACT, "docs/online_pva_default_switch_holdout_v2.md")
        self.assertEqual(comparison.HOLDOUT_V2_SCENARIOS, (("normal", None, None), ("gnss_outage", 60, 10), ("imu_gap", 60, 4)))
        self.assertEqual((comparison.H1_RATIO, comparison.H2_RATIO, comparison.H3_RATIO), (1.00, 1.10, 1.25))
        self.assertEqual((comparison.H4_COVERAGE_LOSS, comparison.H5_TIMING_SLACK_S, comparison.H6_PROCESSING_RATIO), (0.005, 1.0, 2.0))
        self.assertEqual(comparison.H1_METRICS, ("fused_position_m", "rotation_deg"))
        self.assertEqual(comparison.H2_METRICS, ("rtk_position_m", "rtk_velocity_mps", "fused_velocity_mps"))
        self.assertEqual(comparison.H3_METRICS, ("fused_position_m", "rotation_deg"))
        with mock.patch.object(sys, "argv", ["x", "--gate-set", "nope"]), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit): comparison.main()

    def test_pooled_statistics_match_the_scorer_and_add_p99(self):
        values = [float(v) for v in range(-100, 101)]
        pooled, scored = comparison.pooled_stats(values), comparison.stats(values)
        for key in ("count", "rmse", "p50", "p95", "max"): self.assertEqual(pooled[key], scored[key])
        self.assertEqual(pooled["p99"], 99.0)
        self.assertAlmostEqual(comparison.pooled_stats([0.0, 10.0])["p99"], 9.9)
        empty = comparison.pooled_stats([])
        self.assertEqual((empty["count"], empty["rmse"], empty["p95"], empty["p99"]), (0, None, None, None))

    def test_timing_rule_censors_nulls_and_never_treats_them_as_zero(self):
        t = comparison.timing_passed
        self.assertTrue(t(None, None))   # both never: censored
        self.assertTrue(t(None, 3.0))    # only the candidate gets there
        self.assertFalse(t(0.8, None))   # the candidate never does where the control did
        self.assertFalse(t(0.0, None))   # a control value of zero is a value, not a null
        self.assertTrue(t(0.8, 1.8)); self.assertFalse(t(0.8, 1.9)); self.assertTrue(t(0.0, 1.0)); self.assertFalse(t(0.0, 1.1))

    def test_h1_is_strict_and_h2_allows_ten_percent(self):
        # Fused position 0.5% worse fails only H1; RTK position 5% worse passes H2.
        args = self.tree(control=self.everywhere(self.shifted("fused", 1.0), self.shifted("rtk", 1.0)),
                         candidate=self.everywhere(self.shifted("fused", 1.005), self.shifted("rtk", 1.05)))
        _, decision = self.decide_v2(args)
        self.assertEqual(self.failed(decision), {f"H1.{c}.fused_position_m.{s}" for c in ("all", "common") for s in ("rmse", "p95")})
        self.assertEqual(decision["adoption"], "No-Go")
        # RTK position 12% worse fails every H2 RTK-position gate and nothing else.
        _, decision = self.decide_v2(self.tree(control=self.everywhere(self.shifted("rtk", 1.0)),
                                               candidate=self.everywhere(self.shifted("rtk", 1.12))))
        self.assertEqual(self.failed(decision), {f"H2.{c}.rtk_position_m.{s}" for c in ("all", "common") for s in ("rmse", "p95")})
        # Fused velocity 12% worse.
        _, decision = self.decide_v2(self.tree(control=self.everywhere(self.shifted("fused", 0.0, 1.0)),
                                               candidate=self.everywhere(self.shifted("fused", 0.0, 1.12))))
        self.assertEqual(self.failed(decision), {f"H2.{c}.fused_velocity_mps.{s}" for c in ("all", "common") for s in ("rmse", "p95")})
        # Fused velocity 10% worse is exactly at the limit and passes.
        _, decision = self.decide_v2(self.tree(control=self.everywhere(self.shifted("fused", 0.0, 1.0)),
                                               candidate=self.everywhere(self.shifted("fused", 0.0, 1.1))))
        self.assertEqual(self.failed(decision), set())

    def test_rotation_is_a_primary_h1_metric(self):
        _, decision = self.decide_v2(self.tree(control=self.everywhere(self.yaw_error(90)), candidate=self.everywhere(self.yaw_error(90.5))))
        self.assertEqual(self.failed(decision), {f"H1.{c}.rotation_deg.{s}" for c in ("all", "common") for s in ("rmse", "p95")})
        _, decision = self.decide_v2(self.tree(control=self.everywhere(self.yaw_error(90)), candidate=self.everywhere(self.yaw_error(89))))
        self.assertEqual(self.failed(decision), set())

    def test_pooling_tolerates_one_noisy_scenario_but_not_a_pooled_regression(self):
        # One scenario of three is 9% worse: pooled RTK RMSE and P95 are within 1.10, although a per-scenario 1.01x gate would fail.
        control = self.everywhere(self.shifted("rtk", 1.0))
        candidate = {"normal": self.shifted("rtk", 1.0), "gnss_outage": self.shifted("rtk", 1.0), "imu_gap": self.shifted("rtk", 1.09)}
        _, decision = self.decide_v2(self.tree(control=control, candidate=candidate))
        self.assertEqual(self.failed(decision), set())
        self.assertAlmostEqual(decision["runs"][0]["pooled"]["all"]["candidate"]["rtk_position_m"]["rmse"], math.sqrt((2+1.09**2)/3), places=9)
        self.assertEqual(decision["runs"][0]["pooled"]["all"]["candidate"]["rtk_position_m"]["count"], 36)
        # Two of three 12% worse: the pooled P95 and RMSE exceed 1.10.
        candidate["gnss_outage"] = candidate["imu_gap"] = self.shifted("rtk", 1.12)
        _, decision = self.decide_v2(self.tree(control=control, candidate=candidate))
        self.assertIn("H2.all.rtk_position_m.p95", self.failed(decision))

    def test_h3_tail_gate_is_the_pooled_p99_of_the_all_output_cohort(self):
        # The candidate is better almost everywhere (0.5 m against 1 m) but one epoch is 3 m: RMSE and P95 improve, P99 does not.
        def outlier(rows):
            self.shifted("fused", 0.5)(rows)
            rows[5]["fused_x_m"] = 6378137+3.0
        args = self.tree(control=self.everywhere(self.shifted("fused", 1.0)),
                         candidate={"normal": self.shifted("fused", 0.5), "gnss_outage": outlier, "imu_gap": self.shifted("fused", 0.5)})
        _, decision = self.decide_v2(args)
        self.assertEqual(self.failed(decision), {"H3.all.fused_position_m.p99"})
        gate = [g for g in decision["runs"][0]["gates"] if g["name"] == "H3.all.fused_position_m.p99"][0]
        self.assertAlmostEqual(gate["control"], 1.0)
        self.assertAlmostEqual(gate["candidate"], 0.5+(3.0-0.5)*0.68)  # 33 fused epochs: P99 index 31.68
        self.assertNotIn("H3.common.fused_position_m.p99", {g["name"] for g in decision["runs"][0]["gates"]})

    def test_h4_pooled_coverage_and_the_common_cohort(self):
        # Hiding the RTK epoch with the large error helps the all-output cohort; coverage (H4) catches it and the
        # paired common-valid cohort, which drops that epoch from both arms, shows no gain.
        spike = lambda rows: rows[4].update(rtk_x_m=6378137+50.0)
        hide = lambda rows: rows[4].update(rtk_status="0")
        _, decision = self.decide_v2(self.tree(control={"imu_gap": spike}, candidate={"imu_gap": hide}))
        self.assertEqual(self.failed(decision), {"H4.coverage.rtk_available", "H4.coverage.rtk_velocity_available"})
        run = decision["runs"][0]
        self.assertAlmostEqual(run["pooled_coverage"]["control"]["rtk_available"]-run["pooled_coverage"]["candidate"]["rtk_available"], 1/36)
        self.assertEqual(run["pooled"]["all"]["control"]["rtk_position_m"]["count"], 36)
        self.assertEqual(run["pooled"]["common"]["control"]["rtk_position_m"]["count"], 35)
        self.assertEqual(run["pooled"]["all"]["control"]["rtk_position_m"]["max"], 50.0)
        self.assertEqual(run["pooled"]["common"]["control"]["rtk_position_m"]["max"], 0.0)
        self.assertEqual(run["pooled"]["common"]["candidate"], run["pooled"]["common"]["control"])

    def test_h5_timing_in_the_normal_and_disruption_scenarios(self):
        def generation(**fields):
            return lambda report: next(iter(report["generations"].values())).update(fields)
        def recovery(**fields):
            return lambda report: report["scenario"].update(fields)
        def run(control_normal, candidate_normal, control_outage, candidate_outage):
            return self.decide_v2(self.tree(
                control_patch={"normal": generation(**control_normal), "gnss_outage": recovery(**control_outage)},
                candidate_patch={"normal": generation(**candidate_normal), "gnss_outage": recovery(**candidate_outage)}))[1]
        control_normal = dict(first_fresh_s=2.0, first_heading_s=None)
        control_outage = dict(recovery_gnss_update_s=0.8, recovery_fresh_attitude_s=None, recovery_heading_s=3.0)
        # +1.0 s is allowed; a candidate that gets where the control never did is allowed; both null is allowed.
        decision = run(control_normal, dict(first_fresh_s=3.0, first_heading_s=40.0),
                       control_outage, dict(recovery_gnss_update_s=1.8, recovery_fresh_attitude_s=None, recovery_heading_s=4.0))
        self.assertEqual(self.failed(decision), set())
        # +1.1 s fails, and a null candidate against a non-null control fails.
        decision = run(control_normal, dict(first_fresh_s=3.1, first_heading_s=None),
                       control_outage, dict(recovery_gnss_update_s=0.8, recovery_fresh_attitude_s=None, recovery_heading_s=None))
        self.assertEqual(self.failed(decision), {"normal.initial.first_fresh_s", "gnss_outage.scenario.recovery_heading_s"})
        # A control recovery of 0.0 s is a value: a null candidate fails.
        decision = run(control_normal, control_normal, dict(control_outage, recovery_gnss_update_s=0.0),
                       dict(control_outage, recovery_gnss_update_s=None))
        self.assertEqual(self.failed(decision), {"gnss_outage.scenario.recovery_gnss_update_s"})
        # The imu_gap scenario is gated like the GNSS outage.
        decision = self.decide_v2(self.tree(control_patch={"imu_gap": recovery(recovery_fresh_attitude_s=1.0)},
                                            candidate_patch={"imu_gap": recovery(recovery_fresh_attitude_s=2.5)}))[1]
        self.assertEqual(self.failed(decision), {"imu_gap.scenario.recovery_fresh_attitude_s"})

    def test_h6_processing_p95_per_scenario_replay(self):
        slow = lambda value: (lambda rows: [row.update(processing_ms=value) for row in rows])
        self.assertEqual(self.failed(self.decide_v2(self.tree(candidate={"imu_gap": slow(1.0)}))[1]), set())  # exactly 2 x 0.5 ms
        self.assertEqual(self.failed(self.decide_v2(self.tree(candidate={"imu_gap": slow(1.01)}))[1]), {"imu_gap.processing.p95_ms"})

    def test_h7_missing_scenario_scenario_kind_and_window(self):
        args = self.tree()
        shutil.rmtree(self.top/f"candidate/scenarios/{self.RUN}-imu_gap")
        code, decision = self.decide_v2(args)
        self.assertEqual((code, decision["state"], decision["adoption"]), (2, "failed", "No-Go"))
        args = self.tree()
        for side in ("control", "candidate"):
            self.edit_manifest(f"{side}/scenarios/{self.RUN}-gnss_outage", lambda m_: m_["replay"].update(scenario="imu_gap"))
        code, decision = self.decide_v2(args)
        self.assertEqual((code, decision["state"]), (2, "failed"))
        self.assertIn("expected the gnss_outage scenario", decision["error"])
        args = self.tree()
        for side in ("control", "candidate"):
            self.edit_manifest(f"{side}/scenarios/{self.RUN}-imu_gap", lambda m_: m_["replay"].update(scenario_duration_s=10))
        code, decision = self.decide_v2(args)
        self.assertEqual(code, 2)
        self.assertIn("expected scenario window", decision["error"])

    def test_h7_candidate_control_inputs_and_binary_must_match(self):
        code, decision = self.decide_v2(self.tree(), "--candidate-name", "velocity_consistency_v8")
        self.assertEqual(code, 2); self.assertIn("unexpected candidate", decision["error"])
        args = self.tree()
        self.edit_manifest(f"control/normal/{self.RUN}", lambda m_: m_["replay"].update(candidate=self.CANDIDATE))
        code, decision = self.decide_v2(args)
        self.assertEqual(code, 2); self.assertIn("unexpected control", decision["error"])
        args = self.tree()
        self.edit_manifest(f"candidate/scenarios/{self.RUN}-gnss_outage", lambda m_: m_["inputs"]["rover.obs"].update(sha256="different"))
        code, decision = self.decide_v2(args)
        self.assertEqual(code, 2); self.assertIn("different raw inputs", decision["error"])
        args = self.tree()  # all three scenarios of a run must be on one set of raw inputs
        for side in ("control", "candidate"):
            self.edit_manifest(f"{side}/scenarios/{self.RUN}-imu_gap", lambda m_: m_["inputs"]["imu.csv"].update(sha256="other"))
        code, decision = self.decide_v2(args)
        self.assertEqual(code, 2); self.assertIn("share the raw inputs", decision["error"])
        args = self.tree()
        self.edit_manifest(f"candidate/normal/{self.RUN}", lambda m_: m_["binary"].update(sha256="b"*64))
        code, decision = self.decide_v2(args)
        self.assertEqual(code, 2); self.assertIn("one and the same binary", decision["error"])
        args = self.tree()
        self.edit_manifest(f"candidate/normal/{self.RUN}", lambda m_: m_.pop("binary"))
        self.assertEqual(self.decide_v2(args)[0], 2)

    def test_two_runs_are_gated_separately(self):
        args = self.tree()
        harsh = self.tree(run="HKHarshUrban1_novatel", candidate={"normal": self.shifted("fused", 1.0)})  # control is perfect
        for key in ("--baseline-dir", "--candidate-dir", "--baseline-scenario-dir", "--candidate-scenario-dir"):
            self.assertNotEqual(args[args.index(key)+1], harsh[harsh.index(key)+1])
        # Put both runs under one tree: copy the first run's directories next to the second.
        for side in ("control", "candidate"):
            for sub in ("normal", "scenarios"):
                for item in (self.root/"t1"/side/sub).iterdir():
                    shutil.copytree(item, self.root/"t2"/side/sub/item.name)
        code, decision = self.decide_v2(harsh[:-1]+[self.RUN, "HKHarshUrban1_novatel"])
        self.assertEqual(code, 0)
        self.assertEqual([r["name"] for r in decision["runs"]], [self.RUN, "HKHarshUrban1_novatel"])
        self.assertEqual(sorted({f["run"] for f in decision["failures"]}), ["HKHarshUrban1_novatel"])
        self.assertEqual(decision["adoption"], "No-Go")


class HoldoutV3Test(HoldoutHelpers):
    """holdout_v3 = holdout_v2 + H8 (absolute attitude integrity) for velocity_consistency_v10, on synthetic replays only."""
    CANDIDATE = "velocity_consistency_v10"
    GATE_SET = "holdout_v3"
    H8 = "H8.{}.attitude_integrity.rotation_gt_90deg_fraction"

    @staticmethod
    def repaired(rows):
        """The fixture's last epoch is 180 deg off (heading 180, truth 0); give it the truth heading: no flip."""
        a = math.radians(90)/2
        rows[-1]["qw"], rows[-1]["qz"] = math.cos(a), math.sin(a)

    def clean(self, *scenarios):
        return {name: self.repaired for name, _, _ in self.SCENARIOS if name in (scenarios or [n for n, _, _ in self.SCENARIOS])}

    def test_fixed_definitions(self):
        self.assertEqual(comparison.GATE_SETS, ("default", "holdout_v2", "holdout_v3"))
        self.assertEqual(comparison.HOLDOUT_V3_CANDIDATE, "velocity_consistency_v10")
        self.assertEqual(comparison.HOLDOUT_V3_CONTRACT, "docs/online_pva_default_switch_holdout_v3.md")
        self.assertEqual((comparison.ATTITUDE_INTEGRITY_ROTATION_DEG, comparison.ATTITUDE_INTEGRITY_MAX_FRACTION), (90.0, 0.01))
        # holdout_v2 keeps its own candidate and contract.
        self.assertEqual(comparison.HOLDOUT_V2_CANDIDATE, "velocity_consistency_v9")
        self.assertEqual(comparison.HOLDOUT_V2_CONTRACT, "docs/online_pva_default_switch_holdout_v2.md")

    def test_equal_clean_arms_pass_all_42_gates(self):
        code, decision = self.decide_v2(self.tree(control=self.clean(), candidate=self.clean()))
        self.assertEqual(code, 0)
        self.assertEqual((decision["adoption"], decision["failures"]), ("Go", []))
        self.assertEqual((decision["gate_set"], decision["schema"], decision["candidate"]),
                         ("holdout_v3", "libgnsspp.pva_candidate_decision.v2", "velocity_consistency_v10"))
        self.assertNotIn("attitude_integrity", decision)
        gates = decision["runs"][0]["gates"]
        self.assertEqual(len(gates), 42)
        counts = {h: sum(g["hypothesis"] == h for g in gates) for h in ("H1", "H2", "H3", "H4", "H5", "H6", "H8")}
        self.assertEqual(counts, dict(H1=8, H2=12, H3=2, H4=6, H5=8, H6=3, H8=3))
        self.assertEqual(len({g["name"] for g in gates}), len(gates))
        self.assertEqual([g["name"] for g in gates if g["hypothesis"] == "H8"], [self.H8.format(n) for n, _, _ in self.SCENARIOS])
        self.assertTrue(all(g["control"] == 0.0 and g["candidate"] == 0.0 for g in gates if g["hypothesis"] == "H8"))

    def test_h1_to_h7_are_those_of_holdout_v2(self):
        """The same replays under holdout_v2 and holdout_v3 give the same H1-H6 gates."""
        args = self.tree(control=self.everywhere(self.shifted("fused", 1.0), self.shifted("rtk", 1.0)),
                         candidate=self.everywhere(self.shifted("fused", 1.005), self.shifted("rtk", 1.12)))
        _, v3 = self.decide_v2(args)
        v2_args = list(args)
        v2_args[v2_args.index("holdout_v3")] = "holdout_v2"
        _, v2 = self.decide_v2(v2_args, "--candidate-name", self.CANDIDATE)
        self.assertEqual(v2["gate_set"], "holdout_v2")
        self.assertEqual([g for g in v3["runs"][0]["gates"] if g["hypothesis"] != "H8"], v2["runs"][0]["gates"])
        self.assertEqual(len(v2["runs"][0]["gates"]), 39)
        h8 = {g["name"] for g in v3["runs"][0]["gates"] if g["hypothesis"] == "H8"}
        self.assertEqual(self.failed(v3) - h8, self.failed(v2))
        self.assertIn("H1.all.fused_position_m.rmse", self.failed(v3))
        self.assertIn("H2.all.rtk_position_m.rmse", self.failed(v3))

    def test_h8_fails_a_candidate_flip_even_when_the_control_flips_too(self):
        # The control flips in all three scenarios; the candidate is clean in two and flips in the IMU-gap replay.
        code, decision = self.decide_v2(self.tree(candidate=self.clean("normal", "gnss_outage")))
        self.assertEqual(code, 0)
        self.assertEqual(self.failed(decision), {self.H8.format("imu_gap")})
        self.assertEqual(decision["adoption"], "No-Go")
        gate = next(g for g in decision["runs"][0]["gates"] if g["name"] == self.H8.format("imu_gap"))
        self.assertAlmostEqual(gate["candidate"], 1/10)   # 1 of 10 scored rotation epochs
        self.assertAlmostEqual(gate["control"], 1/10)
        self.assertFalse(gate["passed"])
        self.assertIn("absolute", gate["allowed"])

    def test_h8_is_absolute_and_the_control_fraction_is_information_only(self):
        # A flipping control with a clean candidate passes; a clean control with a flipping candidate fails H8.
        code, decision = self.decide_v2(self.tree(candidate=self.clean()))
        self.assertEqual((code, decision["failures"], decision["adoption"]), (0, [], "Go"))
        h8 = [g for g in decision["runs"][0]["gates"] if g["hypothesis"] == "H8"]
        self.assertEqual([(g["control"], g["candidate"]) for g in h8], [(0.1, 0.0)]*3)
        _, decision = self.decide_v2(self.tree(control=self.clean()))
        self.assertTrue({self.H8.format(n) for n, _, _ in self.SCENARIOS} <= self.failed(decision))
        self.assertEqual(decision["adoption"], "No-Go")

    def test_h8_threshold_and_definition(self):
        # One flipped epoch of 100 scored epochs is exactly 0.01 and passes; strictly above 90 deg; epochs without a rotation do not count.
        rows = [dict(rotation_deg="") for _ in range(7)] + [dict(rotation_deg="30.0") for _ in range(99)] + [dict(rotation_deg="179.9")]
        above, scored, fraction = comparison.rotation_flip_fraction(rows)
        self.assertEqual((above, scored), (1, 100))
        self.assertLessEqual(fraction, comparison.ATTITUDE_INTEGRITY_MAX_FRACTION)
        self.assertEqual(comparison.rotation_flip_fraction([dict(rotation_deg="90.0")]*200)[0], 0)
        self.assertGreater(comparison.rotation_flip_fraction(rows + [dict(rotation_deg="91.0")])[2], comparison.ATTITUDE_INTEGRITY_MAX_FRACTION)

    def test_the_candidate_name_must_be_velocity_consistency_v10(self):
        for recorded, requested in (("velocity_consistency_v9", "velocity_consistency_v9"),   # replays and flag agree on v9
                                    ("velocity_consistency_v10", "velocity_consistency_v9"),  # the flag names the wrong candidate
                                    ("velocity_consistency_v10", "vehicle_nhc_latched_v1"),
                                    ("velocity_consistency_v10", "none")):
            self.CANDIDATE = recorded
            code, decision = self.decide_v2(self.tree(control=self.clean(), candidate=self.clean()), "--candidate-name", requested)
            self.assertEqual((code, decision["state"], decision["adoption"]), (2, "failed", "No-Go"), (recorded, requested))
            self.assertIn("requires the candidate velocity_consistency_v10", decision["error"])
            self.assertEqual(decision["runs"], [])
        # Replays recorded with another candidate are refused under the default (v10) name as well.
        self.CANDIDATE = "velocity_consistency_v9"
        code, decision = self.decide_v2(self.tree(control=self.clean(), candidate=self.clean()))
        self.assertEqual((code, decision["state"]), (2, "failed"))
        self.assertIn("unexpected candidate", decision["error"])
        # The v2 gate set does not accept the v10 replays under its own default (v9).
        self.CANDIDATE = "velocity_consistency_v10"
        args = self.tree(control=self.clean(), candidate=self.clean())
        args[args.index("holdout_v3")] = "holdout_v2"
        code, decision = self.decide_v2(args)
        self.assertEqual(code, 2); self.assertIn("unexpected candidate", decision["error"])

    def test_v3_inherits_the_integrity_rules(self):
        args = self.tree(control=self.clean(), candidate=self.clean())
        shutil.rmtree(self.top/f"candidate/scenarios/{self.RUN}-imu_gap")
        code, decision = self.decide_v2(args)
        self.assertEqual((code, decision["state"], decision["adoption"]), (2, "failed", "No-Go"))
        args = self.tree(control=self.clean(), candidate=self.clean())
        self.edit_manifest(f"control/normal/{self.RUN}", lambda m_: m_["replay"].update(candidate=self.CANDIDATE))
        code, decision = self.decide_v2(args)
        self.assertEqual(code, 2); self.assertIn("unexpected control", decision["error"])
        args = self.tree(control=self.clean(), candidate=self.clean())
        self.edit_manifest(f"candidate/normal/{self.RUN}", lambda m_: m_["binary"].update(sha256="b"*64))
        code, decision = self.decide_v2(args)
        self.assertEqual(code, 2); self.assertIn("one and the same binary", decision["error"])

    def test_attitude_integrity_flag_is_not_accepted_with_holdout_v3(self):
        out = self.root/"unused_output"
        with mock.patch.object(sys, "argv", ["compare_online_pva.py", *self.tree(), "--attitude-integrity", "--output-dir", str(out)]), \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit): comparison.main()
        self.assertFalse(out.exists())

    def test_holdout_v2_has_no_h8(self):
        self.CANDIDATE, self.GATE_SET = "velocity_consistency_v9", "holdout_v2"
        _, decision = self.decide_v2(self.tree())
        self.assertEqual(len(decision["runs"][0]["gates"]), 39)
        self.assertNotIn("H8", {g["hypothesis"] for g in decision["runs"][0]["gates"]})
        self.assertNotIn("attitude_integrity", decision)


if __name__ == "__main__":
    unittest.main()
