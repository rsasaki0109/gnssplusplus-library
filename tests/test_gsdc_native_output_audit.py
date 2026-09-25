import json
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "analysis"))
from audit_gsdc_native_outputs import audit, audit_bounded_imu_tail, digest, load_official_keys


class BoundedImuTailAuditTest(unittest.TestCase):
    def setUp(self):
        self.argv = ["--native-leading-imu-states", "--native-leading-imu-bounded-tail"]
        self.proof = {
            "leading_imu_bounded_tail_requested": True,
            "bounded_tail_imu_coverage_verified": True,
            "leading_imu_states_requested": True, "leading_clock_only_epochs": 2,
            "imu_coverage_first_sample_relative_s": -2.,
            "imu_coverage_last_sample_relative_s": 10.,
            "imu_coverage_required_end_relative_s": 10.03,
            "imu_required_real_end_relative_s": 2.,
            "imu_trailing_measurement_hold_s": .03,
            "long_gap_imu_max_sample_gap_s": .02,
            "long_gap_imu_coverage_verified": False,
        }

    def test_hold_is_reported_without_real_coverage_claim(self):
        r = audit_bounded_imu_tail(self.proof, self.argv)
        self.assertFalse(r["real_samples_bracket_full_interval"])
        self.assertEqual(r["terminal_measurement_hold_s"], .03)

    def test_rejects_forged_or_missing_proofs(self):
        bad = {"long_gap_imu_coverage_verified": True,
               "imu_trailing_measurement_hold_s": 0.,
               "imu_required_real_end_relative_s": 10.01,
               "imu_coverage_first_sample_relative_s": .01,
               "long_gap_imu_max_sample_gap_s": .06,
               "imu_coverage_last_sample_relative_s": float("nan"),
               "bounded_tail_imu_coverage_verified": False}
        for key, value in bad.items():
            with self.subTest(key=key), self.assertRaises(ValueError):
                audit_bounded_imu_tail({**self.proof, key: value}, self.argv)
        for key in self.proof:
            with self.subTest(missing=key), self.assertRaises(ValueError):
                audit_bounded_imu_tail({k: v for k, v in self.proof.items() if k != key}, self.argv)

    def test_rejects_extended_hold_and_incompatible_schedules(self):
        proof = {**self.proof, "imu_coverage_required_end_relative_s": 10.06,
                 "imu_trailing_measurement_hold_s": .06}
        with self.assertRaises(ValueError):
            audit_bounded_imu_tail(proof, self.argv)
        for flag in ["--native-imu-supported-long-gaps",
                     "--native-phase201-source-inclusive-forward-imu-schedule"]:
            with self.assertRaises(ValueError):
                audit_bounded_imu_tail(self.proof, self.argv + [flag])


class NativeOutputAuditTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        raw = self.folder / "raw.csv"
        raw.write_text("MessageType,utcTimeMillis\nRaw,1000\nRaw,2000\nRaw,3000\n")
        (self.folder / "solution.csv").write_text(
            "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            "route/pixel5,2000,37,-122\nroute/pixel5,3000,37,-122\n")
        self.argv = ["native.exe", "--android-gnss", str(raw), "--android-raw-clock-only",
                     "--native-phase171-raw-p-no-doppler-imu-main"]
        self.summary = {
            "status": "imu-combined-factor", "truth_used": False,
            "android_gnss_diagnostics": {"no_device_wls_seed": True},
            "graph": {"converged": True},
            "raw_utc_key_contract": {
                "warmup_epoch_excluded": True, "raw_epoch_keys": 3, "target_epochs": 2,
                "exact_solution_epochs": 2, "interpolated_epochs": 0,
                "edge_hold_epochs": 0, "unresolved_epochs": 0,
                "device_wls_coordinates_used": False,
            },
        }

    def check(self, official=None):
        (self.folder / "summary.json").write_text(json.dumps(self.summary))
        record = {"state": "complete", "returncode": 0, "wall_s": 1,
                  "argv": self.argv, "binary_sha256": "fixture",
                  "outputs": {name: digest(self.folder / name)
                              for name in ("summary.json", "solution.csv")}}
        initialization = self.folder / "summary.json.initialization.json"
        if initialization.exists():
            record["outputs"][initialization.name] = digest(initialization)
        (self.folder / "run.json").write_text(json.dumps(record))
        return audit("route/pixel5", {"folder": str(self.folder), "argv": self.argv}, official)

    def test_full_keys_and_native_contract_are_separate_requirements(self):
        self.assertTrue(self.check()["all_output_epochs_from_native_states"])
        self.summary["android_gnss_diagnostics"]["no_device_wls_seed"] = False
        imported = self.check()
        self.assertEqual(imported["status"], "raw-key-coverage-verified")
        self.assertFalse(imported["all_output_epochs_from_native_states"])

    def test_fallback_or_unconverged_output_is_not_a_native_success(self):
        self.summary["status"] = "fallback"
        self.assertFalse(self.check()["all_output_epochs_from_native_states"])
        self.summary["status"] = "imu-combined-factor"
        self.summary["graph"]["converged"] = False
        self.assertFalse(self.check()["all_output_epochs_from_native_states"])

    def test_official_first_epoch_is_required_even_when_raw_audit_passes(self):
        result = self.check({"route/pixel5": [1000, 2000, 3000]})
        self.assertTrue(result["all_output_epochs_from_native_states"])
        self.assertEqual(result["official_key_coverage"]["missing_keys"], [1000])
        self.assertFalse(result["official_key_coverage"]["all_required_keys_from_native_states"])

    def test_official_subset_can_drop_extra_tail_but_cannot_accept_fallback(self):
        result = self.check({"route/pixel5": [2000]})["official_key_coverage"]
        self.assertTrue(result["all_required_keys_from_native_states"])
        self.assertFalse(result["exact_submission_key_order"])
        self.assertEqual(result["extra_nonofficial_keys"], [3000])
        self.summary["raw_utc_key_contract"]["exact_solution_epochs"] = 1
        self.summary["raw_utc_key_contract"]["edge_hold_epochs"] = 1
        self.assertFalse(self.check({"route/pixel5": [2000]})[
            "official_key_coverage"]["all_required_keys_from_native_states"])

    def test_official_authority_requires_matching_hash_and_ordered_unique_keys(self):
        sample = self.folder / "sample.csv"
        data = b"tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\nroute/pixel5,2000,unused,unused\n"
        sha = hashlib.sha256(data).hexdigest()
        sample.write_bytes(data.replace(b"\n", b"\r\n"))
        self.assertEqual(load_official_keys(sample, sha), {"route/pixel5": [2000]})
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            load_official_keys(sample, "wrong")
        data += b"route/pixel5,2000,unused,unused\n"
        sample.write_bytes(data)
        with self.assertRaisesRegex(ValueError, "strictly ordered"):
            load_official_keys(sample, hashlib.sha256(data).hexdigest())

    def test_long_gap_lane_requires_runtime_sensor_coverage_proof(self):
        self.argv.append("--native-imu-supported-long-gaps")
        self.summary["imu_initialization"] = {}
        with self.assertRaisesRegex(ValueError, "coverage proof"):
            self.check()
        self.summary["imu_initialization"] = {
            "imu_supported_long_gaps_requested": True,
            "long_gap_imu_coverage_verified": True,
            "long_gap_imu_max_sample_gap_s": .02,
        }
        self.assertTrue(self.check()["all_output_epochs_from_native_states"])
        self.summary["imu_initialization"]["long_gap_imu_max_sample_gap_s"] = .1
        with self.assertRaisesRegex(ValueError, "coverage proof"):
            self.check()

    def test_leading_states_cannot_masquerade_as_accepted_spp(self):
        self.argv.extend(["--native-leading-imu-states", "--native-temporal-seed-initialization",
                          "--android-include-first-native-epoch"])
        self.summary["imu_initialization"] = {
            "leading_imu_states_requested": True, "leading_clock_only_epochs": 1,
            "long_gap_imu_coverage_verified": True, "long_gap_imu_max_sample_gap_s": .02}
        self.summary["raw_utc_key_contract"].update(
            warmup_epoch_excluded=False, target_epochs=3, exact_solution_epochs=3)
        p = self.folder / "solution.csv"
        lines = p.read_text().splitlines()
        p.write_text("\n".join([lines[0], "route/pixel5,1000,37,-122", *lines[1:]]) + "\n")
        seed = {"temporal_initial_guess_count": 1, "independent_spp_accepted_epochs": 2,
                "seeds": [{"reason": "same-run-leading-linear-initial-guess-not-SPP",
                           "temporal_initial_guess": True, "raw_p_status": "insufficient-pseudorange",
                           "raw_source_index": 0, "initial_guess_left_source": 1,
                           "initial_guess_right_source": 2}]}
        initialization = self.folder / "summary.json.initialization.json"
        initialization.write_text(json.dumps(seed))
        self.assertEqual(self.check()["leading_clock_only_epochs"], 1)
        seed["seeds"][0]["raw_p_status"] = "accepted"
        initialization.write_text(json.dumps(seed))
        with self.assertRaisesRegex(ValueError, "provenance mismatch"):
            self.check()

    def test_plan_cannot_silently_audit_another_replay(self):
        self.check()
        with self.assertRaisesRegex(ValueError, "arguments differ"):
            audit("route/pixel5", {"folder": str(self.folder), "argv": ["another.exe"]})


if __name__ == "__main__":
    unittest.main()
