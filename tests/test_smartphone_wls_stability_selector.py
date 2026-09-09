from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_wls_stability_selector as selector  # noqa: E402


class SmartphoneWlsStabilitySelectorTests(unittest.TestCase):
    def _segment_report(self, stable: bool) -> dict[str, object]:
        return {
            "schema_version": selector.SEGMENT_STABILITY_SCHEMA_VERSION,
            "truth_used": False,
            "decision_policy": {"truth_used": False},
            "population": {
                "segment_count": 1,
                "stable_segment_count": int(stable),
                "unstable_segment_count": int(not stable),
                "fallback_epochs": 0 if stable else 2,
            },
            "segments": [
                {
                    "segment_id": 0,
                    "stable": stable,
                    "start_index": 0,
                    "end_index": 1,
                    "epoch_count": 2,
                    "rejected_epochs": 0 if stable else 2,
                    "reject_fraction": 0.0 if stable else 1.0,
                    "max_consecutive_rejects": 0 if stable else 2,
                    "max_prediction_duration_s": 0.0 if stable else 2.0,
                    "max_normalized_innovation_sigma": 1.0,
                    "decision": "stable_rts" if stable else "unstable_raw_fallback",
                    "reason": "within_fixed_thresholds" if stable else "threshold_exceeded",
                }
            ],
        }

    def test_segment_report_is_all_stable_only_when_every_segment_is_stable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "segment_stability.json"
            path.write_text(json.dumps(self._segment_report(True)), encoding="utf-8")
            self.assertTrue(selector._validate_segment_report(path)["all_segments_stable"])
            path.write_text(json.dumps(self._segment_report(False)), encoding="utf-8")
            self.assertFalse(selector._validate_segment_report(path)["all_segments_stable"])

    def test_position_key_mismatch_and_nonfinite_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            device = root / "device_gnss.csv"
            timestamps = [1668473633000, 1668473634000]
            device.write_text(
                "MessageType,utcTimeMillis\n"
                + "\n".join(f"Raw,{timestamp}" for timestamp in timestamps)
                + "\n",
                encoding="utf-8",
            )
            position = root / "position.pos"
            rows = []
            for timestamp in timestamps:
                week, tow = smoother._device_time_to_week_tow(timestamp, 18)
                rows.append(f"{week} {tow:.6f} 1 2 3 37 -122 10 1 5 1")
            position.write_text("\n".join(rows) + "\n", encoding="ascii")
            valid = selector._validate_position_keys(
                position, device, skip_epochs=0, label="fixture"
            )
            self.assertTrue(valid["exact_key_match"])
            device.write_text(
                "MessageType,utcTimeMillis\nRaw,1668473633000\nRaw,1668473635000\n",
                encoding="utf-8",
            )
            with self.assertRaises(selector.StabilitySelectorError):
                selector._validate_position_keys(position, device, skip_epochs=0, label="fixture")
            nonfinite = root / "nonfinite.pos"
            nonfinite.write_text(
                rows[0].replace("1 2 3", "nan 2 3") + "\n" + rows[1] + "\n",
                encoding="ascii",
            )
            with self.assertRaises(selector.StabilitySelectorError):
                selector._validate_position_keys(
                    nonfinite,
                    root / "device_gnss.csv",
                    skip_epochs=0,
                    label="fixture",
                )

    def test_wls_manifest_ground_truth_input_is_rejected(self) -> None:
        source = (
            ROOT
            / "output"
            / "smartphone-r5"
            / "wls-stability-selector-v1"
            / "routes"
            / "2023-05-16-19-55-us-ca-mtv-xe1__pixel7pro"
        )
        manifest = source / "wls" / "wls_manifest.json"
        position = source / "wls" / "wls.pos"
        device = (
            ROOT
            / "output"
            / "smartphone-r5"
            / "wls-stability-selector-v1"
            / "materialized"
            / "routes"
            / "2023-05-16-19-55-us-ca-mtv-xe1"
            / "pixel7pro"
            / "inputs"
            / "device_gnss.csv"
        )
        original = json.loads(manifest.read_text(encoding="utf-8"))
        original["inputs"]["ground_truth"] = {"path": "forbidden"}
        with tempfile.TemporaryDirectory() as temporary:
            malformed = Path(temporary) / "wls_manifest.json"
            malformed.write_text(json.dumps(original), encoding="utf-8")
            with self.assertRaises(selector.StabilitySelectorError):
                selector._validate_wls_manifest(malformed, position, device)

    def test_real_candidate_publishes_atomic_wls_selection(self) -> None:
        source = (
            ROOT
            / "output"
            / "smartphone-r5"
            / "wls-stability-selector-v1"
            / "routes"
            / "2023-05-16-19-55-us-ca-mtv-xe1__pixel7pro"
        )
        materialized = (
            ROOT
            / "output"
            / "smartphone-r5"
            / "wls-stability-selector-v1"
            / "materialized"
            / "routes"
            / "2023-05-16-19-55-us-ca-mtv-xe1"
            / "pixel7pro"
            / "inputs"
        )
        with tempfile.TemporaryDirectory() as temporary:
            result = selector.select_and_publish(
                source / "native-segment-stability" / "smoothed.pos",
                source / "native-segment-stability" / "segment_stability.json",
                source / "wls" / "wls.pos",
                source / "wls" / "wls_manifest.json",
                materialized / "device_gnss.csv",
                Path(temporary),
                phone="pixel7pro",
                dataset_id="2023-05-16-19-55-us-ca-mtv-xe1/pixel7pro",
                skip_epochs=1,
            )
            self.assertEqual(result["decision"], "wls_raw")
            self.assertTrue((Path(temporary) / "selected.pos").is_file())
            self.assertTrue((Path(temporary) / "submission.csv").is_file())
            loaded = selector.load_selector_manifest(Path(temporary) / "selector_manifest.json")
            self.assertEqual(loaded["decision"], "wls_raw")
            self.assertFalse(loaded["truth_used"])


if __name__ == "__main__":
    unittest.main()
