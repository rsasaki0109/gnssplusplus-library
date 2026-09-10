from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_holdout_eval as HOLDOUT  # noqa: E402


class NativeFgoHoldoutEvaluationTests(unittest.TestCase):
    def _freeze(self) -> dict[str, object]:
        return {
            "schema_version": HOLDOUT.FREEZE_SCHEMA,
            "status": "frozen-before-holdout-payload-access",
            "candidate_id": "native_fgo_pseudorange_tdcp_motion_v1",
            "holdout": {
                "dataset_id": HOLDOUT.HOLDOUT_ID,
                "route": HOLDOUT.HOLDOUT_ROUTE,
                "phone": HOLDOUT.HOLDOUT_PHONE,
                "phone_allowlist": [HOLDOUT.HOLDOUT_PHONE],
                "materialization_forbidden_before_freeze": True,
                "truth_open_forbidden_before_freeze": True,
            },
            "holdout_execution_contract": {
                "authorized": True,
                "truth_free_phase": True,
                "one_shot_authorization": True,
                "no_post_holdout_tuning": True,
                "route": HOLDOUT.HOLDOUT_ROUTE,
                "phone_allowlist": [HOLDOUT.HOLDOUT_PHONE],
                "truth_evaluation_pass_count": 1,
                "algorithm_parameter_hash": "recipe-hash",
            },
            "policy": {
                "truth_free_before_truth": True,
                "future_holdout_payload_materialized_before_freeze": False,
                "future_holdout_truth_open_count_before_freeze": 0,
                "leaderboard_scores_used_for_tuning": False,
                "external_mutation": False,
                "token_access": False,
                "no_post_holdout_tuning": True,
            },
        }

    def test_valid_one_shot_contract_is_accepted(self) -> None:
        HOLDOUT._verify_holdout_contract(self._freeze())

    def test_route_and_phone_allowlist_are_fail_closed(self) -> None:
        payload = self._freeze()
        payload["holdout"]["phone_allowlist"] = ["pixel5"]  # type: ignore[index]
        with self.assertRaises(HOLDOUT.HoldoutFgoError):
            HOLDOUT._verify_holdout_contract(payload)

    def test_one_shot_and_no_tuning_flags_are_required(self) -> None:
        payload = self._freeze()
        payload["holdout_execution_contract"]["one_shot_authorization"] = False  # type: ignore[index]
        with self.assertRaises(HOLDOUT.HoldoutFgoError):
            HOLDOUT._verify_holdout_contract(payload)
        payload = self._freeze()
        payload["policy"]["no_post_holdout_tuning"] = False  # type: ignore[index]
        with self.assertRaises(HOLDOUT.HoldoutFgoError):
            HOLDOUT._verify_holdout_contract(payload)

    def test_recipe_hash_is_canonical(self) -> None:
        first = {"z": 1, "a": [True, 2.0]}
        second = {"a": [True, 2.0], "z": 1}
        self.assertEqual(
            HOLDOUT.algorithm_parameter_hash(first),
            HOLDOUT.algorithm_parameter_hash(second),
        )

    def test_truth_free_manifest_rejects_truth_presence_or_open(self) -> None:
        with tempfile.TemporaryDirectory(prefix="native_fgo_holdout_test_") as raw:
            output = Path(raw)
            artifact = output / "artifact.bin"
            artifact.write_bytes(b"finite artifact\n")
            manifest = {
                "schema_version": HOLDOUT.TRUTH_FREE_SCHEMA,
                "status": "truth-free-artifacts-sealed",
                "dataset_id": HOLDOUT.HOLDOUT_ID,
                "truth_opened": False,
                "truth_materialized": False,
                "truth_open_count": 0,
                "freeze_record_sha256": "freeze",
                "freeze_manifest_sha256": "manifest",
                "artifacts": {
                    "artifact.bin": {
                        "path": "artifact.bin",
                        "bytes": artifact.stat().st_size,
                        "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
                    }
                },
            }
            (output / "truth_free_manifest.json").write_text(
                json.dumps(manifest), encoding="utf-8"
            )
            self.assertRaises(
                HOLDOUT.HoldoutFgoError,
                HOLDOUT._verify_truth_free_manifest,
                output,
                "wrong-freeze",
                "manifest",
            )
            (output / "truth_free_manifest.json").write_text(
                json.dumps({**manifest, "freeze_record_sha256": "freeze"}),
                encoding="utf-8",
            )
            (output / "inputs").mkdir()
            (output / "inputs" / "ground_truth.csv").write_text(
                "must not be here\n", encoding="ascii"
            )
            with self.assertRaises(HOLDOUT.HoldoutFgoError):
                HOLDOUT._verify_truth_free_manifest(output, "freeze", "manifest")


if __name__ == "__main__":
    unittest.main()
