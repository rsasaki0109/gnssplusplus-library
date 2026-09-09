from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_optimizer_stop_eval as OPT  # noqa: E402


class NativeFgoOptimizerStopTests(unittest.TestCase):
    def test_only_stopping_policy_differs_and_commands_are_deterministic(self) -> None:
        self.assertEqual(
            OPT.STOP_POLICIES,
            (("baseline8", 8, 0.0, 0.0), ("candidate50", 50, 1e-6, 0.0)),
        )
        first = OPT.fgo_command(
            Path("obs"), Path("nav"), Path("seed"), Path("out"), 8, 0.0, 0.0
        )
        second = OPT.fgo_command(
            Path("obs"), Path("nav"), Path("seed"), Path("out"), 8, 0.0, 0.0
        )
        self.assertEqual(first, second)
        candidate = OPT.fgo_command(
            Path("obs"), Path("nav"), Path("seed"), Path("out"), 50, 1e-6, 0.0
        )
        fixed = {
            (index, value)
            for index, value in enumerate(first)
            if value not in {"8", "0", "0.0", "0e+00"}
        }
        # All graph, measurement, and output arguments remain exactly fixed;
        # compare the two explicit stopping option values only.
        for option in ("--max-iterations", "--relative-cost-threshold", "--absolute-cost-threshold"):
            index = first.index(option)
            self.assertEqual(first[index], candidate[index])
            self.assertEqual(first[index + 1] != candidate[index + 1], option != "--absolute-cost-threshold")
        first_without_stop = list(first)
        candidate_without_stop = list(candidate)
        for option in ("--max-iterations", "--relative-cost-threshold", "--absolute-cost-threshold"):
            index = first_without_stop.index(option)
            del first_without_stop[index : index + 2]
            index = candidate_without_stop.index(option)
            del candidate_without_stop[index : index + 2]
        self.assertEqual(first_without_stop, candidate_without_stop)

    def test_cost_trace_iteration_bound_is_fail_closed(self) -> None:
        header = "phase,local_iteration,global_iteration,cost,absolute_decrease,relative_decrease,update_norm,converged\n"
        with tempfile.TemporaryDirectory(prefix="optimizer_stop_trace_") as raw:
            trace = Path(raw) / "trace.csv"
            trace.write_text(
                header
                + "float,0,0,10,NaN,NaN,1,0\n"
                + "float,1,9,9,1,0.1,0.5,0\n",
                encoding="ascii",
            )
            with self.assertRaises(OPT.OptimizerStopError):
                OPT.validate_cost_trace(trace, expected_max=8)

    def test_atomic_publish_path_rewrite_is_scoped_to_temporary_root(self) -> None:
        with tempfile.TemporaryDirectory(prefix="optimizer_stop_paths_") as raw:
            root = Path(raw)
            temporary = root / ".route.random"
            published = root / "route"
            payload = {
                "artifact": str(temporary / "baseline8" / "fgo.pos"),
                "command": [str(temporary / "inputs" / "device_gnss.csv")],
                "unrelated": str(temporary.parent / "other"),
            }
            rewritten = OPT._rewrite_published_paths(payload, temporary, published)
            self.assertEqual(
                rewritten["artifact"], str(published / "baseline8" / "fgo.pos")
            )
            self.assertEqual(
                rewritten["command"][0],
                str(published / "inputs" / "device_gnss.csv"),
            )
            self.assertEqual(rewritten["unrelated"], str(temporary.parent / "other"))

    def test_converged_by_eight_requires_repeat_byte_identity(self) -> None:
        summary = {"converged": True, "iterations": 8}
        with tempfile.TemporaryDirectory(prefix="optimizer_stop_identity_") as raw:
            first = Path(raw) / "first.pos"
            second = Path(raw) / "second.pos"
            first.write_bytes(b"finite,deterministic\n")
            second.write_bytes(first.read_bytes())
            report = OPT.deterministic_converged_artifact_check(first, second, summary)
            self.assertTrue(report["checked"])
            self.assertTrue(report["byte_identical"])
            second.write_bytes(b"different\n")
            with self.assertRaises(OPT.OptimizerStopError):
                OPT.deterministic_converged_artifact_check(first, second, summary)

    def test_nonconverged_candidate_does_not_claim_identity(self) -> None:
        summary = {"converged": False, "iterations": 50}
        with tempfile.TemporaryDirectory(prefix="optimizer_stop_identity_") as raw:
            first = Path(raw) / "first.pos"
            second = Path(raw) / "second.pos"
            first.write_bytes(b"a")
            second.write_bytes(b"b")
            report = OPT.deterministic_converged_artifact_check(first, second, summary)
            self.assertFalse(report["checked"])
            self.assertIsNone(report["byte_identical"])

    def test_optimizer_scorer_accepts_first_epoch_from_skip_zero_command(self) -> None:
        position = mock.Mock(timestamp_ms=1000)
        with mock.patch.object(OPT.smoother, "_read_positions", return_value=[position]), \
                mock.patch.object(OPT.smoother, "_read_device_epochs", return_value=[1000]) as read_epochs, \
                mock.patch.object(OPT.native_eval, "_raw_rows", return_value=[]), \
                mock.patch.object(OPT.smoother_eval, "_score_rows", return_value={"ok": True}), \
                mock.patch.object(OPT.native_eval, "_normalize_score_schema", side_effect=lambda value: value):
            result = OPT._score_optimizer_position(
                Path("fgo.pos"), Path("device_gnss.csv"), {}
            )
        self.assertEqual(result, {"ok": True})
        read_epochs.assert_called_once_with(Path("device_gnss.csv"), 0)


if __name__ == "__main__":
    unittest.main()
