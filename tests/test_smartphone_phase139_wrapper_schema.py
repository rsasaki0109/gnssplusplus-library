"""Launch-free tests for the Phase139 Phase138-wrapper schema correction.

The tests exercise only in-memory route metadata and static command
snapshots.  They never hash/open a raw payload, launch a solver, or read
solution/truth/MAT/PDC data.
"""

from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
WRAPPER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase138_affine_tdcp_structural_authorized_execute.py"
)


def _load_wrapper():
    spec = importlib.util.spec_from_file_location(
        "phase139_authorized_wrapper", WRAPPER_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to load authorized wrapper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


WRAPPER = _load_wrapper()


def _route() -> dict:
    authorization = WRAPPER.read_object(WRAPPER.AUTH, "authorization")
    return copy.deepcopy(authorization["routes"][0])


class Phase139WrapperSchemaTests(unittest.TestCase):
    def test_flat_metadata_resolves_without_payload_access(self) -> None:
        route = _route()
        paths = WRAPPER.actual_payload_paths(route)
        self.assertEqual(
            set(paths), {"device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs"}
        )
        self.assertEqual(paths["device_gnss.csv"].name, "device_gnss.csv")
        self.assertEqual(paths["base.obs"].name, "base.obs")

    def test_nested_filename_metadata_is_rejected(self) -> None:
        route = _route()
        metadata = route["raw_inputs"]["device_gnss.csv"]
        route["raw_inputs"]["device_gnss.csv"] = {
            "device_gnss.csv": metadata,
        }
        with self.assertRaises(WRAPPER.AuthorizationError):
            WRAPPER.actual_payload_paths(route)

    def test_missing_key_is_rejected(self) -> None:
        route = _route()
        del route["raw_inputs"]["device_imu.csv"]["sha256"]
        with self.assertRaises(WRAPPER.AuthorizationError):
            WRAPPER.actual_payload_paths(route)

    def test_wrong_metadata_types_are_rejected(self) -> None:
        cases = []

        route = _route()
        route["raw_inputs"]["brdc.nav"] = [route["raw_inputs"]["brdc.nav"]]
        cases.append(route)

        route = _route()
        route["raw_inputs"]["device_gnss.csv"]["bytes"] = True
        cases.append(route)

        route = _route()
        route["base_input"]["interval_s"] = "1"
        cases.append(route)

        route = _route()
        route["base_input"]["hash_read_before_authorization"] = 0
        cases.append(route)

        for bad in cases:
            with self.subTest(route=bad):
                with self.assertRaises(WRAPPER.AuthorizationError):
                    WRAPPER.actual_payload_paths(bad)

    def test_absolute_and_traversal_paths_are_rejected(self) -> None:
        for bad_path in ("../device_gnss.csv", "/tmp/device_gnss.csv", "./device_gnss.csv"):
            route = _route()
            route["raw_inputs"]["device_gnss.csv"]["path"] = bad_path
            with self.subTest(path=bad_path):
                with self.assertRaises(WRAPPER.AuthorizationError):
                    WRAPPER.actual_payload_paths(route)

    def test_unknown_key_and_selector_injection_are_rejected(self) -> None:
        route = _route()
        route["raw_inputs"]["device_gnss.csv"][
            "--native-phase138-affine-tdcp-anchor-range-constant"
        ] = "enabled"
        with self.assertRaises(WRAPPER.AuthorizationError):
            WRAPPER.actual_payload_paths(route)

        route = _route()
        route["raw_inputs"]["--native-phase138-affine-tdcp-anchor-range-constant"] = (
            route["raw_inputs"]["device_gnss.csv"]
        )
        with self.assertRaises(WRAPPER.AuthorizationError):
            WRAPPER.actual_payload_paths(route)

    def test_duplicate_json_keys_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.json"
            path.write_text('{"path":"a","path":"b"}\n', encoding="utf-8")
            with self.assertRaises(WRAPPER.AuthorizationError):
                WRAPPER.read_object(path, "duplicate")

    def test_authorization_schema_version_is_required(self) -> None:
        authorization = WRAPPER.read_object(WRAPPER.AUTH, "authorization")
        authorization["schema_version"] = "wrong-schema"
        with self.assertRaises(WRAPPER.AuthorizationError):
            WRAPPER.verify_authorization(authorization)

    def test_selector_snapshot_remains_isolated(self) -> None:
        for route in WRAPPER.STATIC.ROUTES:
            command = WRAPPER.STATIC.command_template(route)
            WRAPPER.validate_typed_command(command, route)
            self.assertEqual(
                command.count(WRAPPER.STATIC.PHASE138_SELECTOR), 1
            )
            self.assertEqual(
                command.count(WRAPPER.STATIC.PHASE135_SELECTOR), 1
            )
            self.assertEqual(
                command.count(WRAPPER.STATIC.PHASE118_SELECTOR), 1
            )


if __name__ == "__main__":
    unittest.main()
