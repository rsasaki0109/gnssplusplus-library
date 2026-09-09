"""Launch-free Phase147 H/U authorization and argv regression tests.

The fixtures use synthetic placeholders.  No route payload, native summary,
solution row, truth, MAT/PDC artifact, or solver is opened by this test file.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase147_hu_diagnostic_execute.py"
)
AUTH_PATH = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase147_hu_structural_diagnostic_authorization_v1.json"
)


def _load():
    spec = importlib.util.spec_from_file_location("phase147_hu_execute_test", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MODULE = _load()


def test_authorization_and_plan_validate_without_payload_reads() -> None:
    auth = MODULE.read_object(AUTH_PATH, "synthetic Phase147 authorization")
    plan = MODULE.validate_authorization(auth)
    assert plan["routes"] == list(MODULE.ROUTES)
    assert all(value == 0 for value in auth["pre_authorization_read_accounting"].values())


def test_synthetic_h_then_u_argv_is_exact_and_distinct() -> None:
    commands = []
    for route in MODULE.ROUTES:
        command = MODULE.synthetic_command_for(route)
        paths = {
            name: MODULE.ROOT / f"__PHASE147_{name.replace('.', '_').upper()}__"
            for name in MODULE.RAW_NAMES
        }
        MODULE.validate_command(command, route, paths, "0" * 64,
                                MODULE.ROOT / "__phase147_synthetic_output__")
        assert command[command.index("--dataset-id") + 1] == route
        assert command.count(MODULE.PHASE144_SELECTOR) == 1
        commands.append(command)
    assert commands[0] != commands[1]


def test_unknown_route_fails_closed_before_any_payload_path_is_used() -> None:
    with __import__("pytest").raises(MODULE.Phase147AuthorizationError):
        MODULE.synthetic_command_for("unknown-route/pixel5")


def test_wrong_phone_cannot_become_a_known_h_or_u_route() -> None:
    for route in MODULE.ROUTES:
        wrong_phone = route.rsplit("/", 1)[0] + "/pixel7"
        with __import__("pytest").raises(MODULE.Phase147AuthorizationError):
            MODULE.synthetic_command_for(wrong_phone)

