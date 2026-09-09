"""Focused, raw/truth-free qualification for the Phase105 linkage repair."""

from __future__ import annotations

import importlib
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
MODULE_DIR = ROOT / "apps/commands/benchmarks"
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import gnss_smartphone_phase105_runtime_linkage_attribution as contract


def test_phase105_freeze_single_child_only_candidate() -> None:
    freeze = contract.verify_phase105_freeze()
    candidate = freeze["candidate"]
    assert candidate["candidate_count"] == 1
    assert candidate["child_process_only"] is True
    assert candidate["exact_assignment"].startswith("LD_LIBRARY_PATH=/home/sasaki/.local/lib:")
    assert candidate["parent_environment_changed"] is False
    assert candidate["native_binary_changed"] is False
    assert candidate["solver_or_graph_changed"] is False
    assert candidate["no_fallback_or_rerun"] is True


def test_phase105_manifest_reuses_phase104_graph_and_two_route_contract() -> None:
    manifest = contract.verify_manifest()
    assert manifest["phase105_linkage_authority"]["parent_environment_changed"] is False
    assert manifest["phase105_linkage_authority"]["solver_changed"] is False
    assert manifest["candidate"]["routes"] == list(contract.ROUTES)
    assert manifest["candidate"]["runs_per_route"] == 1
    assert manifest["matrix"]["native_invocations_planned"] == 2
    assert manifest["matrix"]["truth_reads_planned"] == 0
    assert manifest["read_accounting_before_execution"]["native_solver_invocations"] == 0


def test_pre_raw_is_launch_free_and_no_input_reads() -> None:
    result = contract.verify_pre_raw()
    assert result["status"] == "pre-raw-verified"
    assert result["native_solver_invocations"] == 0
    assert result["raw_reads"] == 0
    assert result["truth_reads"] == 0
    assert result["accuracy_calculations"] == 0


def test_linkage_fix_is_exact_child_environment_prepend() -> None:
    wrapper = importlib.import_module("gnss_smartphone_phase105_runtime_linkage_execute")
    environment = wrapper._safe_environment()
    assert environment["LD_LIBRARY_PATH"].split(":", 1)[0] == "/home/sasaki/.local/lib"
    assert environment["LC_ALL"] == "C"
    assert environment["LANG"] == "C"
    assert environment["TZ"] == "UTC"


def test_phase104_native_source_and_binary_pins_remain_unchanged() -> None:
    details = contract._base.verify_implementation()
    assert details["implementation_commit"] == "88f726a02d98e6dddd84e561b427ded43368e61d"
    assert details["paths"]["apps/native/gnss_fgo_imu_no_base.cpp"] == contract.APP_SHA
