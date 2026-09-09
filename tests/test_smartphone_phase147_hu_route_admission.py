"""Focused Phase147 route-admission checks.

These tests inspect only the tracked native source and the Phase147 plan.  They
do not materialize Android GNSS/IMU, navigation, base, solution, truth, MAT,
or Kaggle artifacts and do not start the native executable.
"""

from __future__ import annotations

import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
PLAN = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase147_hu_structural_preflight_manifest_v1.json"
)

EXPECTED_ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
H_ROUTE = EXPECTED_ROUTES[2]
U_ROUTE = EXPECTED_ROUTES[3]


def _route_allowlist(source: str, selector: str) -> tuple[str, ...]:
    marker = f"if (options.{selector})"
    start = source.index(marker)
    block_start = source.index("const bool supported_route", start)
    block_end = source.index(";", block_start)
    block = source[block_start:block_end]
    return tuple(re.findall(r'options\.dataset_id == "([^"]+)"', block))


def _source() -> str:
    return SOURCE.read_text(encoding="utf-8")


def test_phase135_and_phase143_accept_exactly_four_known_pixel5_routes() -> None:
    source = _source()
    for selector in (
        "native_phase135_official_affine_measurement_family",
        "native_phase143_official_main_lm_termination_budget",
    ):
        routes = _route_allowlist(source, selector)
        assert routes == EXPECTED_ROUTES
        assert len(routes) == len(set(routes))
        assert all(route.endswith("/pixel5") for route in routes)


def test_unknown_and_non_pixel5_routes_are_rejected_by_the_allowlist() -> None:
    source = _source()
    routes = set(_route_allowlist(
        source, "native_phase135_official_affine_measurement_family"
    ))
    assert H_ROUTE in routes and U_ROUTE in routes
    assert "2021-08-24-20-32-us-ca-mtv-h/pixel7" not in routes
    assert "unknown-route/pixel5" not in routes
    assert "2021-08-24-20-32-us-ca-mtv-h/pixel5/extra" not in routes


def test_h_u_type_mapping_and_pixel5_offset_guards_remain_source_backed() -> None:
    source = _source()
    config = (ROOT / "include/libgnss++/algorithms/fgo_config.hpp").read_text(
        encoding="utf-8"
    )
    assert f'{{"{H_ROUTE}", "Street", 0.1, 0.4}}' in source
    assert f'{{"{U_ROUTE}", "Street", 0.1, 0.4}}' in source
    assert 'setting_type == "Street" || setting_type == "Mix"' in config
    assert "threshold_sigma = 0.2" in config
    offset = (ROOT / "include/libgnss++/algorithms/upstream_position_offset.hpp").read_text(
        encoding="utf-8"
    )
    assert 'phone.find("pixel5")' in offset
    assert "phoneFromDatasetId(options.dataset_id) != \"pixel5\"" in source


def test_phase147_plan_is_diagnostic_only_and_has_no_truth_authority() -> None:
    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    assert plan["phase"] == 147
    assert plan["routes"] == [H_ROUTE, U_ROUTE]
    assert plan["candidate"]["diagnostic_only"] is True
    assert plan["candidate"]["algorithm_or_solver_change"] is False
    boundary = plan["execution_boundary"]
    assert boundary["raw_execution_authorized"] is False
    assert boundary["solver_execution_authorized"] is False
    assert boundary["truth_authorized"] is False
    assert boundary["accuracy_authorized"] is False
    assert boundary["solution_publication_authorized"] is False
    assert boundary["kaggle_or_token_authorized"] is False
    accounting = plan["read_accounting_before_authorization"]
    assert all(value == 0 for value in accounting.values())
