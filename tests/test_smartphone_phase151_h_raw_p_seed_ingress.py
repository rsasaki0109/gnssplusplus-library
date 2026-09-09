"""Launch-free Phase151 raw-clock ingress correction checks.

This test reads only tracked source and sealed JSON records.  It uses
synthetic argv values and never opens route GNSS/nav/IMU/base, solution,
truth, MAT, or Kaggle payloads.
"""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PHASE150_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase150_h_raw_p_seed_diagnostic_manifest_v1.json"
)
PHASE150_RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase150_h_raw_p_seed_diagnostic_result_v1.json"
)
PHASE151_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase151_h_raw_p_seed_ingress_correction_manifest_v1.json"
)
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
LOADER = ROOT / "src/io/android_raw_gnss.cpp"


def _json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_corrected_synthetic_phase149_argv_selects_raw_clock_contract() -> None:
    old = _json(PHASE150_MANIFEST)
    corrected = _json(PHASE151_MANIFEST)
    old_argv = old["command"]["argv"]
    argv = corrected["corrected_command"]["argv"]

    assert "--android-raw-clock-only" not in old_argv
    assert argv.count("--native-phase149-raw-p-seed-stage") == 1
    assert argv.count("--android-raw-clock-only") == 1
    assert "--android-gnss" in argv and "--nav" in argv
    assert "--android-imu" not in argv
    assert "--out" not in argv
    assert "--native-phase144-telemetry-schema" not in argv
    assert "--native-base-pseudorange-compensation" not in argv
    assert corrected["corrected_command"]["no_payload_read_in_test"] is True


def test_parser_predicate_and_app_mapping_are_source_backed() -> None:
    app = APP.read_text(encoding="utf-8")
    loader = LOADER.read_text(encoding="utf-8")

    assert "android_gnss_config.verify_enriched_pseudorange =" in app
    assert "!options.android_raw_clock_only" in app
    assert "std::abs(lhs - rhs) <= tolerance" in loader
    assert "config.enriched_pseudorange_tolerance_m" in loader
    assert "if (config.verify_enriched_pseudorange)" in loader
    assert "raw-clock pseudorange disagrees with enriched RawPseudorangeMeters" in loader


def test_historical_failure_is_preserved_and_correction_does_not_use_enriched_p() -> None:
    old = _json(PHASE150_MANIFEST)
    result = _json(PHASE150_RESULT)
    corrected = _json(PHASE151_MANIFEST)

    assert result["status"] == "fail-closed-at-raw-gnss-ingress"
    assert result["execution"]["invocation_count"] == 1
    assert result["execution"]["rerun"] is False
    assert result["read_accounting"]["native_spp_invocations"] == 0
    assert result["read_accounting"]["broadcast_navigation_reads"] == 0
    assert result["read_accounting"]["truth_reads"] == 0
    assert corrected["source_rule"]["estimator_pseudorange"] == "raw-clock reconstruction only"
    assert corrected["source_rule"]["enriched_RawPseudorangeMeters"] == "diagnostic-only-and-ignored"
    assert old["stage_contract"]["external_or_precomputed_positions"] is False
