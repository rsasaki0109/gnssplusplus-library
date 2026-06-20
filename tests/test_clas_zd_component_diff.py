#!/usr/bin/env python3
"""Tests for CLAS zero-difference component comparison helpers."""

from __future__ import annotations

import csv
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "clas_zd_component_diff.py"

spec = importlib.util.spec_from_file_location("clas_zd_component_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
component_diff = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = component_diff
spec.loader.exec_module(component_diff)


FIELDNAMES = [
    "record",
    "stage",
    "week",
    "tow",
    "sat",
    "freq",
    "signal",
    "pseudorange_rinex_code",
    "carrier_rinex_code",
    "pseudorange_rtklib_code",
    "carrier_rtklib_code",
    "signal_family",
    "requested_pseudorange_rinex_code",
    "requested_carrier_rinex_code",
    "observation_exact_identity_requested",
    "observation_exact_match",
    "observation_family_fallback",
    "bias_exact_identity",
    "code_bias_source_signal_id",
    "phase_bias_source_signal_id",
    "code_bias_present",
    "phase_bias_present",
    "code_bias_fallback",
    "phase_bias_fallback",
    "prc_m",
    "PRC",
    "cpc_m",
    "code_bias_m",
    "code_bias",
    "phase_bias_m",
    "receiver_ant_m",
    "receiver_antenna_m",
    "trop_correction_m",
    "iono_l1_m",
    "stec_tecu",
    "iono_scaled_m",
    "iono_scale",
    "relativity_m",
    "atmos_ref_week",
    "atmos_ref_tow",
    "clock_ref_week",
    "clock_ref_tow",
    "atmos_clock_gap_s",
]


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def code_row(
    *,
    sat: str,
    freq: str,
    prc: str,
    code_bias: str,
    receiver_ant: str,
    trop: str = "2.0",
    iono_l1: str = "0.0",
    stec_tecu: str = "",
    iono_scaled: str = "0.0",
    iono_scale: str = "",
    relativity: str = "0.0",
    atmos_ref_week: str = "",
    atmos_ref_tow: str = "",
    clock_ref_week: str = "",
    clock_ref_tow: str = "",
    atmos_clock_gap_s: str = "",
    record: str = "CODE",
    stage: str = "",
    signal: str = "",
    pseudorange_rinex_code: str = "",
    carrier_rinex_code: str = "",
    pseudorange_rtklib_code: str = "",
    carrier_rtklib_code: str = "",
    signal_family: str = "",
    requested_pseudorange_rinex_code: str = "",
    requested_carrier_rinex_code: str = "",
    observation_exact_identity_requested: str = "",
    observation_exact_match: str = "",
    observation_family_fallback: str = "",
    bias_exact_identity: str = "",
    code_bias_source_signal_id: str = "",
    phase_bias_source_signal_id: str = "",
    code_bias_present: str = "",
    phase_bias_present: str = "",
    code_bias_fallback: str = "",
    phase_bias_fallback: str = "",
) -> dict[str, str]:
    return {
        "record": record,
        "stage": stage,
        "week": "2068",
        "tow": "230425.000",
        "sat": sat,
        "freq": freq,
        "signal": signal,
        "pseudorange_rinex_code": pseudorange_rinex_code,
        "carrier_rinex_code": carrier_rinex_code,
        "pseudorange_rtklib_code": pseudorange_rtklib_code,
        "carrier_rtklib_code": carrier_rtklib_code,
        "signal_family": signal_family,
        "requested_pseudorange_rinex_code": requested_pseudorange_rinex_code,
        "requested_carrier_rinex_code": requested_carrier_rinex_code,
        "observation_exact_identity_requested": observation_exact_identity_requested,
        "observation_exact_match": observation_exact_match,
        "observation_family_fallback": observation_family_fallback,
        "bias_exact_identity": bias_exact_identity,
        "code_bias_source_signal_id": code_bias_source_signal_id,
        "phase_bias_source_signal_id": phase_bias_source_signal_id,
        "code_bias_present": code_bias_present,
        "phase_bias_present": phase_bias_present,
        "code_bias_fallback": code_bias_fallback,
        "phase_bias_fallback": phase_bias_fallback,
        "prc_m": prc,
        "PRC": "",
        "cpc_m": "",
        "code_bias_m": code_bias,
        "code_bias": "",
        "phase_bias_m": "",
        "receiver_ant_m": receiver_ant,
        "receiver_antenna_m": "",
        "trop_correction_m": trop,
        "iono_l1_m": iono_l1,
        "stec_tecu": stec_tecu,
        "iono_scaled_m": iono_scaled,
        "iono_scale": iono_scale,
        "relativity_m": relativity,
        "atmos_ref_week": atmos_ref_week,
        "atmos_ref_tow": atmos_ref_tow,
        "clock_ref_week": clock_ref_week,
        "clock_ref_tow": clock_ref_tow,
        "atmos_clock_gap_s": atmos_clock_gap_s,
    }


class ClasZdComponentDiffTest(unittest.TestCase):
    def test_derives_prc_closure_components_from_existing_fields(self) -> None:
        components = component_diff.extract_components(
            code_row(
                sat="G14",
                freq="1",
                prc="3.0",
                code_bias="0.5",
                receiver_ant="0.1",
                trop="2.0",
                iono_scaled="0.3",
                relativity="0.02",
            ),
            ["prc_component_sum_m", "prc_closure_residual_m"],
        )

        self.assertAlmostEqual(components["prc_component_sum_m"], 2.92)
        self.assertAlmostEqual(components["prc_closure_residual_m"], 0.08)

    def test_extracts_atmosphere_reference_components(self) -> None:
        components = component_diff.extract_components(
            code_row(
                sat="G14",
                freq="1",
                prc="3.0",
                code_bias="0.5",
                receiver_ant="0.1",
                atmos_ref_week="2068",
                atmos_ref_tow="230430.0",
                clock_ref_week="2068",
                clock_ref_tow="230420.0",
                atmos_clock_gap_s="10.0",
            ),
            [
                "atmos_ref_week",
                "atmos_ref_tow",
                "clock_ref_week",
                "clock_ref_tow",
                "atmos_clock_gap_s",
            ],
        )

        self.assertEqual(components["atmos_ref_week"], 2068.0)
        self.assertEqual(components["atmos_ref_tow"], 230430.0)
        self.assertEqual(components["clock_ref_week"], 2068.0)
        self.assertEqual(components["clock_ref_tow"], 230420.0)
        self.assertEqual(components["atmos_clock_gap_s"], 10.0)

    def test_skips_prc_closure_when_required_component_is_missing(self) -> None:
        components = component_diff.extract_components(
            code_row(
                sat="G14",
                freq="1",
                prc="3.0",
                code_bias="",
                receiver_ant="0.1",
                trop="2.0",
                iono_scaled="0.3",
                relativity="0.02",
            ),
            ["prc_component_sum_m", "prc_closure_residual_m"],
        )

        self.assertNotIn("prc_component_sum_m", components)
        self.assertNotIn("prc_closure_residual_m", components)

    def test_derives_iono_scale_from_gps_rinex_band(self) -> None:
        components = component_diff.extract_components(
            code_row(
                sat="G14",
                freq="1",
                prc="3.0",
                code_bias="0.5",
                receiver_ant="0.1",
                pseudorange_rinex_code="C2W",
            ),
            ["iono_scale"],
        )

        self.assertAlmostEqual(
            components["iono_scale"],
            component_diff.GPS_IONO_SCALE_BY_BAND["2"],
        )

    def test_raw_iono_scale_overrides_rinex_band(self) -> None:
        components = component_diff.extract_components(
            code_row(
                sat="G14",
                freq="1",
                prc="3.0",
                code_bias="0.5",
                receiver_ant="0.1",
                pseudorange_rinex_code="C2W",
                iono_scale="1.25",
            ),
            ["iono_scale"],
        )

        self.assertAlmostEqual(components["iono_scale"], 1.25)

    def test_derives_iono_scaled_closure_residual(self) -> None:
        scale = component_diff.GPS_IONO_SCALE_BY_BAND["2"]
        components = component_diff.extract_components(
            code_row(
                sat="G14",
                freq="1",
                prc="3.0",
                code_bias="0.5",
                receiver_ant="0.1",
                pseudorange_rinex_code="C2W",
                iono_l1="0.25",
                iono_scaled=str(scale * 0.25 + 0.01),
            ),
            ["iono_scaled_closure_residual_m"],
        )

        self.assertAlmostEqual(components["iono_scaled_closure_residual_m"], 0.01)

    def test_derives_iono_l1_from_stec_tecu(self) -> None:
        components = component_diff.extract_components(
            code_row(
                sat="G14",
                freq="1",
                prc="3.0",
                code_bias="0.5",
                receiver_ant="0.1",
                stec_tecu="2.0",
            ),
            ["iono_l1_from_stec_m"],
        )

        self.assertAlmostEqual(
            components["iono_l1_from_stec_m"],
            2.0 * component_diff.GPS_L1_TECU_TO_METERS,
        )

    def test_derives_iono_l1_stec_closure_residual(self) -> None:
        iono_l1_from_stec = 2.0 * component_diff.GPS_L1_TECU_TO_METERS
        components = component_diff.extract_components(
            code_row(
                sat="G14",
                freq="1",
                prc="3.0",
                code_bias="0.5",
                receiver_ant="0.1",
                iono_l1=str(iono_l1_from_stec + 0.02),
                stec_tecu="2.0",
            ),
            ["iono_l1_stec_closure_residual_m"],
        )

        self.assertAlmostEqual(components["iono_l1_stec_closure_residual_m"], 0.02)

    def test_observation_identity_prefers_row_type_specific_rinex_code(self) -> None:
        self.assertEqual(
            component_diff.observation_identity(
                {
                    "record": "CODE",
                    "signal": "3",
                    "pseudorange_rinex_code": "C2W",
                    "carrier_rinex_code": "L2W",
                },
                "code",
            ),
            "C2W",
        )
        self.assertEqual(
            component_diff.observation_identity(
                {
                    "record": "PHASE",
                    "signal": "3",
                    "pseudorange_rinex_code": "C2W",
                    "carrier_rinex_code": "L2W",
                },
                "phase",
            ),
            "L2W",
        )

    def test_lookup_provenance_columns_can_be_compared(self) -> None:
        components = component_diff.extract_components(
            code_row(
                sat="G14",
                freq="1",
                prc="0.50",
                code_bias="0.10",
                receiver_ant="-0.02",
                bias_exact_identity="1",
                code_bias_source_signal_id="9",
                phase_bias_source_signal_id="9",
                code_bias_present="1",
                phase_bias_present="1",
                code_bias_fallback="0",
                phase_bias_fallback="0",
                observation_exact_identity_requested="1",
                observation_exact_match="1",
                observation_family_fallback="0",
            ),
            [
                "bias_exact_identity",
                "observation_exact_identity_requested",
                "observation_exact_match",
                "observation_family_fallback",
                "code_bias_source_signal_id",
                "phase_bias_source_signal_id",
                "code_bias_present",
                "phase_bias_present",
                "code_bias_fallback",
                "phase_bias_fallback",
            ],
        )

        self.assertEqual(
            components,
            {
                "bias_exact_identity": 1.0,
                "observation_exact_identity_requested": 1.0,
                "observation_exact_match": 1.0,
                "observation_family_fallback": 0.0,
                "code_bias_source_signal_id": 9.0,
                "phase_bias_source_signal_id": 9.0,
                "code_bias_present": 1.0,
                "phase_bias_present": 1.0,
                "code_bias_fallback": 0.0,
                "phase_bias_fallback": 0.0,
            },
        )

    def test_exact_observation_code_is_part_of_row_key(self) -> None:
        base = component_diff.normalize_rows(
            [
                code_row(
                    sat="G14",
                    freq="1",
                    prc="0.50",
                    code_bias="0.10",
                    receiver_ant="-0.02",
                    pseudorange_rinex_code="C2W",
                )
            ],
            component_names=["prc_m"],
            stage_filter=None,
            row_type_filter="code",
        )
        candidate = component_diff.normalize_rows(
            [
                code_row(
                    sat="G14",
                    freq="1",
                    prc="0.50",
                    code_bias="0.10",
                    receiver_ant="-0.02",
                    pseudorange_rinex_code="C2X",
                )
            ],
            component_names=["prc_m"],
            stage_filter=None,
            row_type_filter="code",
        )

        report = component_diff.build_report(
            base,
            candidate,
            base_label="claslib",
            candidate_label="native",
            threshold_m=None,
            top_deltas=10,
            top_unmatched=10,
        )

        self.assertEqual(report["common_rows"], 0)
        self.assertEqual(report["base_only_rows"], 1)
        self.assertEqual(report["candidate_only_rows"], 1)
        self.assertEqual(report["top_base_only"][0]["rinex_code"], "C2W")
        self.assertEqual(report["top_candidate_only"][0]["rinex_code"], "C2X")

    def test_slice_filters_apply_after_row_identity_normalization(self) -> None:
        rows = [
            code_row(
                sat="G14",
                freq="1",
                prc="0.50",
                code_bias="0.10",
                receiver_ant="-0.02",
                pseudorange_rinex_code="C2W",
            ),
            code_row(
                sat="G14",
                freq="1",
                prc="0.60",
                code_bias="0.10",
                receiver_ant="-0.02",
                pseudorange_rinex_code="C2X",
            ),
            code_row(
                sat="G25",
                freq="1",
                prc="0.70",
                code_bias="0.10",
                receiver_ant="-0.02",
                pseudorange_rinex_code="C2W",
            ),
            code_row(
                sat="G14",
                freq="0",
                prc="0.80",
                code_bias="0.10",
                receiver_ant="-0.02",
                pseudorange_rinex_code="C1C",
            ),
        ]

        filtered = component_diff.normalize_rows(
            rows,
            component_names=["prc_m"],
            stage_filter=None,
            row_type_filter="code",
            sat_filter={"G14"},
            freq_filter={1},
            rinex_code_filter={"C2W"},
        )

        self.assertEqual(len(filtered), 1)
        self.assertEqual(filtered[0].sat, "G14")
        self.assertEqual(filtered[0].freq, 1)
        self.assertEqual(filtered[0].signal, "C2W")

    def test_duplicate_policy_controls_repeated_row_keys(self) -> None:
        base = component_diff.normalize_rows(
            [
                code_row(
                    sat="G14",
                    freq="1",
                    prc="0.50",
                    code_bias="0.10",
                    receiver_ant="-0.02",
                    pseudorange_rinex_code="C2W",
                ),
                code_row(
                    sat="G14",
                    freq="1",
                    prc="0.70",
                    code_bias="0.10",
                    receiver_ant="-0.02",
                    pseudorange_rinex_code="C2W",
                ),
            ],
            component_names=["prc_m"],
            stage_filter=None,
            row_type_filter="code",
        )
        candidate = component_diff.normalize_rows(
            [
                code_row(
                    sat="G14",
                    freq="1",
                    prc="1.00",
                    code_bias="0.10",
                    receiver_ant="-0.02",
                    pseudorange_rinex_code="C2W",
                )
            ],
            component_names=["prc_m"],
            stage_filter=None,
            row_type_filter="code",
        )

        last_report = component_diff.build_report(
            base,
            candidate,
            base_label="claslib",
            candidate_label="native",
            threshold_m=None,
            top_deltas=10,
            top_unmatched=10,
        )
        self.assertEqual(last_report["duplicate_policy"], "last")
        self.assertEqual(last_report["base_duplicate_keys"], 1)
        self.assertAlmostEqual(last_report["top_component_deltas"][0]["delta_m"], 0.30)

        mean_report = component_diff.build_report(
            base,
            candidate,
            base_label="claslib",
            candidate_label="native",
            threshold_m=None,
            top_deltas=10,
            top_unmatched=10,
            duplicate_policy="mean",
        )
        self.assertEqual(mean_report["duplicate_policy"], "mean")
        self.assertEqual(mean_report["base_duplicate_keys"], 1)
        self.assertAlmostEqual(mean_report["top_component_deltas"][0]["delta_m"], 0.40)

        with self.assertRaisesRegex(ValueError, "duplicate row keys are present"):
            component_diff.build_report(
                base,
                candidate,
                base_label="claslib",
                candidate_label="native",
                threshold_m=None,
                top_deltas=10,
                top_unmatched=10,
                duplicate_policy="fail",
            )

    def test_compares_common_component_rows_and_reports_unmatched(self) -> None:
        base_rows = [
            code_row(
                sat="G14",
                freq="1",
                prc="0.50",
                code_bias="0.10",
                receiver_ant="-0.02",
                pseudorange_rinex_code="C2W",
            ),
            code_row(sat="J01", freq="0", prc="3.75", code_bias="0.00", receiver_ant="-0.05"),
        ]
        candidate_rows = [
            code_row(
                sat="G14",
                freq="1",
                prc="0.75",
                code_bias="0.10",
                receiver_ant="0.00",
                stage="accepted",
                signal="3",
                pseudorange_rinex_code="C2W",
                atmos_ref_tow="230430.0",
                clock_ref_tow="230420.0",
                atmos_clock_gap_s="10.0",
            ),
            code_row(
                sat="J02",
                freq="0",
                prc="5.00",
                code_bias="0.00",
                receiver_ant="-0.08",
                stage="accepted",
                signal="18",
            ),
        ]

        base = component_diff.normalize_rows(
            base_rows,
            component_names=[
                "prc_m",
                "code_bias_m",
                "receiver_antenna_m",
                "atmos_ref_tow",
                "clock_ref_tow",
                "atmos_clock_gap_s",
            ],
            stage_filter=None,
            row_type_filter="code",
        )
        candidate = component_diff.normalize_rows(
            candidate_rows,
            component_names=[
                "prc_m",
                "code_bias_m",
                "receiver_antenna_m",
                "atmos_ref_tow",
                "clock_ref_tow",
                "atmos_clock_gap_s",
            ],
            stage_filter=None,
            row_type_filter="code",
        )
        report = component_diff.build_report(
            base,
            candidate,
            base_label="claslib",
            candidate_label="native",
            threshold_m=0.1,
            top_deltas=10,
            top_unmatched=10,
        )

        self.assertEqual(report["schema"], "clas_zd_component_diff.v1")
        self.assertEqual(report["base_rows"], 2)
        self.assertEqual(report["candidate_rows"], 2)
        self.assertEqual(report["common_rows"], 1)
        self.assertEqual(report["base_only_rows"], 1)
        self.assertEqual(report["candidate_only_rows"], 1)
        self.assertEqual(report["threshold_exceedances"], 1)
        self.assertEqual(report["top_base_only"][0]["sat"], "J01")
        self.assertEqual(report["top_candidate_only"][0]["sat"], "J02")
        self.assertEqual(report["top_component_deltas"][0]["component"], "prc_m")
        self.assertAlmostEqual(report["top_component_deltas"][0]["delta_m"], 0.25)
        self.assertEqual(report["top_component_deltas"][0]["candidate_signal"], "C2W")
        row_breakdown = report["top_row_component_breakdowns"][0]
        self.assertEqual(row_breakdown["sat"], "G14")
        self.assertEqual(row_breakdown["rinex_code"], "C2W")
        self.assertEqual(row_breakdown["component_count"], 3)
        self.assertEqual(row_breakdown["missing_component_count"], 3)
        self.assertAlmostEqual(row_breakdown["sum_abs_delta_m"], 0.27)
        self.assertEqual(
            [item["component"] for item in row_breakdown["components"]],
            ["prc_m", "receiver_antenna_m", "code_bias_m"],
        )
        self.assertAlmostEqual(row_breakdown["components"][0]["delta_m"], 0.25)
        missing_by_component = {
            item["component"]: item for item in row_breakdown["missing_components"]
        }
        self.assertFalse(missing_by_component["atmos_ref_tow"]["base_present"])
        self.assertTrue(missing_by_component["atmos_ref_tow"]["candidate_present"])
        self.assertEqual(missing_by_component["atmos_ref_tow"]["candidate_value"], 230430.0)
        self.assertEqual(missing_by_component["clock_ref_tow"]["candidate_value"], 230420.0)
        self.assertEqual(missing_by_component["atmos_clock_gap_s"]["candidate_value"], 10.0)

    def test_report_summarizes_gps_l2w_identity_provenance(self) -> None:
        base = component_diff.normalize_rows(
            [
                code_row(
                    sat="G14",
                    freq="1",
                    prc="0.50",
                    code_bias="0.10",
                    receiver_ant="-0.02",
                    pseudorange_rinex_code="C2W",
                )
            ],
            component_names=[
                "bias_exact_identity",
                "observation_exact_identity_requested",
                "observation_exact_match",
                "observation_family_fallback",
                "code_bias_present",
                "phase_bias_present",
                "code_bias_fallback",
                "phase_bias_fallback",
            ],
            stage_filter=None,
            row_type_filter="code",
        )
        candidate = component_diff.normalize_rows(
            [
                code_row(
                    sat="G14",
                    freq="1",
                    prc="0.50",
                    code_bias="0.10",
                    receiver_ant="-0.02",
                    pseudorange_rinex_code="C2W",
                    bias_exact_identity="1",
                    observation_exact_identity_requested="1",
                    observation_exact_match="1",
                    observation_family_fallback="0",
                    code_bias_present="1",
                    phase_bias_present="1",
                    code_bias_fallback="0",
                    phase_bias_fallback="0",
                ),
                code_row(
                    sat="G25",
                    freq="1",
                    prc="0.75",
                    code_bias="0.09",
                    receiver_ant="-0.02",
                    pseudorange_rinex_code="C2W",
                    bias_exact_identity="0",
                    observation_exact_identity_requested="1",
                    observation_exact_match="0",
                    observation_family_fallback="1",
                    code_bias_present="1",
                    phase_bias_present="0",
                    code_bias_fallback="1",
                    phase_bias_fallback="0",
                ),
                code_row(
                    sat="J01",
                    freq="1",
                    prc="0.30",
                    code_bias="0.00",
                    receiver_ant="-0.02",
                    pseudorange_rinex_code="C2X",
                    bias_exact_identity="1",
                    observation_exact_identity_requested="1",
                    observation_exact_match="1",
                    code_bias_present="1",
                ),
            ],
            component_names=[
                "bias_exact_identity",
                "observation_exact_identity_requested",
                "observation_exact_match",
                "observation_family_fallback",
                "code_bias_present",
                "phase_bias_present",
                "code_bias_fallback",
                "phase_bias_fallback",
            ],
            stage_filter=None,
            row_type_filter="code",
        )

        report = component_diff.build_report(
            base,
            candidate,
            base_label="claslib",
            candidate_label="native",
            threshold_m=None,
            top_deltas=10,
            top_unmatched=10,
        )

        native = report["identity_provenance"]["native"]
        self.assertEqual(native["rows"], 3)
        self.assertEqual(native["gps_l2w_rows"], 2)
        self.assertEqual(native["gps_l2w_bias_exact_identity_rows"], 1)
        self.assertEqual(native["gps_l2w_observation_exact_identity_requested_rows"], 2)
        self.assertEqual(native["gps_l2w_observation_exact_match_rows"], 1)
        self.assertEqual(native["gps_l2w_observation_family_fallback_rows"], 1)
        self.assertEqual(native["gps_l2w_code_bias_present_rows"], 2)
        self.assertEqual(native["gps_l2w_phase_bias_present_rows"], 1)
        self.assertEqual(native["gps_l2w_code_bias_fallback_rows"], 1)
        self.assertEqual(native["bias_exact_identity_rows"], 2)

    def test_cli_writes_schema_json_and_details_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="clas_zd_component_diff_test_") as temp_dir:
            root = Path(temp_dir)
            base_csv = root / "claslib.csv"
            candidate_csv = root / "native.csv"
            report_path = root / "report.json"
            details_path = root / "details.csv"
            write_csv(
                base_csv,
                [
                    code_row(
                        sat="G14",
                        freq="1",
                        prc="0.50",
                        code_bias="0.10",
                        receiver_ant="-0.02",
                        pseudorange_rinex_code="C2W",
                    )
                ],
            )
            write_csv(
                candidate_csv,
                [
                    code_row(
                        sat="G14",
                        freq="1",
                        prc="0.75",
                        code_bias="0.10",
                        receiver_ant="0.00",
                        signal="3",
                        pseudorange_rinex_code="C2W",
                    )
                ],
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(base_csv),
                    str(candidate_csv),
                    "--base-label",
                    "claslib",
                    "--candidate-label",
                    "native",
                    "--row-type",
                    "code",
                    "--sat",
                    "G14",
                    "--freq",
                    "1",
                    "--rinex-code",
                    "C2W",
                    "--duplicate-policy",
                    "mean",
                    "--component",
                    "prc_m",
                    "--component",
                    "receiver_antenna_m",
                    "--component-threshold-m",
                    "0.1",
                    "--json-out",
                    str(report_path),
                    "--details-csv",
                    str(details_path),
                    "--fail-on-diff",
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 2, msg=result.stderr)
            self.assertIn("clas_zd_component_diff:", result.stdout)
            self.assertIn("max_row_delta:", result.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["schema"], "clas_zd_component_diff.v1")
            self.assertEqual(report["threshold_exceedances"], 1)
            self.assertEqual(report["component_names"], ["prc_m", "receiver_antenna_m"])
            self.assertEqual(report["sat_filter"], ["G14"])
            self.assertEqual(report["freq_filter"], [1])
            self.assertEqual(report["rinex_code_filter"], ["C2W"])
            self.assertEqual(report["duplicate_policy"], "mean")
            self.assertEqual(report["top_row_component_breakdowns"][0]["sat"], "G14")
            self.assertEqual(
                report["top_row_component_breakdowns"][0]["components"][0]["component"],
                "prc_m",
            )
            with details_path.open(newline="", encoding="utf-8") as details_file:
                rows = list(csv.DictReader(details_file))
            self.assertEqual(rows[0]["component"], "prc_m")
            self.assertEqual(rows[0]["sat"], "G14")
            self.assertEqual(rows[0]["rinex_code"], "C2W")
            self.assertEqual(rows[0]["candidate_signal"], "C2W")
            self.assertEqual(float(rows[0]["delta_m"]), 0.25)


if __name__ == "__main__":
    unittest.main()
