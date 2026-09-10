"""Launch-free Phase128 parser/admission tests using synthetic records only."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase128_glonass_provenance as contract  # noqa: E402

APP_SOURCE = (ROOT / "apps/native/gnss_fgo_imu_no_base.cpp").read_text(encoding="utf-8")
CONFIG_SOURCE = (ROOT / "include/libgnss++/algorithms/fgo_config.hpp").read_text(encoding="utf-8")
PHASE128_HEADER = (ROOT / "include/libgnss++/algorithms/phase128_glonass_provenance.hpp").read_text(encoding="utf-8")


def fixed(value: float, start: int, width: int = 19) -> str:
    text = f"{value:19.12E}"[:width]
    row = [" "] * (start + width)
    row[start : start + width] = text
    return "".join(row)


def row_with_fields(starts: tuple[int, ...], values: tuple[float, ...]) -> str:
    width = max(start + 19 for start in starts)
    row = [" "] * width
    for start, value in zip(starts, values):
        row[start : start + 19] = f"{value:19.12E}"[:19]
    return "".join(row)


def synthetic_nav(prn: int = 7, fcn: float = -4.0, rinex3: bool = True) -> list[str]:
    if rinex3:
        first = f"R{prn:02d} 2022 01 02 03 04 05"
        first_starts = (23, 42, 61)
        continuation_starts = (4, 23, 42, 61)
    else:
        first = f"R{prn:02d} 22 01 02 03 04 05."
        first_starts = (22, 41, 60)
        continuation_starts = (3, 22, 41, 60)
    first_row = list(first.ljust(first_starts[2] + 19))
    for start, value in zip(first_starts, (1.0, 2.0, 3.0)):
        text = f"{value:19.12E}"[:19]
        first_row[start : start + 19] = text
    rows = ["".join(first_row)]
    rows.append(row_with_fields(continuation_starts, (4.0, 5.0, 6.0, 0.0)))
    rows.append(row_with_fields(continuation_starts, (7.0, 8.0, 9.0, fcn)))
    rows.append(row_with_fields(continuation_starts, (10.0, 11.0, 12.0, 13.0)))
    # Keep exactly four lines; the canonical FCN is data[10] on row 3.
    return rows


def header_line(content: str = "") -> str:
    return content.ljust(60) + "GLONASS SLOT / FRQ #"


class Phase128GlonassProvenanceTests(unittest.TestCase):
    def test_cli_and_library_selector_are_default_off_and_composed(self) -> None:
        self.assertIn(contract.SELECTOR, APP_SOURCE)
        self.assertIn("native_phase128_glonass_provenance_parser_admission = false", APP_SOURCE)
        self.assertIn("use_native_phase128_glonass_provenance_parser_admission = false", CONFIG_SOURCE)
        self.assertIn("HeaderStatus::Absent", PHASE128_HEADER)
        self.assertIn("data[0..14]", PHASE128_HEADER)

    def test_encoded_fcn_and_field_position(self) -> None:
        fields = [1.0] * 15
        fields[10] = 252.0
        values, fcn = contract.decode_canonical_geph(fields)
        self.assertEqual(len(values), 15)
        self.assertEqual(fcn, -4)
        fields[9] = float("nan")
        with self.assertRaisesRegex(contract.Phase128ContractError, "nonfinite-field"):
            contract.decode_canonical_geph(fields)

    def test_rinex3_per_record_malformed_does_not_poison_valid_record(self) -> None:
        malformed = synthetic_nav(fcn=252.0)
        malformed[2] = malformed[2][:61] + "bad"
        valid = synthetic_nav(prn=8, fcn=252.0)
        ledger = contract.parse_nav_geph_records(malformed + valid, 3.03)
        self.assertEqual(ledger.records_seen, 2)
        self.assertEqual(ledger.accepted_records[0].satellite, contract.SatelliteId("R", 8))
        self.assertEqual(ledger.accepted_records[0].fcn, -4)
        self.assertEqual(ledger.rejected_records, 1)

    def test_rinex2_year_and_native_gpst_toe(self) -> None:
        ledger = contract.parse_nav_geph_records(synthetic_nav(fcn=252.0, rinex3=False), 2.11)
        self.assertEqual(ledger.records_seen, 1)
        self.assertEqual(len(ledger.accepted_records), 1)
        self.assertEqual(ledger.accepted_records[0].fcn, -4)
        self.assertGreaterEqual(ledger.accepted_records[0].toe_gpst[0], 2000)

    def test_header_states_absent_empty_entries_malformed(self) -> None:
        self.assertEqual(contract.parse_glonass_header([]).status, "absent")
        self.assertEqual(contract.parse_glonass_header([header_line()]).status, "valid-empty")
        entries = list(" " * 60)
        entries[4:10] = list("R07 -4")
        parsed = contract.parse_glonass_header([header_line("".join(entries))])
        self.assertEqual(parsed.status, "entries")
        self.assertEqual(parsed.entries[0], (contract.SatelliteId("R", 7), -4))
        malformed = list(" " * 60)
        malformed[4] = "X"
        self.assertEqual(contract.parse_glonass_header([header_line("".join(malformed))]).status,
                         "malformed")

    def test_absent_header_resolves_and_carrier_frequency_is_ignored(self) -> None:
        records = contract.parse_nav_geph_records(synthetic_nav(fcn=252.0), 3.03).accepted_records
        result = contract.admit_records(
            records,
            [{"satellite": contract.SatelliteId("R", 7),
              "week": records[0].toe_gpst[0], "tow": records[0].toe_gpst[1],
              "carrierFrequencyHz": 999999999.0}],
            contract.parse_glonass_header([]),
        )
        self.assertTrue(result["full_coverage"])
        self.assertFalse(result["phone_carrier_frequency_used_as_fcn"])

    def test_tie_and_header_mismatch_fail_closed(self) -> None:
        records = list(contract.parse_nav_geph_records(synthetic_nav(fcn=252.0), 3.03).accepted_records)
        other = contract.CanonicalGephRecord(records[0].satellite,
                                             records[0].toe_gpst,
                                             records[0].fields, -3)
        query = {"satellite": records[0].satellite, "week": records[0].toe_gpst[0],
                 "tow": records[0].toe_gpst[1]}
        tie = contract.admit_records(records + [other], [query])
        self.assertEqual(tie["failure_counts"].get("different-fcn-tie"), 1)
        header = contract.HeaderLedger("entries", ((records[0].satellite, -3),), 1, 0, {})
        mismatch = contract.admit_records(records, [query], header)
        self.assertEqual(mismatch["failure_counts"].get("header-geph-fcn-mismatch"), 1)

    def test_read_accounting_is_zero(self) -> None:
        accounting = contract.zero_pre_raw_accounting()
        self.assertEqual(accounting["solver_invocations"], 0)
        self.assertEqual(accounting["truth_reads"], 0)
        self.assertFalse(accounting["raw_content_copied_or_transformed"])


if __name__ == "__main__":
    unittest.main()
