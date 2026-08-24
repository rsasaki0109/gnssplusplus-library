"""Unit and negative tests for the auditable Japan static-survey route."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "apps" / "commands"))
sys.path.insert(0, str(ROOT_DIR / "apps" / "commands" / "benchmarks"))

import gnss_japan_static_survey as survey  # noqa: E402


class JapanStaticSurveyTest(unittest.TestCase):
    def test_fixed_source_contract_is_pinned_and_auditable(self) -> None:
        required = {
            "rover_crx_gz",
            "base_crx_gz",
            "nav_gz",
            "sp3_gz",
            "clk_gz",
            "truth_snx_gz",
            "rover_log",
            "base_log",
            "igs20_ssc",
            "antex",
            "rnxcmp_linux_x86_64",
        }
        self.assertTrue(required.issubset(survey.SOURCES))
        for name in required:
            source = survey.SOURCES[name]
            self.assertRegex(source["url"], r"^https://")
            self.assertRegex(source["sha256"], r"^[0-9a-f]{64}$")
            self.assertGreater(int(source["bytes"]), 0)

        self.assertEqual(
            survey.SOURCES["truth_snx_gz"]["sha256"],
            "f0d95d00ef5b1f789a52154992a5935a140e04e9ad77ae8224df3fa01aa9850f",
        )
        self.assertIn("coord/IGS20/IGS20.ssc", survey.SOURCES["igs20_ssc"]["url"])
        self.assertIn("RNXCMP_4.2.0_Linux_gcc_64bit", survey.SOURCES["rnxcmp_linux_x86_64"]["url"])

    def test_holdout_is_a_distinct_pinned_dataset(self) -> None:
        args = survey.parse_args(["--output-dir", "/tmp/japan-survey-test", "--dataset", "holdout"])
        self.assertEqual(args.dataset, "holdout")
        self.assertNotEqual(
            survey.HOLDOUT_SOURCES["rover_crx_gz"]["url"],
            survey.SOURCES["rover_crx_gz"]["url"],
        )
        self.assertIn("/002/", survey.HOLDOUT_SOURCES["nav_gz"]["url"])
        self.assertIn("20240020000", survey.HOLDOUT_SOURCES["sp3_gz"]["url"])

    def test_verify_source_fails_closed_for_missing_size_and_hash(self) -> None:
        with tempfile.TemporaryDirectory(prefix="japan_survey_source_") as temp_dir:
            root = Path(temp_dir)
            path = root / "fixture.bin"
            content = b"pinned fixture\n"
            path.write_bytes(content)
            source = {
                "url": "https://example.invalid/fixture.bin",
                "bytes": len(content),
                "sha256": hashlib.sha256(content).hexdigest(),
            }
            survey.verify_source(path, source)

            wrong_size = {**source, "bytes": len(content) + 1}
            with self.assertRaisesRegex(ValueError, "byte-size mismatch"):
                survey.verify_source(path, wrong_size)

            wrong_hash = {**source, "sha256": "0" * 64}
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                survey.verify_source(path, wrong_hash)

            with self.assertRaisesRegex(ValueError, "missing cached source"):
                survey.verify_source(root / "missing.bin", source)

    def test_fetch_source_is_atomic_and_reuses_hash_verified_cache(self) -> None:
        with tempfile.TemporaryDirectory(prefix="japan_survey_fetch_") as temp_dir:
            root = Path(temp_dir)
            payload = b"cached source\n"
            local = root / "source.bin"
            local.write_bytes(payload)
            source = {
                "url": local.as_uri(),
                "bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
            }
            cache = root / "cache"
            first = survey.fetch_source(cache, source, offline=False, force=False, timeout=2.0)
            self.assertEqual(first.read_bytes(), payload)
            second = survey.fetch_source(cache, source, offline=True, force=False, timeout=2.0)
            self.assertEqual(second, first)

            first.write_bytes(b"tampered")
            with self.assertRaisesRegex(ValueError, "(byte-size mismatch|SHA-256 mismatch)"):
                survey.fetch_source(cache, source, offline=True, force=False, timeout=2.0)

    def test_rinex_antenna_type_is_required(self) -> None:
        with tempfile.TemporaryDirectory(prefix="japan_survey_rinex_") as temp_dir:
            root = Path(temp_dir)
            valid = root / "valid.rnx"
            valid.write_text(
                "".join(
                    (
                        "                    TRM159900.00    NONE                ANT # / TYPE\n",
                        "                                                            END OF HEADER\n",
                    )
                ),
                encoding="ascii",
            )
            self.assertEqual(survey.rinex_antenna_type(valid), "TRM159900.00    NONE")

            missing = root / "missing.rnx"
            missing.write_text("                                                            END OF HEADER\n", encoding="ascii")
            with self.assertRaisesRegex(ValueError, "antenna type is missing"):
                survey.rinex_antenna_type(missing)

    def test_crx_converter_failure_does_not_silently_materialize(self) -> None:
        with tempfile.TemporaryDirectory(prefix="japan_survey_crx_") as temp_dir:
            root = Path(temp_dir)
            source = root / "observation.crx.gz"
            # A valid gzip payload still must not be accepted when the converter fails.
            import gzip

            source.write_bytes(gzip.compress(b"not a compact RINEX file\n"))
            output = root / "observation.rnx"
            with self.assertRaises((FileNotFoundError, ValueError)):
                survey.convert_crx(source, output, root / "missing-CRX2RNX")
            self.assertFalse(output.exists())

    def test_crx_conversion_removes_stale_output_before_noninteractive_replay(self) -> None:
        with tempfile.TemporaryDirectory(prefix="japan_survey_replay_") as temp_dir:
            root = Path(temp_dir)
            import gzip

            source = root / "observation.crx.gz"
            source.write_bytes(gzip.compress(b"compact payload\n"))
            output = root / "observation.rnx"
            output.write_text("stale\n", encoding="ascii")
            converter = root / "converter"
            converter.write_text(
                "#!/bin/sh\nprintf 'fresh\\n' > \"${1%.crx}.rnx\"\n",
                encoding="ascii",
            )
            converter.chmod(0o755)
            survey.convert_crx(source, output, converter)
            self.assertEqual(output.read_text(encoding="ascii"), "fresh\n")

    def test_child_commands_carry_independent_truth_and_antenna_contract(self) -> None:
        with tempfile.TemporaryDirectory(prefix="japan_survey_commands_") as temp_dir:
            root = Path(temp_dir)
            paths = {
                "rover_obs": root / "rover.rnx",
                "base_obs": root / "base.rnx",
                "nav": root / "nav.rnx",
                "sp3": root / "final.sp3",
                "clk": root / "final.clk",
                "antex": root / "igs20.atx",
            }
            metadata = {
                "truth": {
                    "TSK2": {"ecef_m": [-3957184.9682679, 3310231.00877522, 3737703.81925572]},
                    "TSKB": {"ecef_m": [-3957200.08745802, 3310199.00772871, 3737711.49330339]},
                },
                "antennas": {"TSK2": "TRM159900.00"},
            }
            commands = survey.child_commands(paths, metadata, root / "out", "smoke")
            relative = commands["relative"]
            ppp = commands["ppp"]
            self.assertTrue(any(token.startswith("--truth-ecef=-3957184.968267") for token in relative))
            self.assertIn("--truth-source", relative)
            self.assertIn("--truth-frame", relative)
            self.assertTrue(any(token.startswith("--base-ecef=-3957200.087458") for token in relative))
            self.assertIn("--antex", ppp)
            self.assertIn(str(paths["sp3"]), ppp)
            self.assertIn("--receiver-antenna-type", ppp)
            self.assertIn("TRM159900.00", ppp)
            self.assertTrue(any(token.startswith("--truth-ecef=-3957184.968267") for token in ppp))
            self.assertEqual(commands["relative_png"][-1], str(root / "out" / "relative_static.pos"))

    def test_acquisition_failure_leaves_failed_manifest(self) -> None:
        with tempfile.TemporaryDirectory(prefix="japan_survey_manifest_") as temp_dir:
            root = Path(temp_dir)
            args = survey.parse_args(["--output-dir", str(root / "out"), "--offline"])
            with mock.patch.object(survey, "acquire", side_effect=ValueError("hash mismatch")):
                result = survey.run(args)
            self.assertEqual(result, 1)
            manifest = json.loads((root / "out" / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["status"], "failed")
            self.assertEqual(manifest["exit_status"], 1)
            self.assertIn("hash mismatch", manifest["failure_reason"])
            self.assertTrue((root / "out" / "bundle.log").is_file())

    def test_guide_records_truth_epoch_and_coordinate_boundaries(self) -> None:
        guide = (ROOT_DIR / "docs" / "use_cases" / "japan_static_survey.md").read_text(
            encoding="utf-8"
        )
        for token in (
            "japan-static-survey",
            "TSK200JPN",
            "TSKB00JPN",
            "SOLUTION/EPOCHS",
            "SOLUTION/ESTIMATE",
            "SOLUTION/APRIORI",
            "IGS20.ssc",
            "igs20.atx",
            "TRM159900.00",
            "AOAD/M_T",
            "ellipsoidal",
            "orthometric",
            "ARP",
            "PCO/PCV",
            "no `VELX`",
            "control-point",
            "best-effort",
            "RNXCMP",
        ):
            self.assertIn(token, guide)
        self.assertIn("use_cases/japan_static_survey.md", (ROOT_DIR / "docs/index.md").read_text(encoding="utf-8"))
        self.assertIn("use_cases/japan_static_survey.md", (ROOT_DIR / "docs/use_cases.md").read_text(encoding="utf-8"))
        self.assertIn("use_cases/japan_static_survey.md", (ROOT_DIR / "mkdocs.yml").read_text(encoding="utf-8"))

    def test_native_ecef_cli_parsing_is_sequenced(self) -> None:
        source = (ROOT_DIR / "apps" / "native" / "gnss_solve.cpp").read_text(encoding="utf-8")
        # Multiple argv[++i] expressions in one Eigen constructor have an
        # unspecified evaluation order in C++; the real TSKB run caught this.
        self.assertIn("const double base_x = std::stod(argv[++i]);", source)
        self.assertIn("const double base_y = std::stod(argv[++i]);", source)
        self.assertIn("const double base_z = std::stod(argv[++i]);", source)
        self.assertIn("config.base_position_ecef = Eigen::Vector3d(base_x, base_y, base_z);", source)


if __name__ == "__main__":
    unittest.main()
