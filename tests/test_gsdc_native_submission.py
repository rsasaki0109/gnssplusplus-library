import csv
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts" / "analysis"))
from assemble_gsdc_native_submission import assemble, select_rows
from audit_gsdc_native_outputs import digest


class NativeSubmissionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        raw = self.root / "raw.csv"
        raw.write_text("MessageType,utcTimeMillis\nRaw,1000\nRaw,2000\n")
        binary = self.root / "native.exe"
        binary.write_bytes(b"synthetic test executable identity")
        other = []
        for name in ("imu.csv", "nav", "base.obs"):
            p = self.root / name
            p.write_text("synthetic input")
            other.append(p)
        runs = {}
        for i in range(40):
            case = f"route{i:02d}/pixel5"
            folder = self.root / f"run{i:02d}"
            folder.mkdir()
            argv = [str(binary), "--dataset-id", case, "--android-gnss", str(raw),
                    "--android-imu", str(other[0]), "--nav", str(other[1]),
                    "--native-base-rinex", str(other[2]), "--android-raw-clock-only",
                    "--native-phase171-raw-p-no-doppler-imu-main"]
            summary = {"status": "imu-combined-factor", "truth_used": False,
                       "android_gnss_diagnostics": {"no_device_wls_seed": True},
                       "graph": {"converged": True},
                       "raw_utc_key_contract": {
                           "warmup_epoch_excluded": False, "raw_epoch_keys": 2,
                           "target_epochs": 2, "exact_solution_epochs": 2,
                           "interpolated_epochs": 0, "edge_hold_epochs": 0,
                           "unresolved_epochs": 0, "device_wls_coordinates_used": False}}
            (folder / "summary.json").write_text(json.dumps(summary))
            (folder / "solution.csv").write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                f"{case},1000,37,-122\n{case},2000,38,-121\n")
            entry = {"folder": str(folder), "argv": argv,
                     "inputs": [{"path": str(p), "sha256": digest(p)} for p in [raw, *other]]}
            run = {**entry, "state": "complete", "returncode": 0, "wall_s": 1,
                   "binary_sha256": digest(binary),
                   "outputs": {n: digest(folder / n) for n in ("solution.csv", "summary.json")}}
            (folder / "run.json").write_text(json.dumps(run))
            runs[case] = entry
        self.plan = self.root / "plan.json"
        self.plan.write_text(json.dumps({"runs": runs, "binary_sha256": digest(binary)}))
        self.plan_hash = digest(self.plan)
        self.sample = self.root / "sample.csv"
        # Official order differs from plan order; sample coordinates are unrelated.
        text = "tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
        text += "".join(f"{case},2000,0,0\n" for case in reversed(runs))
        self.sample.write_bytes(text.encode())
        self.sample_hash = hashlib.sha256(text.encode()).hexdigest()
        self.output = self.root / "submission"

    def assemble(self):
        return assemble(self.plan, self.plan_hash, self.sample, self.sample_hash, self.output)

    def test_complete_audited_export_uses_native_coordinates_and_official_order(self):
        proof = self.assemble()
        with (self.output / "submission.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 40)
        self.assertEqual(rows[0]["tripId"], "route39/pixel5")
        self.assertEqual(rows[-1]["tripId"], "route00/pixel5")
        self.assertTrue(all(r["LatitudeDegrees"] == "38" and r["LongitudeDegrees"] == "-121" for r in rows))
        self.assertEqual(proof["dropped_nonofficial_rows"], 40)
        self.assertFalse(proof["submitted"])
        self.assertFalse(proof["sample_coordinates_used"])

    def test_pending_drive_writes_no_partial_submission(self):
        p = self.root / "run00/run.json"
        r = json.loads(p.read_text())
        r["state"] = "running"
        p.write_text(json.dumps(r))
        with self.assertRaisesRegex(ValueError, "incomplete or failed"):
            self.assemble()
        self.assertFalse(self.output.exists())

    def test_complete_keys_do_not_admit_device_wls(self):
        folder = self.root / "run00"
        summary = json.loads((folder / "summary.json").read_text())
        summary["android_gnss_diagnostics"]["no_device_wls_seed"] = False
        (folder / "summary.json").write_text(json.dumps(summary))
        run = json.loads((folder / "run.json").read_text())
        run["outputs"]["summary.json"] = digest(folder / "summary.json")
        (folder / "run.json").write_text(json.dumps(run))
        with self.assertRaisesRegex(ValueError, "native official-key audit failed"):
            self.assemble()
        self.assertFalse(self.output.exists())

    def test_changed_input_fails_before_export(self):
        (self.root / "imu.csv").write_text("changed input")
        with self.assertRaisesRegex(ValueError, "input hash mismatch"):
            self.assemble()
        self.assertFalse(self.output.exists())

    def test_mixed_executable_record_is_rejected(self):
        p = self.root / "run00/run.json"
        r = json.loads(p.read_text())
        r["binary_sha256"] = "different executable"
        p.write_text(json.dumps(r))
        with self.assertRaisesRegex(ValueError, "fixed executable/arguments mismatch"):
            self.assemble()

    def test_changed_plan_and_existing_export_are_rejected(self):
        with self.plan.open("a") as stream:
            stream.write(" ")
        with self.assertRaisesRegex(ValueError, "plan hash mismatch"):
            self.assemble()
        self.output.mkdir()
        with self.assertRaisesRegex(ValueError, "already exists"):
            self.assemble()

    def test_selector_does_not_fill_missing_or_duplicate_native_keys(self):
        row = {"phone": "drive", "UnixTimeMillis": "1", "LatitudeDegrees": "1", "LongitudeDegrees": "2"}
        with self.assertRaisesRegex(ValueError, "missing native official keys"):
            select_rows({"drive": [row]}, [("drive", 2)])
        with self.assertRaisesRegex(ValueError, "duplicate"):
            select_rows({"drive": [row, row]}, [("drive", 1)])


if __name__ == "__main__":
    unittest.main()
