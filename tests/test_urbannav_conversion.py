"""Synthetic-input tests for scripts/convert_urbannav_to_ppc_layout.py (holdout contract v1)."""
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
import convert_urbannav_to_ppc_layout as conv

RINEX_HEADER = (b"     3.02           OBSERVATION DATA    M: Mixed            RINEX VERSION / TYPE\r\n"
                b"  2018    12    19     3    56   15.0000000     GPS         TIME OF FIRST OBS   \r\n"
                b"                                                            END OF HEADER       \r\n")
IMU_RAW_HEADER = ("GPS TOW (s), GPS Week, Acceleration X (m/s^2), Acceleration Y (m/s^2), Acceleration Z (m/s^2), "
                  "Angular rate X (rad/s), Angular rate Y (rad/s), Angular rate Z (rad/s), Wheel velocity (m/s)\n")
REF_RAW_HEADER = ("GPS TOW (s), GPS Week, Latitude (deg), Longitude (deg), Ellipsoid Height (m), ECEF X (m), ECEF Y (m), "
                  "ECEF Z (m), Roll (deg), Pitch (deg), Heading (deg), Velocity X (m/s), Velocity Y (m/s), Velocity Z (m/s), "
                  "Acceleration X (m/s^2), Acceleration Y (m/s^2), Acceleration Z (m/s^2), Angular rate X (rad/s), "
                  "Angular rate Y (rad/s), Angular rate Z (rad/s)\n")
SECONDS = ("15.0000000", "15.1000000", "15.2000000", "15.2000001", "15.3000000", "15.4000000", "15.6000000", "16.2000000")


def epoch(second, nsat=2):
    # CRLF records with trailing spaces, as in the real receiver files.
    lines = [f"> 2018 12 19  3 56 {second}  0 {nsat:2d}                     \r\n".encode()]
    lines += [f"G{n:2d}  23338470.203   122644569.975       -3852.309          44.250    \r\n".encode() for n in range(nsat)]
    return b"".join(lines)


def imu_line(tow, ax=0.0, ay=0.0, az=0.0, gx=0.0, gy=0.0, gz=0.0, wheel=0.0):
    return f"{tow}, 2032, {ax:.8f}, {ay:.8f}, {az:.8f}, {gx:.11f}, {gy:.11f}, {gz:.11f}, {wheel:.6f}\n"


def ref_line(tow, east_raw):
    return (f"{tow}, 2032, 35.62931853, 139.78712595, 44.6995, -3963426.7981, 3350882.1576, 3694865.5458, "
            f"-1.511332, 1.714764, 326.649944, {east_raw}, -0.000000, 0.000000, -0.17517300, -0.04017600, "
            "-0.12864900, 0.00182900000, 0.00099200000, 0.00037100000\n")


class TimeHelpersTest(unittest.TestCase):
    def test_calendar_to_gps_tow_matches_known_week(self):
        # Matches the UrbanNav Odaiba header and IMU: 2018-12-19 03:56:15 GPS is week 2032, TOW 273375.
        self.assertEqual(conv.gps_tow(2018, 12, 19, 3, 56, "15.0000000"), (2032, 273375))

    def test_multiples_of_two_tenths_use_exact_arithmetic(self):
        # In binary floating point 273375.6 % 0.2 is not zero; exact fractions must still accept it.
        for text in ("273375.6", "273375.60", "273375.2", "273376.0"):
            self.assertTrue(conv.is_multiple(conv.Fraction(text), conv.EPOCH_STEP), text)
        for text in ("273375.1", "273375.2000001", "273375.19"):
            self.assertFalse(conv.is_multiple(conv.Fraction(text), conv.EPOCH_STEP), text)

    def test_microseconds_are_exact_and_reject_finer_resolution(self):
        self.assertEqual(conv.micros("273375.009338"), 273375009338)
        with self.assertRaises(ValueError):
            conv.micros("273375.0093381")


class ConverterCase(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, name, data):
        path = self.root/name
        path.write_bytes(data if isinstance(data, bytes) else data.encode())
        return path


class RoverDecimationTest(ConverterCase):
    def test_only_two_tenth_second_epochs_survive_and_bytes_are_preserved(self):
        source = self.write("in.obs", RINEX_HEADER + b"".join(epoch(s, nsat=1 + i % 3) for i, s in enumerate(SECONDS)))
        stats = conv.decimate_rinex_obs(source, self.root/"out.obs")
        kept = ("15.0000000", "15.2000000", "15.4000000", "15.6000000", "16.2000000")
        expected = RINEX_HEADER + b"".join(epoch(s, nsat=1 + SECONDS.index(s) % 3) for s in kept)
        self.assertEqual((self.root/"out.obs").read_bytes(), expected)
        self.assertEqual((stats["epochs_in"], stats["epochs_out"]), (len(SECONDS), len(kept)))
        self.assertEqual((stats["first_tow_out"], stats["last_tow_out"]), (273375.0, 273376.2))

    def test_epochs_outside_the_truth_span_are_dropped_with_inclusive_bounds(self):
        source = self.write("in.obs", RINEX_HEADER + b"".join(epoch(s) for s in SECONDS))
        span = (conv.Fraction("273375.2"), conv.Fraction("273375.6"))
        stats = conv.decimate_rinex_obs(source, self.root/"out.obs", span)
        # 15.0 precedes the first truth row and 16.2 follows the last; the bounds themselves are kept.
        expected = RINEX_HEADER + b"".join(epoch(s) for s in ("15.2000000", "15.4000000", "15.6000000"))
        self.assertEqual((self.root/"out.obs").read_bytes(), expected)
        self.assertEqual((stats["epochs_out"], stats["epochs_dropped_outside_truth_span"]), (3, 2))
        self.assertEqual((stats["first_tow_out"], stats["last_tow_out"]), (273375.2, 273375.6))
        self.assertEqual(conv.decimate_rinex_obs(source, self.root/"all.obs")["epochs_dropped_outside_truth_span"], 0)

    def test_non_gps_time_system_and_backward_epochs_are_rejected(self):
        utc = RINEX_HEADER.replace(b"GPS         TIME", b"UTC         TIME")
        with self.assertRaisesRegex(ValueError, "GPS time system"):
            conv.decimate_rinex_obs(self.write("utc.obs", utc + epoch("15.0000000")), self.root/"o1")
        with self.assertRaisesRegex(ValueError, "non-monotonic"):
            conv.decimate_rinex_obs(self.write("back.obs", RINEX_HEADER + epoch("15.2000000") + epoch("15.0000000")),
                                    self.root/"o2")


class ImuTest(ConverterCase):
    def run_imu(self, rows):
        source = self.write("imu_in.csv", IMU_RAW_HEADER + "".join(rows))
        stats = conv.convert_imu(source, self.root/"imu_out.csv")
        return (self.root/"imu_out.csv").read_text().splitlines(), stats

    def test_ppc_header_axes_units_and_linear_interpolation(self):
        rad = math.radians(1.0)
        lines, stats = self.run_imu([
            imu_line("273375.01", ax=1.0, ay=2.0, az=-9.0, gx=rad, gy=2*rad, gz=3*rad, wheel=5.0),
            imu_line("273375.03", ax=3.0, ay=4.0, az=-11.0, gx=3*rad, gy=4*rad, gz=5*rad, wheel=5.0)])
        self.assertEqual(lines[0], conv.PPC_IMU_HEADER.rstrip("\n"))
        self.assertEqual(len(lines), 2)
        self.assertEqual(stats["grid_samples"], 1)
        fields = [f.strip() for f in lines[1].split(",")]
        self.assertEqual(fields[:2], ["273375.02", "2032"])
        self.assertEqual(len(fields), 8)  # wheel speed dropped
        # Midpoint of FRD (1,2,-9)/(1,2,3 deg/s) and (3,4,-11)/(3,4,5 deg/s) mapped to FLU: y and z negated.
        ax, ay, az, gx, gy, gz = (float(v) for v in fields[2:])
        for got, want in zip((ax, ay, az, gx, gy, gz), (2.0, -3.0, 10.0, 2.0, -3.0, -4.0)):
            self.assertAlmostEqual(got, want, places=7)
        self.assertEqual(lines[1], "273375.02, 2032,  2.00000000, -3.00000000, 10.00000000,  2.00000000, -3.00000000, -4.00000000")

    def test_grid_is_inside_pairs_and_gaps_over_a_tenth_second_are_skipped(self):
        lines, stats = self.run_imu([
            imu_line("100.01"), imu_line("100.03"), imu_line("100.05"),
            imu_line("100.20"),  # 0.15 s after the previous sample: nothing is written inside or at the end of the pair
            imu_line("100.21"), imu_line("100.23")])
        self.assertEqual([line.split(",")[0] for line in lines[1:]], ["100.02", "100.04", "100.22"])
        self.assertEqual(stats["grid_points_skipped_gap"], 8)       # 100.06 ... 100.20
        self.assertEqual(stats["grid_points_skipped_on_raw_sample"], 0)
        self.assertEqual(stats["pairs_over_gap"], 1)
        self.assertEqual(stats["grid_points_in_raw_span"], 11)
        self.assertEqual(stats["grid_points_skipped"], 8)

    def test_grid_point_on_a_raw_sample_takes_the_raw_value(self):
        lines, stats = self.run_imu([imu_line("100.00", ax=7.0), imu_line("100.01", ax=1.0),
                                     imu_line("100.02", ax=3.0), imu_line("100.03", ax=5.0)])
        self.assertEqual([line.split(",")[0] for line in lines[1:]], ["100.02"])
        self.assertAlmostEqual(float(lines[1].split(",")[2]), 3.0, places=7)
        self.assertEqual(stats["grid_points_skipped_on_raw_sample"], 1)  # 100.00: first sample, no pair ends on it
        self.assertEqual(stats["grid_points_skipped"], 1)

    def test_exactly_one_tenth_second_pair_is_bridged_and_no_extrapolation_happens(self):
        lines, _ = self.run_imu([imu_line("100.01", ax=0.0), imu_line("100.11", ax=10.0)])
        times = [line.split(",")[0] for line in lines[1:]]
        self.assertEqual(times, ["100.02", "100.04", "100.06", "100.08", "100.10"])  # nothing before 100.01 or after 100.11
        self.assertAlmostEqual(float(lines[3].split(",")[2]), 5.0, places=7)

    def test_unchanged_time_negative_zero_and_week_changes(self):
        lines, _ = self.run_imu([imu_line("100.01", ay=0.0), imu_line("100.03", ay=0.0)])
        self.assertNotIn("-0.00000000", lines[1])  # negated zero is written as plain zero
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            self.run_imu([imu_line("100.03"), imu_line("100.03")])
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            self.run_imu([imu_line("100.03"), imu_line("100.01")])
        with self.assertRaisesRegex(ValueError, "week"):
            self.run_imu([imu_line("100.01"), imu_line("100.03").replace("2032", "2033")])

    def test_unexpected_header_is_rejected(self):
        source = self.write("bad.csv", IMU_RAW_HEADER.replace("rad/s", "deg/s") + imu_line("100.01"))
        with self.assertRaisesRegex(ValueError, "header"):
            conv.convert_imu(source, self.root/"o.csv")


class ReferenceTest(ConverterCase):
    def test_velocity_columns_renamed_rows_filtered_and_values_untouched(self):
        raw = [ref_line(f"273375.{i:02d}", f"0.00{i % 10}000") for i in (10, 20, 30, 40, 60, 80)]
        stats = conv.convert_reference(self.write("ref_in.csv", REF_RAW_HEADER + "".join(raw)), self.root/"ref_out.csv")
        out = (self.root/"ref_out.csv").read_bytes().decode().split("\n")
        header = out[0].split(",")
        self.assertEqual(tuple(header[:14]), conv.PPC_REFERENCE_HEADER)
        self.assertEqual(header[11:14], ["East Velocity (m/s)", "North Velocity (m/s)", "Up Velocity (m/s)"])
        self.assertEqual(header[14:], ["Acceleration X (m/s^2)", "Acceleration Y (m/s^2)", "Acceleration Z (m/s^2)",
                                       "Angular rate X (rad/s)", "Angular rate Y (rad/s)", "Angular rate Z (rad/s)"])
        self.assertEqual(out[1:-1], [raw[1].rstrip("\n"), raw[3].rstrip("\n"), raw[4].rstrip("\n"), raw[5].rstrip("\n")])
        self.assertEqual((stats["rows_in"], stats["rows_out"]), (6, 4))
        self.assertEqual((stats["first_tow_out"], stats["last_tow_out"]), (273375.2, 273375.8))

    def test_header_must_match_and_time_must_increase(self):
        with self.assertRaisesRegex(ValueError, "header"):
            conv.convert_reference(self.write("h.csv", REF_RAW_HEADER.replace("Heading", "Yaw") + ref_line("1.20", "0")),
                                   self.root/"o1.csv")
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            conv.convert_reference(self.write("b.csv", REF_RAW_HEADER + ref_line("1.20", "0") + ref_line("1.00", "0")),
                                   self.root/"o2.csv")


class EndToEndTest(ConverterCase):
    def make_raw(self):
        raw = self.root/"raw"
        raw.mkdir()
        body = RINEX_HEADER + b"".join(epoch(s) for s in SECONDS)
        (raw/"Odaiba_rover_trimble.obs").write_bytes(body)
        (raw/"Odaiba_base_trimble.obs").write_bytes(RINEX_HEADER + epoch("15.0000000"))
        (raw/"Odaiba_base.nav").write_bytes(b"nav bytes\r\n")
        (raw/"Odaiba_imu.csv").write_text(IMU_RAW_HEADER + imu_line("273375.01") + imu_line("273375.03") + imu_line("273375.05"))
        (raw/"Odaiba_reference.csv").write_text(
            REF_RAW_HEADER + "".join(ref_line(t, "1.000000") for t in ("273375.20", "273375.30", "273375.40", "273375.60", "273375.80")))
        return raw

    def test_trimble_rover_is_cut_to_the_truth_span_and_manifest_hashes_every_file(self):
        raw = self.make_raw()
        out = self.root/"out"
        manifest = conv.convert(raw, "Odaiba", "trimble", out)
        directory = out/"urbannav/Odaiba_trimble"
        self.assertEqual((directory/"rover.obs").read_bytes().count(b">"), 3)  # 15.2, 15.4, 15.6 of 0.2 s multiples 15.0..16.2
        self.assertEqual(manifest["stats"]["rover.obs"]["epochs_dropped_outside_truth_span"], 2)
        self.assertEqual(conv.reference_span(directory/"reference.csv"), (conv.Fraction("273375.2"), conv.Fraction("273375.8")))
        self.assertEqual((directory/"base.obs").read_bytes(), (raw/"Odaiba_base_trimble.obs").read_bytes())
        self.assertEqual((directory/"base.nav").read_bytes(), b"nav bytes\r\n")
        self.assertEqual(sorted(p.name for p in directory.iterdir()),
                         ["base.nav", "base.obs", "imu.csv", "reference.csv", "rover.obs"])
        self.assertEqual(set(manifest["raw_inputs"]), set(manifest["outputs"]))
        for name, pin in manifest["outputs"].items():
            self.assertEqual(pin["sha256"], hashlib.sha256((directory/name).read_bytes()).hexdigest())
        on_disk = json.loads((out/"urbannav/Odaiba_trimble.manifest.json").read_text())
        self.assertEqual(on_disk["raw_inputs"], manifest["raw_inputs"])
        self.assertEqual(manifest["raw_inputs"]["rover.obs"]["sha256"],
                         hashlib.sha256((raw/"Odaiba_rover_trimble.obs").read_bytes()).hexdigest())

    def test_ublox_is_not_supported(self):
        with self.assertRaisesRegex(ValueError, "rover"):
            conv.convert(self.make_raw(), "Odaiba", "ublox", self.root/"out")

    def test_cli_refuses_to_overwrite_and_reports_missing_inputs(self):
        raw = self.make_raw()
        argv = ["--raw-dir", str(raw), "--run", "Odaiba", "--rover", "trimble", "--output-root", str(self.root/"o")]
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(conv.main(argv), 0)
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(conv.main(argv), 2)
            (raw/"Odaiba_base.nav").unlink()
            argv[-1] = str(self.root/"o2")
            self.assertEqual(conv.main(argv), 2)


if __name__ == "__main__":
    unittest.main()
