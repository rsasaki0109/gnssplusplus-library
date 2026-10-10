"""Synthetic-input tests for scripts/convert_urbannav_hk_to_ppc_layout.py (holdout contract v2)."""
import contextlib
import gzip
import hashlib
import io
import json
import math
import re
from fractions import Fraction
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"scripts"))
sys.path.insert(0, str(ROOT/"apps/commands/benchmarks"))
sys.path.insert(0, str(ROOT/"scripts/analysis"))
import check_urbannav_hk_frames as frames
import convert_urbannav_hk_to_ppc_layout as conv
import gnss_pva_metrics as metrics

WEEK = 2158
IMU_HEADER = ("%time,field.header.seq,field.header.stamp,field.header.frame_id,field.orientation.x,field.orientation.y,"
              "field.orientation.z,field.orientation.w,field.orientation_covariance0,field.orientation_covariance1,"
              "field.orientation_covariance2,field.orientation_covariance3,field.orientation_covariance4,"
              "field.orientation_covariance5,field.orientation_covariance6,field.orientation_covariance7,"
              "field.orientation_covariance8,field.angular_velocity.x,field.angular_velocity.y,field.angular_velocity.z,"
              "field.angular_velocity_covariance0,field.angular_velocity_covariance1,field.angular_velocity_covariance2,"
              "field.angular_velocity_covariance3,field.angular_velocity_covariance4,field.angular_velocity_covariance5,"
              "field.angular_velocity_covariance6,field.angular_velocity_covariance7,field.angular_velocity_covariance8,"
              "field.linear_acceleration.x,field.linear_acceleration.y,field.linear_acceleration.z,"
              "field.linear_acceleration_covariance0,field.linear_acceleration_covariance1,"
              "field.linear_acceleration_covariance2,field.linear_acceleration_covariance3,"
              "field.linear_acceleration_covariance4,field.linear_acceleration_covariance5,"
              "field.linear_acceleration_covariance6,field.linear_acceleration_covariance7,"
              "field.linear_acceleration_covariance8\n")
TRUTH_HEADER = ("      UTCTime       Week   GPSTime         Latitude        Longitude        H-Ell VelBdyX VelBdyY VelBdyZ "
                "AccBdyX AccBdyY AccBdyZ           Roll          Pitch        Heading Q\n"
                "        (sec)    (weeks)     (sec)       (+/-D M S)       (+/-D M S)          (m)   (m/s)   (m/s)   (m/s) "
                "(m/s^2) (m/s^2) (m/s^2)          (deg)          (deg)          (deg)  \n")
OBS_HEADER = (b"     3.03           OBSERVATION DATA    M: Mixed            RINEX VERSION / TYPE\r\n"
              b" -2419198.4545  5385470.4273  2405391.4254                  APPROX POSITION XYZ \r\n"
              b"  2021     5    21     6    29    6.0000000     GPS         TIME OF FIRST OBS   \r\n"
              b"                                                            END OF HEADER       \r\n")
BASE_HEADER = (b"     3.02           OBSERVATION DATA    M: MIXED            RINEX VERSION / TYPE\n"
               b" -2414266.9197  5386768.9868  2407460.0314                  APPROX POSITION XYZ\n"
               b"  2021    05    21    06    00    0.0000000     GPS         TIME OF FIRST OBS\n"
               b"                                                            END OF HEADER\n")


def calendar(tow):
    """(year, month, day, hour, minute, second) of GPS week 2158 time of week `tow` (2021-05-16 is its Sunday)."""
    days, rest = divmod(int(tow), 86400)
    return 2021, 5, 16 + days, rest//3600, rest % 3600//60, rest % 60 + (tow - int(tow))


def epoch_line(tow, nsat=2, crlf=True, flag=0):
    y, mo, d, h, mi, s = calendar(tow)
    text = f"> {y} {mo:2d} {d:2d} {h:2d} {mi:2d} {float(s):10.7f}  {flag} {nsat:2d}\n"
    return (text.replace("\n", "\r\n") if crlf else text).encode()


def obs_epoch(tow, nsat=2, crlf=True):
    end = b"\r\n" if crlf else b"\n"
    return epoch_line(tow, nsat, crlf) + b"".join(
        f"G{n:2d}  23338470.203 6 122644569.975 6       -3852.309          44.250".encode() + end for n in range(1, nsat+1))


def base_event(lines=3):
    return f">                              4 {lines:2d}\n".encode() + b"".join(f"header information {n}\n".encode() for n in range(lines))


def truth_line(tow, lat="22 18 05.61075", lon="114 11 25.11071", height="2.894", vel=(0.0, 0.0, 0.0), rpy=("-0.2", "-0.6", "-59.36"), q=1):
    utc = WEEK*604800 + tow + 315964800 - conv.LEAP_SECONDS
    return (f"{utc:.2f} {WEEK}.00000 {tow:.2f}   {lat}  {lon}  {height}  {vel[0]:.3f} {vel[1]:.3f} {vel[2]:.3f}  0.1 0.2 0.3  "
            f"{rpy[0]} {rpy[1]} {rpy[2]} {q}\n")


def stamp_ns(tow):
    """Unix-UTC header.stamp (ns) of GPS time of week `tow` (a Fraction or int, seconds)."""
    return int((Fraction(WEEK*604800) + Fraction(tow) + 315964800 - conv.LEAP_SECONDS)*10**9)


def imu_line(tow, acc=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0), frame="/imu"):
    fields = ["0"]*41
    fields[1], fields[2], fields[3] = "1", str(stamp_ns(tow)), frame
    fields[17:20] = (f"{v:.12f}" for v in gyro)
    fields[29:32] = (f"{v:.12f}" for v in acc)
    return ",".join(fields)+"\n"


def nav_file(system, version="3.02", records=1):
    names = dict(G="GPS", R="GLONASS", E="GALILEO", C="BEIDOU")
    head = [f"{version:>9}{'':11}{'N: GNSS NAV DATA':<20}{system + ': ' + names[system]:<20}RINEX VERSION / TYPE\n",
            f"{'Spider test':<20}{'SMO':<20}{'20210522 000850 UTC':<20}PGM / RUN BY / DATE\n"]
    if system == "G":
        head.append(f"{'GPSA   6.5193D-09  2.2352D-08 -5.9605D-08 -1.1921D-07':<60}IONOSPHERIC CORR\n")
    if system == "C":
        head.append(f"{'BDSA   1.0245D-08  5.9605D-08 -5.3644D-07  8.3447D-07':<60}IONOSPHERIC CORR\n")
    head.append(f"{'    18    18  1929     7':<60}LEAP SECONDS\n")
    head.append(f"{'':<60}END OF HEADER\n")
    body = []
    for n in range(1, records+1):
        body.append(f"{system}{n:02d} 2021 05 20 08 00 00 6.838440895081D-04-1.114131009672D-11 0.000000000000D+00\r\n")
        body += ["     9.100000000000D+01 3.115625000000D+01 3.991237562673D-09 1.829370330354D+00\r\n"]*3
    return "".join(head+body).encode()


class Case(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, name, data):
        path = self.root/name
        path.write_bytes(data if isinstance(data, bytes) else data.encode())
        return path


class GeometryTest(unittest.TestCase):
    def test_dms_is_exact_and_signed_by_the_degree_token(self):
        self.assertEqual(conv.dms_to_degrees("22", "30", "0"), Fraction(45, 2))
        self.assertEqual(conv.dms_to_degrees("-22", "30", "0"), -Fraction(45, 2))
        self.assertEqual(conv.dms_to_degrees("-0", "30", "0"), -Fraction(1, 2))  # south/west with zero degrees
        self.assertEqual(conv.dms_to_degrees("114", "11", "25.11071"), Fraction(114) + Fraction(11, 60) + Fraction("25.11071")/3600)

    def test_geodetic_to_ecef(self):
        polar = 6378137.0*math.sqrt(1-0.00669437999014)  # WGS84 semi-minor axis
        for lat, lon, h, want in ((0, 0, 0, (6378137.0, 0, 0)), (0, 90, 10, (0, 6378147.0, 0)), (90, 0, 0, (0, 0, polar))):
            got = conv.geodetic_to_ecef(lat, lon, h)
            for a, b in zip(got, want): self.assertAlmostEqual(a, b, places=3)

    def test_body_velocity_is_x_right_y_forward_z_up(self):
        e, n, u = conv.body_velocity_to_enu(0, 10, 0, 0, 0, 90)      # driving forward heading east
        for got, want in zip((e, n, u), (10, 0, 0)): self.assertAlmostEqual(got, want, places=9)
        for got, want in zip(conv.body_velocity_to_enu(0, 10, 0, 0, 0, 0), (0, 10, 0)): self.assertAlmostEqual(got, want, places=9)
        for got, want in zip(conv.body_velocity_to_enu(1, 0, 0, 0, 0, 0), (1, 0, 0)):   # +x is to the right: east when heading north
            self.assertAlmostEqual(got, want, places=9)
        for got, want in zip(conv.body_velocity_to_enu(0, 0, 1, 0, 0, 0), (0, 0, 1)):   # +z is up
            self.assertAlmostEqual(got, want, places=9)
        for got, want in zip(conv.body_velocity_to_enu(0, 10, 0, 0, 10, 0), (0, 10*math.cos(math.radians(10)), 10*math.sin(math.radians(10)))):
            self.assertAlmostEqual(got, want, places=9)                                   # nose up climbs

    def test_conversion_is_the_inverse_of_the_scorer_body_velocity(self):
        # gnss_pva_metrics labels reverse driving with body = R^T [N, E, -U] in FRD; forward must be the truth y axis.
        roll, pitch, heading = 3.0, -4.0, 123.0
        vx, vy, vz = 0.3, 7.0, -0.2
        e, n, u = conv.body_velocity_to_enu(vx, vy, vz, roll, pitch, heading)
        frd = metrics.rotate(metrics.transpose(metrics.euler_matrix(roll, pitch, heading)), [n, e, -u])
        for got, want in zip(frd, (vy, vx, -vz)): self.assertAlmostEqual(got, want, places=9)

    def test_time_of_week_text(self):
        self.assertEqual(conv.format_tow(Fraction(455346)), "455346.0")
        self.assertEqual(conv.format_tow(Fraction("455346.5")), "455346.5")


class TruthTest(Case):
    def test_rows_exact_times_and_leap_second_check(self):
        path = self.write("t.txt", TRUTH_HEADER + truth_line(100) + truth_line(101, lat="-0 30 0"))
        rows = conv.parse_truth(path)
        self.assertEqual([r["tow"] for r in rows], [100, 101])
        self.assertEqual(rows[1]["lat"], -Fraction(1, 2))
        self.assertEqual((rows[0]["week"], rows[0]["heading"], rows[0]["quality"], rows[0]["height"]), (WEEK, "-59.36", "1", "2.894"))
        bad_leap = TRUTH_HEADER + truth_line(100).replace(f"{WEEK*604800 + 100 + 315964800 - 18:.2f}", f"{WEEK*604800 + 100 + 315964800 - 17:.2f}")
        with self.assertRaisesRegex(ValueError, "leap seconds"):
            conv.parse_truth(self.write("leap.txt", bad_leap))

    def test_header_ordering_columns_and_week(self):
        with self.assertRaisesRegex(ValueError, "header"):
            conv.parse_truth(self.write("h.txt", TRUTH_HEADER.replace("Heading", "Yaw") + truth_line(100)))
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            conv.parse_truth(self.write("o.txt", TRUTH_HEADER + truth_line(101) + truth_line(100)))
        with self.assertRaisesRegex(ValueError, "fields"):
            conv.parse_truth(self.write("f.txt", TRUTH_HEADER + truth_line(100).rsplit(" ", 2)[0] + "\n"))
        with self.assertRaisesRegex(ValueError, "no rows"):
            conv.parse_truth(self.write("e.txt", TRUTH_HEADER))

    def test_reference_rows_only_at_kept_epochs_with_enu_velocity(self):
        rows = conv.parse_truth(self.write("t.txt", TRUTH_HEADER + truth_line(100, vel=(0.0, 10.0, 0.5), rpy=("0", "0", "90"), q=2)
                                           + truth_line(101) + truth_line(102, vel=(1.0, 0.0, 0.0), rpy=("0", "0", "0"), q=5)))
        stats = conv.write_reference(rows, {Fraction(100), Fraction(102)}, self.root/"ref.csv")
        lines = (self.root/"ref.csv").read_text().splitlines()
        self.assertEqual(lines[0].split(","), list(conv.PPC_REFERENCE_HEADER) + ["Truth Quality (Q)"])
        self.assertEqual(len(lines), 3)
        first, second = lines[1].split(","), lines[2].split(",")
        self.assertEqual(first[:2], ["100.0", str(WEEK)])
        self.assertAlmostEqual(float(first[2]), 22+18/60+5.61075/3600, places=9)   # decimal degrees from D M S
        self.assertEqual((first[4], first[8:11], first[-1]), ("2.894", ["0", "0", "90"], "2"))  # height, angles and Q are the raw text
        self.assertEqual([float(v) for v in first[11:14]], [10.0, 0.0, 0.5])    # forward y, heading east -> East
        self.assertEqual([float(v) for v in second[11:14]], [1.0, 0.0, 0.0])    # right x, heading north -> East
        self.assertEqual((stats["rows_in"], stats["rows_out"], stats["rows_dropped"], stats["quality_counts"]), (3, 2, 1, {"2": 1, "5": 1}))


class ImuTest(Case):
    def run_imu(self, rows, header=IMU_HEADER):
        source = self.write("imu_in.csv", header + "".join(rows))
        stats, grid = conv.convert_imu(source, self.root/"imu_out.csv")
        return (self.root/"imu_out.csv").read_text().splitlines(), stats, grid

    def test_ppc_header_rfu_to_flu_units_and_linear_interpolation(self):
        rad = math.radians(1.0)
        lines, stats, grid = self.run_imu([
            imu_line(Fraction("100.005"), acc=(1.0, 2.0, 9.0), gyro=(rad, 2*rad, 3*rad)),
            imu_line(Fraction("100.015"), acc=(3.0, 4.0, 11.0), gyro=(3*rad, 4*rad, 5*rad))])
        self.assertEqual(lines[0], conv.PPC_IMU_HEADER.rstrip("\n"))
        self.assertEqual(len(lines), 2)
        self.assertEqual((stats["grid_samples"], grid), (1, {10001}))
        fields = [f.strip() for f in lines[1].split(",")]
        self.assertEqual(fields[:2], ["100.01", str(WEEK)])
        self.assertEqual(len(fields), 8)
        # Midpoint raw (right, forward, up) acc (2, 3, 10) -> FLU (forward, left, up) = (3, -2, 10);
        # raw gyro (2, 3, 4) deg/s -> FLU (y, -x, z) = (3, -2, 4).
        for got, want in zip((float(v) for v in fields[2:]), (3.0, -2.0, 10.0, 3.0, -2.0, 4.0)): self.assertAlmostEqual(got, want, places=6)

    def test_hundred_hertz_grid_gap_rule_and_no_extrapolation(self):
        # 400 Hz raw samples with one 0.15 s hole: nothing is written inside or at the end of the hole.
        times = [Fraction(100) + Fraction(k, 400) for k in range(0, 9)]            # 100.0000 .. 100.0200
        times += [Fraction("100.17"), Fraction("100.1725"), Fraction("100.195"), Fraction("100.205")]
        lines, stats, grid = self.run_imu([imu_line(t) for t in times])
        self.assertEqual([line.split(",")[0] for line in lines[1:]], ["100.01", "100.02", "100.18", "100.19", "100.20"])
        self.assertEqual(stats["pairs_over_gap"], 1)
        self.assertEqual(stats["grid_points_skipped_gap"], 15)           # 100.03 .. 100.17 lie in or at the end of the hole
        self.assertEqual(stats["grid_points_skipped_on_raw_sample"], 1)  # 100.00 is the first raw sample
        self.assertEqual(stats["grid_points_skipped"], 16)
        self.assertEqual(stats["grid_points_in_raw_span"], 21)
        self.assertEqual(sorted(grid), [10001, 10002, 10018, 10019, 10020])

    def test_stamp_is_unix_utc_converted_to_gps_time_of_week(self):
        lines, stats, _ = self.run_imu([imu_line(Fraction("455346")-Fraction("0.01")), imu_line(Fraction("455346")+Fraction("0.01"))])
        self.assertEqual(lines[1].split(",")[:2], ["455346.00", f" {WEEK}"])
        self.assertEqual(stats["week_of_samples"], WEEK)

    def test_errors(self):
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            self.run_imu([imu_line(100), imu_line(100)])
        with self.assertRaisesRegex(ValueError, "frame_id"):
            self.run_imu([imu_line(100, frame="/other"), imu_line(101)])
        with self.assertRaisesRegex(ValueError, "column"):
            self.run_imu([imu_line(100)], header=IMU_HEADER.replace("field.angular_velocity.z", "field.angular_velocity.q"))
        with self.assertRaisesRegex(ValueError, "fewer than two"):
            self.run_imu([imu_line(100)])
        with self.assertRaisesRegex(ValueError, "fields"):
            self.run_imu([imu_line(100), imu_line(101).replace(",0,", ",", 1)])


class ObservationTest(Case):
    def test_rover_epochs_are_kept_by_set_and_bytes_are_preserved(self):
        epochs = [100, 101, 102, 104, 105]
        source = self.write("rover.obs", OBS_HEADER + b"".join(obs_epoch(t, nsat=1 + t % 3) for t in epochs))
        stats, kept = conv.filter_rover_obs(source, self.root/"o.obs", {Fraction(101), Fraction(104), Fraction(105)}, {Fraction(101), Fraction(102), Fraction(104), Fraction(105)})
        self.assertEqual((self.root/"o.obs").read_bytes(), OBS_HEADER + b"".join(obs_epoch(t, nsat=1 + t % 3) for t in (101, 104, 105)))
        self.assertEqual(kept, [101, 104, 105])
        self.assertEqual((stats["epochs_in"], stats["epochs_out"], stats["epochs_dropped"]), (5, 3, 2))
        self.assertEqual((stats["epochs_without_truth_row"], stats["epochs_with_truth_row_without_imu_sample"]), (1, 1))
        self.assertEqual((stats["first_tow_out"], stats["last_tow_out"], stats["max_gap_out_s"]), (101.0, 105.0, 3.0))

    def test_rover_rejections(self):
        utc = OBS_HEADER.replace(b"GPS         TIME", b"UTC         TIME")
        with self.assertRaisesRegex(ValueError, "GPS time system"):
            conv.filter_rover_obs(self.write("a.obs", utc + obs_epoch(100)), self.root/"o1", {Fraction(100)}, set())
        with self.assertRaisesRegex(ValueError, "non-monotonic"):
            conv.filter_rover_obs(self.write("b.obs", OBS_HEADER + obs_epoch(101) + obs_epoch(100)), self.root/"o2", set(), set())
        with self.assertRaisesRegex(ValueError, "not an observation epoch"):
            conv.filter_rover_obs(self.write("c.obs", OBS_HEADER + obs_epoch(100) + base_event()), self.root/"o3", set(), set())
        with self.assertRaisesRegex(ValueError, "event record"):
            conv.filter_rover_obs(self.write("d.obs", OBS_HEADER + epoch_line(100, flag=3) + obs_epoch(101)), self.root/"o4", set(), set())

    def test_base_keeps_the_epoch_records_inside_the_span_byte_for_byte(self):
        epochs = (360, 361, 362, 363)
        data = BASE_HEADER + b"".join(obs_epoch(t, crlf=False) for t in epochs) + base_event()
        for name, content in (("b.rnx", data), ("b.rnx.gz", gzip.compress(data))):
            path = self.write(name, content)
            dst = self.root/("out_"+name)
            stats, kept = conv.assemble_base([path], dst, (Fraction(361), Fraction(362)))
            self.assertEqual(dst.read_bytes(), BASE_HEADER + obs_epoch(361, crlf=False) + obs_epoch(362, crlf=False))
            self.assertEqual(kept, {361, 362})
            self.assertEqual((stats["epochs_in_files"], stats["epochs_out"], stats["epochs_dropped_before_span"],
                              stats["epochs_dropped_after_span"], stats["event_records_dropped"]), (4, 2, 1, 1, 1))
            self.assertEqual((stats["first_tow_out"], stats["last_tow_out"]), (361.0, 362.0))
            self.assertEqual(stats["approx_position_ecef"], [-2414266.9197, 5386768.9868, 2407460.0314])

    def test_a_span_that_holds_every_epoch_changes_nothing_but_the_closing_event(self):
        data = BASE_HEADER + b"".join(obs_epoch(t, crlf=False) for t in (360, 361, 363)) + base_event()
        dst = self.root/"all"
        conv.assemble_base([self.write("b.rnx", data)], dst, (Fraction(0), Fraction(10**6)))
        self.assertEqual(dst.read_bytes() + base_event(), data)

    def test_hour_files_are_concatenated_and_events_are_dropped(self):
        first = BASE_HEADER + obs_epoch(358, crlf=False) + obs_epoch(359, crlf=False) + base_event(3)
        second = BASE_HEADER + obs_epoch(360, crlf=False) + obs_epoch(361, crlf=False) + base_event(2)
        span = (Fraction(359), Fraction(360))
        stats, kept = conv.assemble_base([self.write("a", first), self.write("b", second)], self.root/"cat", span)
        self.assertEqual((self.root/"cat").read_bytes(), BASE_HEADER + obs_epoch(359, crlf=False) + obs_epoch(360, crlf=False))
        self.assertEqual((stats["files"], stats["epochs_in_files"], stats["epochs_out"], stats["event_records_dropped"]), (2, 4, 2, 2))
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            conv.assemble_base([self.write("c", second), self.write("d", first)], self.root/"bad", span)

    def test_an_event_record_in_the_middle_is_dropped_too(self):
        data = BASE_HEADER + obs_epoch(360, crlf=False) + base_event(2) + obs_epoch(361, crlf=False)
        stats, kept = conv.assemble_base([self.write("m", data)], self.root/"mid", (Fraction(0), Fraction(1000)))
        self.assertEqual((self.root/"mid").read_bytes(), BASE_HEADER + obs_epoch(360, crlf=False) + obs_epoch(361, crlf=False))
        self.assertEqual(stats["event_records_dropped"], 1)

    def test_base_rejections(self):
        span = (Fraction(0), Fraction(1000))
        with self.assertRaisesRegex(ValueError, "GPS time system"):
            conv.assemble_base([self.write("u", BASE_HEADER.replace(b"GPS         TIME", b"UTC         TIME") + obs_epoch(360))], self.root/"o", span)
        with self.assertRaisesRegex(ValueError, "APPROX POSITION"):
            conv.assemble_base([self.write("z", BASE_HEADER.replace(b"-2414266.9197  5386768.9868  2407460.0314", b"        0.0000        0.0000        0.0000") + obs_epoch(360))], self.root/"o", span)
        with self.assertRaisesRegex(ValueError, "observation file"):
            conv.assemble_base([self.write("n", nav_file("G"))], self.root/"o", span)

    def test_compact_rinex_is_expanded_when_the_hatanaka_package_is_available(self):
        try:
            import hatanaka
        except ImportError:
            self.skipTest("hatanaka package not installed")
        header = BASE_HEADER.replace(b"                                                            END OF HEADER",
            b"G    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES\n"
            b"                                                            END OF HEADER")
        def epoch(tow):
            fields = f"{23338470.203:14.3f} 6{122644569.975:14.3f} 6{-3852.309:14.3f}  {44.25:14.3f}  "
            return epoch_line(tow, nsat=1, crlf=False) + b"G01" + fields.encode() + b"\n"
        rinex = header + epoch(360) + epoch(361) + epoch(362)
        try:
            compressed = hatanaka.compress(rinex)
        except Exception as error:  # the compressor is picky about the header; a failure here is not a converter defect
            self.skipTest(f"hatanaka.compress rejected the synthetic file: {error}")
        path = self.write("c.crx.gz", compressed)  # hatanaka.compress returns gzip-wrapped CRINEX, as the published .crx.gz
        dst = self.root/"expanded"
        stats, epochs = conv.assemble_base([path], dst, (Fraction(360), Fraction(361)))
        self.assertEqual(epochs, {360, 361})
        self.assertTrue(dst.read_bytes().startswith(b"     3.02           OBSERVATION DATA"))
        self.assertEqual(dst.read_bytes().count(b"\n> "), 2)


class NavigationTest(Case):
    def merge(self, systems="CEGR", **kw):
        paths = [self.write(f"{s}N.rnx", nav_file(s, records=n)) for s, n in zip(systems, kw.get("counts", (1,)*len(systems)))]
        stats = conv.merge_navigation(paths, self.root/"nav")
        return (self.root/"nav").read_bytes(), stats

    def test_mixed_header_canonical_order_and_unchanged_bodies(self):
        data, stats = self.merge("CEGR", counts=(1, 2, 3, 4))
        lines = data.split(b"\n")
        self.assertEqual(lines[0], b"     3.02           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE")
        header = data[:data.index(b"END OF HEADER")+len(b"END OF HEADER\n")]
        labels = [l[60:].strip() for l in header.split(b"\n") if l]
        self.assertEqual(labels, [b"RINEX VERSION / TYPE", b"PGM / RUN BY / DATE", b"COMMENT", b"IONOSPHERIC CORR", b"IONOSPHERIC CORR",
                                  b"LEAP SECONDS", b"END OF HEADER"])
        self.assertIn(b"GPSA   6.5193D-09", header)
        self.assertIn(b"BDSA   1.0245D-08", header)
        self.assertEqual(stats["systems"], ["G", "R", "E", "C"])
        self.assertEqual(stats["records_per_system"], dict(G=3, R=4, E=2, C=1))
        body = data[len(header):]
        want = b"".join(nav_file(s, records=n).split(b"END OF HEADER\n", 1)[1] for s, n in zip("GREC", (3, 4, 2, 1)))
        self.assertEqual(body, want)
        self.assertEqual(len(re.findall(rb"^[GREC]\d\d ", body, re.M)), 10)

    def test_rejections(self):
        with self.assertRaisesRegex(ValueError, "repeat"):
            conv.merge_navigation([self.write("a", nav_file("G")), self.write("b", nav_file("G"))], self.root/"n1")
        with self.assertRaisesRegex(ValueError, "versions"):
            conv.merge_navigation([self.write("c", nav_file("G")), self.write("d", nav_file("R", version="3.04"))], self.root/"n2")
        with self.assertRaisesRegex(ValueError, "not a RINEX navigation"):
            conv.merge_navigation([self.write("e", BASE_HEADER + obs_epoch(1))], self.root/"n3")
        wrong = nav_file("G").replace(b"G01 2021", b"R01 2021")
        with self.assertRaisesRegex(ValueError, "record of system R in a G file"):
            conv.merge_navigation([self.write("f", wrong)], self.root/"n4")


class EndToEndTest(Case):
    def make_raw(self):
        """Truth 100..106 (7 rows, 105 missing a rover epoch), rover 98..107, IMU covering 99.0..105.5 at 400 Hz."""
        raw = self.root/"raw"
        raw.mkdir()
        rover_times = [98, 99, 100, 101, 102, 103, 104, 106, 107]
        (raw/"rover.obs").write_bytes(OBS_HEADER + b"".join(obs_epoch(t) for t in rover_times))
        (raw/"truth.txt").write_text(TRUTH_HEADER + "".join(truth_line(t, vel=(0.0, 5.0, 0.0), rpy=("0", "0", "90")) for t in range(100, 107)))
        imu = [imu_line(Fraction(99) + Fraction(k, 400), acc=(0.0, 1.0, 9.8)) for k in range(0, int(6.5*400)+1)]
        (raw/"imu.csv").write_text(IMU_HEADER + "".join(imu))
        (raw/"base.rnx").write_bytes(BASE_HEADER + b"".join(obs_epoch(t, crlf=False) for t in range(95, 110) if t != 103) + base_event())
        for s in "GREC": (raw/f"{s}N.rnx").write_bytes(nav_file(s))
        return raw

    def convert(self, raw, out, run="HKDeepUrban1"):
        return conv.convert(run, "novatel", raw/"rover.obs", raw/"truth.txt", raw/"imu.csv", [raw/"base.rnx"],
                            [raw/f"{s}N.rnx" for s in "GREC"], out)

    def test_layout_epoch_rules_and_manifest(self):
        raw, out = self.make_raw(), self.root/"out"
        manifest = self.convert(raw, out)
        directory = out/"urbannav/HKDeepUrban1_novatel"   # the replay accepts <tokyo|nagoya|urbannav>/<run>
        self.assertEqual(directory.parent.name, "urbannav")
        self.assertEqual(sorted(p.name for p in directory.iterdir()), ["base.nav", "base.obs", "imu.csv", "reference.csv", "rover.obs"])
        # Kept: truth rows 100..106 with a rover epoch (100-104, 106) and an IMU sample (IMU ends at 105.5): 100-104 only.
        stats = manifest["stats"]
        self.assertEqual(stats["rover.obs"]["epochs_in"], 9)
        self.assertEqual(stats["rover.obs"]["epochs_without_truth_row"], 3)               # 98, 99, 107
        self.assertEqual(stats["rover.obs"]["epochs_with_truth_row_without_imu_sample"], 1)  # 106
        self.assertEqual((stats["rover.obs"]["first_tow_out"], stats["rover.obs"]["last_tow_out"], stats["rover.obs"]["epochs_out"]), (100.0, 104.0, 5))
        self.assertEqual((directory/"rover.obs").read_bytes(), OBS_HEADER + b"".join(obs_epoch(t) for t in range(100, 105)))
        self.assertEqual(stats["reference.csv"]["rows_out"], 5)
        ref = (directory/"reference.csv").read_text().splitlines()
        self.assertEqual([line.split(",")[0] for line in ref[1:]], ["100.0", "101.0", "102.0", "103.0", "104.0"])
        self.assertEqual([float(v) for v in ref[1].split(",")[11:14]], [5.0, 0.0, 0.0])     # heading 90, 5 m/s forward -> East
        # base.obs holds the base epochs from the first to the last kept rover epoch (103 is missing in the base).
        self.assertEqual((directory/"base.obs").read_bytes(), BASE_HEADER + b"".join(obs_epoch(t, crlf=False) for t in (100, 101, 102, 104)))
        self.assertEqual((stats["base.obs"]["epochs_in_files"], stats["base.obs"]["epochs_out"], stats["base.obs"]["epochs_dropped_before_span"],
                          stats["base.obs"]["epochs_dropped_after_span"], stats["base.obs"]["event_records_dropped"]), (14, 4, 5, 5, 1))
        self.assertEqual(stats["verification"]["rover_epochs_without_exact_base_epoch"], 1)   # base misses 103
        self.assertEqual(stats["verification"]["epochs_without_exact_base_epoch"], [103.0])
        self.assertEqual((stats["verification"]["rover_epochs_without_truth_row"], stats["verification"]["rover_epochs_without_imu_sample"]), (0, 0))
        self.assertEqual(stats["verification"]["reference_rows_without_rover_epoch"], 0)
        self.assertTrue((directory/"imu.csv").read_text().startswith(conv.PPC_IMU_HEADER))
        self.assertEqual(manifest["schema"], "libgnsspp.urbannav_hk_ppc_conversion.v1")
        for name, entry in manifest["outputs"].items():
            self.assertEqual(entry["sha256"], hashlib.sha256((directory/name).read_bytes()).hexdigest())
        self.assertEqual(manifest["raw_inputs"]["rover.obs"]["sha256"], hashlib.sha256((raw/"rover.obs").read_bytes()).hexdigest())
        self.assertEqual([e["name"] for e in manifest["raw_inputs"]["base.obs"]], ["base.rnx"])
        self.assertEqual(len(manifest["raw_inputs"]["base.nav"]), 4)
        on_disk = json.loads((out/"urbannav/HKDeepUrban1_novatel.manifest.json").read_text())
        self.assertEqual(on_disk["outputs"], manifest["outputs"])
        # Every kept rover epoch has an IMU grid sample exactly on it.
        imu_times = {line.split(",")[0] for line in (directory/"imu.csv").read_text().splitlines()[1:]}
        for t in range(100, 105): self.assertIn(f"{t}.00", imu_times)

    def test_medium_urban_label_changes_only_the_names(self):
        """HKMediumUrban1 is accepted; every converted file equals that of another run label byte for byte."""
        self.assertEqual(conv.RUNS, ("HKDeepUrban1", "HKHarshUrban1", "HKMediumUrban1"))
        raw = self.make_raw()
        deep = self.convert(raw, self.root/"d", run="HKDeepUrban1")
        medium = self.convert(raw, self.root/"m", run="HKMediumUrban1")
        self.assertEqual(medium["run"], "HKMediumUrban1")
        self.assertEqual(medium["outputs"].keys(), deep["outputs"].keys())
        for name in medium["outputs"]:
            self.assertEqual((self.root/"m/urbannav/HKMediumUrban1_novatel"/name).read_bytes(),
                             (self.root/"d/urbannav/HKDeepUrban1_novatel"/name).read_bytes(), name)
            self.assertEqual(medium["outputs"][name]["sha256"], deep["outputs"][name]["sha256"])
        self.assertEqual(medium["stats"], deep["stats"])
        self.assertTrue((self.root/"m/urbannav/HKMediumUrban1_novatel.manifest.json").is_file())

    def test_unsupported_run_rover_and_existing_output(self):
        raw = self.make_raw()
        with self.assertRaisesRegex(ValueError, "run must be"):
            self.convert(raw, self.root/"o1", run="Odaiba")
        self.convert(raw, self.root/"o2")
        with self.assertRaisesRegex(ValueError, "already exists"):
            self.convert(raw, self.root/"o2")
        with self.assertRaisesRegex(ValueError, "rover"):
            conv.convert("HKDeepUrban1", "ublox", raw/"rover.obs", raw/"truth.txt", raw/"imu.csv", [raw/"base.rnx"], [raw/"GN.rnx"], self.root/"o3")

    def test_cli_exit_codes(self):
        raw = self.make_raw()
        argv = ["--run", "HKDeepUrban1", "--rover-obs", str(raw/"rover.obs"), "--reference", str(raw/"truth.txt"), "--imu", str(raw/"imu.csv"),
                "--base-obs", str(raw/"base.rnx"), "--nav", *[str(raw/f"{s}N.rnx") for s in "GREC"], "--output-root", str(self.root/"cli")]
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(conv.main(argv), 0)
        self.assertEqual(json.loads(out.getvalue())["stats"]["rover.obs"]["epochs_out"], 5)
        with contextlib.redirect_stderr(io.StringIO()) as err:
            self.assertEqual(conv.main(argv), 2)  # refuses to overwrite
            self.assertIn("already exists", err.getvalue())
            (raw/"imu.csv").unlink()
            argv[-1] = str(self.root/"cli2")
            self.assertEqual(conv.main(argv), 2)  # missing raw input
            self.assertIn("missing raw input", err.getvalue())


class FrameCheckTest(Case):
    """check_urbannav_hk_frames.py on a synthetic drive: speed and heading vary, roll = pitch = 0."""
    SECONDS = 150
    G = frames.GRAVITY

    @staticmethod
    def dms(value):
        sign = "-" if value < 0 else ""
        value = abs(value)
        d, m = int(value), int(value*60) % 60
        return f"{sign}{d} {m} {(value*3600) % 60:.5f}"

    def drive(self):
        speed = lambda t: 10 + 2*math.sin(2*math.pi*t/20)
        heading = lambda t: 60*math.sin(2*math.pi*t/30)   # degrees clockwise from north
        lat, lon, truth = 22.3, 114.2, []
        for k in range(0, self.SECONDS+1):
            truth.append(truth_line(1000 + k, lat=self.dms(lat), lon=self.dms(lon), vel=(0.0, speed(k), 0.0), rpy=("0", "0", f"{heading(k) % 360:.6f}")))
            # 1 s step by the mean velocity over the interval (fine integration: 100 sub-steps)
            for j in range(100):
                t = k + (j + .5)/100
                lat += speed(t)*math.cos(math.radians(heading(t)))*0.01/6378137*180/math.pi
                lon += speed(t)*math.sin(math.radians(heading(t)))*0.01/(6378137*math.cos(math.radians(lat)))*180/math.pi
        imu = []
        for k in range(0, (self.SECONDS+4)*400):
            tow = 998 + k/400   # the drive starts at truth time of week 1000
            t, h = tow - 1000, 1e-4
            a_fwd = (speed(t+h)-speed(t-h))/(2*h)
            yaw_ccw = -math.radians((heading(t+h)-heading(t-h))/(2*h))
            # FLU specific force (a_fwd, v*yaw_ccw, g); raw Xsens axes are right, forward, up
            imu.append(imu_line(Fraction(tow).limit_denominator(4000), acc=(-speed(t)*yaw_ccw, a_fwd, self.G), gyro=(0.0, 0.0, yaw_ccw)))
        return self.write("truth.txt", TRUTH_HEADER + "".join(truth)), self.write("imu.csv", IMU_HEADER + "".join(imu))

    def test_permutations_are_the_24_proper_rotations(self):
        matrices = [m_ for m_, _ in frames.proper_signed_permutations()]
        self.assertEqual(len(matrices), 24)
        self.assertEqual(len({str(m_) for m_ in matrices}), 24)
        self.assertAlmostEqual(frames.correlation([1, 2, 3], [2, 4, 6]), 1.0)
        self.assertAlmostEqual(frames.correlation([1, 2, 3], [3, 2, 1]), -1.0)

    def test_the_forward_axis_the_imu_frame_and_a_zero_lag_are_recovered(self):
        truth, imu = self.drive()
        report = json.loads(self.run_cli(truth, imu))
        self.assertTrue(report["velocity"]["ranked_rms_mps"][0]["convention"].startswith("perm=(1,"))  # forward is the truth y axis
        self.assertLess(report["velocity"]["ranked_rms_mps"][0]["rms"], 0.2)
        self.assertGreater(report["velocity"]["worst_rms_mps"], 5)
        best = report["imu"]["ranked_rms_mps2"][0]
        self.assertEqual(best["raw_to_flu"], [[0, 1, 0], [-1, 0, 0], [0, 0, 1]])                       # FLU = (y, -x, z)
        self.assertLess(best["rms"], 0.2)
        self.assertGreater(report["imu"]["identity_rms_mps2"], 1.0)
        self.assertAlmostEqual(report["imu"]["gyro_z_vs_heading_rate"]["best_lag_s"], 0.0, delta=0.02)
        self.assertGreater(report["imu"]["gyro_z_vs_heading_rate"]["best_corr"], 0.99)

    def run_cli(self, truth, imu):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(frames.main(["--truth", str(truth), "--imu", str(imu)]), 0)
        return out.getvalue()


if __name__ == "__main__":
    unittest.main()
