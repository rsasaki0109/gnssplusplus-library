"""Synthetic-input tests for scripts/convert_smartpnt_pos_to_ppc_layout.py."""
import contextlib
import io
import json
import math
from fractions import Fraction
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"scripts"))
import convert_smartpnt_pos_to_ppc_layout as conv

WEEK = 2243
T0 = 274800                     # 2023-01-04 04:20:00 GPST (a Wednesday; 2023-01-01 is the Sunday of week 2243)
SECONDS = 60
MASTER = (30 + 31/60 + 43.32227/3600, 114 + 21/60 + 25.57915/3600, 40.0)
GYRO_SCALE, ACCEL_SCALE, RATE = 1e-6, 1e-6, 200.0


def calendar(tow):
    days, rest = divmod(int(tow), 86400)
    return 2023, 1, 1 + days, rest//3600, rest % 3600//60, rest % 60


def obs_header(position=" -2267700.0947  5009133.3821  3221207.5825", delta="        1.4007        0.0000        0.0000"):
    return (f"     3.04           OBSERVATION DATA    M                   RINEX VERSION / TYPE\n"
            f"{position:<60}APPROX POSITION XYZ \n"
            f"{delta:<60}ANTENNA: DELTA H/E/N\n"
            f"  2023     1     4     4    20   00.0000000     GPS         TIME OF FIRST OBS   \n"
            f"{'':<60}END OF HEADER       \n").encode()


def obs_file(first, last):
    body = b""
    for tow in range(first, last + 1):
        y, mo, d, h, mi, s = calendar(tow)
        body += f"> {y} {mo:02d} {d:02d} {h:02d} {mi:02d} {s:10.7f}  0  1\n".encode()
        body += b"G01  23338470.203 6 122644569.975 6       -3852.309          44.250\n"
    return body


def motion(t):
    """Speed (m/s), forward acceleration, yaw rate about up (deg/s) and heading (azimuth, deg) at time t."""
    s = t - T0
    speed = 5.0 + math.sin(0.3*s)
    accel = 0.3*math.cos(0.3*s)
    yaw_rate = 10.0*math.sin(0.2*s)
    heading = 90.0 + 10.0/0.2*(math.cos(0.2*s) - 1.0)   # d(heading)/dt = -yaw_rate
    return speed, accel, yaw_rate, heading


FRD_FROM_FLU = ((1, 0, 0), (0, -1, 0), (0, 0, -1))   # STIM300 body (README mounting 180, 0, 90) from FLU


def imr_bytes(first, last, flags=(1, 1, 2), magic=b"$IMURAW\0", body_from_flu=FRD_FROM_FLU, name=b"STIM300"):
    head = bytearray(512)
    head[0:8] = magic
    struct.pack_into("<b", head, 8, 0)
    struct.pack_into("<d", head, 9, 8.2)
    struct.pack_into("<ii", head, 17, flags[0], flags[1])
    struct.pack_into("<ddd", head, 25, RATE, GYRO_SCALE, ACCEL_SCALE)
    struct.pack_into("<i", head, 49, flags[2])
    head[65:65 + len(name)] = name
    records = bytearray()
    dt = 1/RATE
    n = int(round((last - first)*RATE))
    for k in range(n + 1):
        t = first + k*dt
        speed, accel, yaw_rate, _ = motion(t - dt/2)
        # FLU specific force and rate, then the IMU body axes.
        gyro_flu = (0.0, 0.0, yaw_rate)
        acc_flu = (accel, speed*math.radians(yaw_rate), 9.8)
        gyro = [sum(body_from_flu[i][j]*gyro_flu[j] for j in range(3)) for i in range(3)]
        acc = [sum(body_from_flu[i][j]*acc_flu[j] for j in range(3)) for i in range(3)]
        records += struct.pack("<d6i", t, *(round(v*dt/GYRO_SCALE) for v in gyro),
                               *(round(v*dt/ACCEL_SCALE) for v in acc))
    return bytes(head) + bytes(records)


def truth_text(first, last, sensor_lever="x=-0.512, y=-0.284, z=0.187", amb="Fixed"):
    head = ("\nProject:     demo\nProgram:     Inertial Explorer Version 9.00.0731\n"
            "Master 1:    Name 5948R4002, Status ENABLED\n"
            "             Antenna height 1.401 m, to L1PC [TRM57971.00(NONE)]\n"
            "             Lat, Lon, El Hgt 30 31 43.32227, 114 21 25.57915, 40.000 m [WGS84, 2023.003]\n"
            "IMU to GNSS Antenna Lever Arms:\n             x=-0.512, y=-0.284, z=0.187 m (x-right, y-fwd, z-up)\n"
            "Body to IMU Rotations:\n             xRot=180.000, yRot=0.000, zRot=0.000 degrees\n"
            f"IMU->Secondary Sensor Lever Arms: \n            {sensor_lever} m (x-right, y-fwd, z-up, IMU->SENSOR)\n"
            "Week    GPSTime       Latitude      Longitude        H-Ell Q AmbStatus        Date     GPSTime          "
            "X-ECEF          Y-ECEF          Z-ECEF         X-LL         Y-LL         Z-LL   VX-ECEF   VY-ECEF   "
            "VZ-ECEF     VEast    VNorth       VUp       Heading         Pitch          Roll\n"
            "week      (sec)          (deg)          (deg)          (m)                   (YMD)       (HMS)\n")
    rows = []
    for tow in range(first, last + 1):
        speed, _, _, heading = motion(tow)
        ve, vn = speed*math.sin(math.radians(heading)), speed*math.cos(math.radians(heading))
        rows.append(f"{WEEK} {tow:.3f}  30.5284464256 114.3557211472       21.973 1 {amb}      2023/01/04  4:20:00.00 "
                    f"-2267687.789702  5009376.728007  3220982.546671     -133.369      -28.438       -0.322    "
                    f"-0.001     0.002     0.000    {ve:.3f}    {vn:.3f}     0.000 {heading % 360.0:.9f}   0.5   -0.25 9.9\n")
    trailer = ("\r\nErrors/warnings:\r\n"
               "W102: Number of numerical digits larger than allowed width (3 occurances)\r\n\r\n")
    return head + "".join(rows) + trailer


def readme_text(marker):
    return ("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<DataInfo>\n\t<BASE>\n"
            f"\t\t<GNSS_BaseXYZ>{marker[0]:.4f}  {marker[1]:.4f}  {marker[2]:.4f}</GNSS_BaseXYZ>\n"
            "\t\t<GNSS_AntHeight>1.401</GNSS_AntHeight>\n\t</BASE>\n"
            "\t<ROVE><ID>01</ID><SINS_IMUType>ISA-100C</SINS_IMUType>"
            "<SINS_LeverArm_GNSS>-0.512 -0.284 0.187</SINS_LeverArm_GNSS><SINS_RotAngle_IMU>180.0 0.0 0.0</SINS_RotAngle_IMU></ROVE>\n"
            "\t<ROVE><ID>03</ID><SINS_IMUType>STIM300</SINS_IMUType>"
            "<SINS_LeverArm_GNSS>-0.500 -0.282 -0.057</SINS_LeverArm_GNSS><SINS_RotAngle_IMU>180.0 0.0 90.0</SINS_RotAngle_IMU></ROVE>\n"
            "</DataInfo>\n")


def make_run(root, run="Data12_20230104", imu_first=T0 - 2, imu_last=T0 + SECONDS + 2, truth=None, readme_marker=None):
    folder = Path(root)/f"{run}_ISA-100C_CAR_Complex"
    folder.mkdir(parents=True)
    marker = readme_marker or conv.geodetic_to_ecef(*MASTER)
    (folder/"README.xml").write_text(readme_text(marker), encoding="utf-8")
    (folder/"ROVE_03.23O").write_bytes(obs_header() + obs_file(T0 - 5, T0 + SECONDS + 5))
    (folder/"BASE.23O").write_bytes(obs_header(" -2267808.6227  5009324.7699  3221016.8928") + obs_file(T0 - 100, T0 + SECONDS + 100))
    (folder/"brdm0040.23p").write_bytes(b"     3.04           N: GNSS NAV DATA    M (MIXED)           RINEX VERSION / TYPE\n")
    (folder/"STIM300.imr").write_bytes(imr_bytes(imu_first, imu_last))
    (folder/"Rove_03_GroundTruth.txt").write_text(truth or truth_text(T0, T0 + SECONDS), encoding="latin-1")
    return folder


ISA_FROM_FLU = ((0, -1, 0), (-1, 0, 0), (0, 0, -1))   # README mounting 180, 0, 0: body x right, y back, z down


def make_generic_run(root):
    """Data13 layout: one ROVE entry (ISA-100C), ROVE.<yy>O, ROVE_GroundTruth.txt, ISA-100C.imr."""
    folder = Path(root)/"Data13_20211118_ISA-100C_CAR_Complex"
    folder.mkdir(parents=True)
    marker = conv.geodetic_to_ecef(*MASTER)
    (folder/"README.xml").write_text(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<DataInfo>\n\t<BASE>\n"
        f"\t\t<GNSS_BaseXYZ>{marker[0]:.4f} {marker[1]:.4f} {marker[2]:.4f}</GNSS_BaseXYZ>\n"
        "\t\t<GNSS_AntHeight>1.401</GNSS_AntHeight>\n\t</BASE>\n"
        "\t<ROVE><ID>01</ID><GNSS_OBSFile>ROVE.21O</GNSS_OBSFile><SINS_IMUType>ISA-100C</SINS_IMUType>"
        "<SINS_LeverArm_GNSS>-0.512 -0.284 0.187</SINS_LeverArm_GNSS>"
        "<SINS_RotAngle_IMU>180.000 0.000 0.000</SINS_RotAngle_IMU></ROVE>\n</DataInfo>\n", encoding="utf-8")
    (folder/"ROVE.21O").write_bytes(obs_header() + obs_file(T0 - 5, T0 + SECONDS + 5))
    # A header position of zeros, as Data02 has: the converter rewrites it anyway.
    (folder/"BASE.21O").write_bytes(obs_header("        0.0000        0.0000        0.0000") +
                                   obs_file(T0 - 100, T0 + SECONDS + 100))
    (folder/"brdm3220.21p").write_bytes(b"     3.04           N: GNSS NAV DATA    M (MIXED)           RINEX VERSION / TYPE\n")
    (folder/"ISA-100C.imr").write_bytes(imr_bytes(T0 - 2, T0 + SECONDS + 2, body_from_flu=ISA_FROM_FLU,
                                                  name=b"ISA-100C"))
    (folder/"ROVE_GroundTruth.txt").write_text(truth_text(T0, T0 + SECONDS), encoding="latin-1")
    return folder


class Geometry(unittest.TestCase):
    def test_stim300_mounting_is_frd_and_zero_mounting_is_rfu(self):
        self.assertEqual(conv.body_to_flu_matrix((180.0, 0.0, 90.0)), [[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.assertEqual(conv.body_to_flu_matrix((0.0, 0.0, 0.0)), [[0, 1, 0], [-1, 0, 0], [0, 0, 1]])

    def test_geodetic_round_trip_and_phase_centre(self):
        xyz = conv.geodetic_to_ecef(*MASTER)
        lat, lon, h = conv.ecef_to_geodetic(*xyz)
        self.assertAlmostEqual(lat, MASTER[0], places=10)
        self.assertAlmostEqual(lon, MASTER[1], places=10)
        self.assertAlmostEqual(h, MASTER[2], places=6)
        pc = conv.base_phase_centre(xyz, 1.401)
        self.assertAlmostEqual(conv.ecef_to_geodetic(*pc)[2], 41.401, places=6)
        self.assertAlmostEqual(math.dist(pc, xyz), 1.401, places=9)


class Imu(unittest.TestCase):
    def test_header_rejections(self):
        with tempfile.TemporaryDirectory() as tmp:
            for kwargs in (dict(magic=b"$IMUXXX\0"), dict(flags=(0, 1, 2)), dict(flags=(1, 1, 1))):
                path = Path(tmp)/"x.imr"
                path.write_bytes(imr_bytes(T0, T0 + 1, **kwargs))
                with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                    conv.convert_imu(path, Path(tmp)/"imu.csv", WEEK, (180.0, 0.0, 90.0))

    def test_rates_flu_and_the_hundred_hertz_grid(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp)/"STIM300.imr"
            src.write_bytes(imr_bytes(T0, T0 + 2))
            stats, grid, means = conv.convert_imu(src, Path(tmp)/"imu.csv", WEEK, (180.0, 0.0, 90.0))
            lines = (Path(tmp)/"imu.csv").read_text().splitlines()
        self.assertEqual(lines[0] + "\n", conv.PPC_IMU_HEADER)
        self.assertEqual(stats["raw_records"], 401)
        self.assertEqual(stats["rate_samples"], 400)
        # The first rate sample sits at T0 + 0.0075 s; grid points are written from T0 + 0.01 to T0 + 1.99.
        self.assertEqual(lines[1].split(",")[0], f"{T0}.01")
        self.assertEqual(lines[-1].split(",")[0], f"{T0 + 1}.99")
        self.assertEqual(len(grid), 199)
        fields = [float(v) for v in lines[100].split(",")]
        speed, accel, yaw_rate, _ = motion(fields[0])
        self.assertEqual(int(fields[1]), WEEK)
        self.assertAlmostEqual(fields[2], accel, places=3)                          # forward
        self.assertAlmostEqual(fields[3], speed*math.radians(yaw_rate), places=3)  # left = - right
        self.assertAlmostEqual(fields[4], 9.8, places=3)                            # up = - down
        self.assertAlmostEqual(fields[7], yaw_rate, places=3)                       # about up
        self.assertIn(T0 + 1, means)

    def test_a_gap_is_not_bridged(self):
        with tempfile.TemporaryDirectory() as tmp:
            data = bytearray(imr_bytes(T0, T0 + 2))
            # Drop 0.5 s of records after T0 + 1.
            start, stop = 512 + 32*200, 512 + 32*300
            src = Path(tmp)/"STIM300.imr"
            src.write_bytes(bytes(data[:start] + data[stop:]))
            stats, grid, _ = conv.convert_imu(src, Path(tmp)/"imu.csv", WEEK, (180.0, 0.0, 90.0))
        self.assertEqual(stats["pairs_over_gap"], 1)
        self.assertNotIn((T0 + 1)*100 + 25, grid)

    def test_a_duplicate_record_stream_is_dropped(self):
        with tempfile.TemporaryDirectory() as tmp:
            clean = imr_bytes(T0, T0 + 2)
            head, body = clean[:512], clean[512:]
            # As at the start of Data13: records 100-110 each have a twin 0.17 ms later with the same full-period
            # increments.
            mixed = bytearray(head)
            for k in range(len(body)//32):
                record = body[32*k:32*(k + 1)]
                mixed += record
                if 100 <= k <= 110:
                    t, *increments = conv.IMR_RECORD.unpack(record)
                    mixed += conv.IMR_RECORD.pack(t + 0.00017, *increments)
            (Path(tmp)/"clean.imr").write_bytes(clean)
            (Path(tmp)/"twins.imr").write_bytes(bytes(mixed))
            stats, _, _ = conv.convert_imu(Path(tmp)/"clean.imr", Path(tmp)/"clean.csv", WEEK, (180.0, 0.0, 90.0))
            twins, _, _ = conv.convert_imu(Path(tmp)/"twins.imr", Path(tmp)/"twins.csv", WEEK, (180.0, 0.0, 90.0))
            self.assertEqual(stats["duplicate_records_dropped"], 0)
            self.assertEqual(twins["duplicate_records_dropped"], 11)
            self.assertEqual(twins["raw_records"], stats["raw_records"] + 11)
            self.assertEqual(twins["rate_samples"], stats["rate_samples"])
            self.assertEqual((Path(tmp)/"twins.csv").read_text(), (Path(tmp)/"clean.csv").read_text())

    def test_a_timestamp_phase_shift_is_kept(self):
        with tempfile.TemporaryDirectory() as tmp:
            data = bytearray(imr_bytes(T0, T0 + 2))
            # As in Data04: from record 201 on, the stamps are 3 ms earlier, so record 201 follows 200 by 2 ms.
            for k in range(201, (len(data) - 512)//32):
                t, = struct.unpack_from("<d", data, 512 + 32*k)
                struct.pack_into("<d", data, 512 + 32*k, t - 0.003)
            src = Path(tmp)/"STIM300.imr"
            src.write_bytes(bytes(data))
            stats, _, _ = conv.convert_imu(src, Path(tmp)/"imu.csv", WEEK, (180.0, 0.0, 90.0))
        self.assertEqual(stats["duplicate_records_dropped"], 0)
        self.assertEqual(stats["rate_samples"], stats["raw_records"] - 1)
        self.assertAlmostEqual(stats["max_raw_dt_s"], 0.005, places=6)


class Truth(unittest.TestCase):
    def test_header_facts_and_rows_by_the_date_token(self):
        with tempfile.TemporaryDirectory() as tmp:
            for amb in ("Fixed", "No Fix"):
                path = Path(tmp)/f"truth_{amb[0]}.txt"
                path.write_text(truth_text(T0, T0 + 3, amb=amb), encoding="latin-1")
                header, rows = conv.parse_truth(path)
                with self.subTest(amb=amb):
                    self.assertEqual(header["antenna_lever_arm"], [-0.512, -0.284, 0.187])
                    self.assertEqual(header["sensor_lever_arm"], [-0.512, -0.284, 0.187])
                    for got, want in zip(header["master_lat_lon_height"], MASTER):
                        self.assertAlmostEqual(got, want, places=12)
                    self.assertEqual(len(rows), 4)
                    self.assertEqual(rows[0]["tow"], Fraction(T0))
                    self.assertEqual(rows[0]["ecef"], ["-2267687.789702", "5009376.728007", "3220982.546671"])
                    self.assertEqual((rows[0]["pitch"], rows[0]["roll"]), ("0.5", "-0.25"))

    def test_non_increasing_time_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            body, trailer = truth_text(T0, T0 + 2).split("\r\nErrors/warnings:", 1)
            lines = body.splitlines(keepends=True)
            path = Path(tmp)/"truth.txt"
            path.write_text("".join(lines + [lines[-1]]) + "\r\nErrors/warnings:" + trailer, encoding="latin-1")
            with self.assertRaisesRegex(ValueError, "not strictly increasing"):
                conv.parse_truth(path)

    def test_axis_check_accepts_the_right_axes_and_rejects_a_flipped_gyro(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp)/"STIM300.imr"
            src.write_bytes(imr_bytes(T0 - 2, T0 + SECONDS + 2))
            _, _, means = conv.convert_imu(src, Path(tmp)/"imu.csv", WEEK, (180.0, 0.0, 90.0))
            truth = Path(tmp)/"truth.txt"
            truth.write_text(truth_text(T0, T0 + SECONDS), encoding="latin-1")
            _, rows = conv.parse_truth(truth)
        keep = {row["tow"] for row in rows}
        result = conv.check_imu_axes(means, rows, keep)
        self.assertGreater(result["gyro_z_vs_heading_rate"], 0.99)
        self.assertGreater(result["accel_x_vs_speed_change"], 0.9)
        self.assertGreater(result["accel_y_vs_centripetal"], 0.9)
        flipped = {s: [v[0], v[1], v[2], v[3], v[4], -v[5]] for s, v in means.items()}
        with self.assertRaises(ValueError):
            conv.check_imu_axes(flipped, rows, keep)


class BaseHeader(unittest.TestCase):
    def test_position_and_antenna_delta_are_rewritten_and_nothing_else(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"base.obs"
            original = obs_header(" -2267808.6227  5009324.7699  3221016.8928") + obs_file(T0, T0 + 1)
            path.write_bytes(original)
            conv.rewrite_base_position(path, (-2267809.8051, 5009324.1468, 3221016.6904))
            new = path.read_bytes().splitlines()
        old = original.splitlines()
        self.assertEqual(len(new), len(old))
        self.assertEqual(new[1], b" -2267809.8051  5009324.1468  3221016.6904                  APPROX POSITION XYZ")
        self.assertEqual(new[2], b"        0.0000        0.0000        0.0000                  ANTENNA: DELTA H/E/N")
        self.assertEqual([line for i, line in enumerate(new) if i not in (1, 2)],
                         [line for i, line in enumerate(old) if i not in (1, 2)])


class Conversion(unittest.TestCase):
    def test_layout_rules_and_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = make_run(Path(tmp)/"raw")
            out = Path(tmp)/"out"
            manifest = conv.convert("Data12_20230104", folder, out)
            target = out/"smartpnt"/"Data12_20230104_rove03_stim300"
            self.assertEqual(sorted(p.name for p in target.iterdir()),
                             ["base.nav", "base.obs", "imu.csv", "reference.csv", "rover.obs"])
            stats = manifest["stats"]
            self.assertEqual(stats["rover.obs"]["epochs_out"], SECONDS + 1)
            self.assertEqual(stats["verification"]["rover_epochs_without_exact_base_epoch"], 0)
            self.assertEqual(stats["base.obs"]["epochs_out"], SECONDS + 1)
            self.assertEqual(manifest["replay"]["lever_arm_flu_m"], [-0.282, 0.5, -0.057])
            self.assertEqual(manifest["contract"], "docs/smartpnt_fuse_output_holdout.md")
            base_lines = (target/"base.obs").read_bytes().splitlines()
            readme = conv.read_readme(folder/"README.xml", "STIM300")   # the marker as the README prints it
            want = conv.base_phase_centre(readme["base_marker_ecef"], 1.401)
            self.assertEqual(base_lines[1][:42].decode(), "".join(f"{v:14.4f}" for v in want))
            reference = (target/"reference.csv").read_text().splitlines()
            self.assertEqual(reference[0].split(",")[-1], "Truth Quality (Q)")
            self.assertEqual(len(reference), SECONDS + 2)
            on_disk = json.loads((out/"smartpnt"/"Data12_20230104_rove03_stim300.manifest.json").read_text())
            self.assertEqual(on_disk["outputs"]["imu.csv"]["sha256"], manifest["outputs"]["imu.csv"]["sha256"])
            with self.assertRaises(ValueError):
                conv.convert("Data12_20230104", folder, out)          # output exists

    def test_a_blank_line_after_the_rover_header_is_dropped(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = make_run(Path(tmp)/"raw")
            rover = folder/"ROVE_03.23O"
            rover.write_bytes(rover.read_bytes().replace(b"END OF HEADER       \n", b"END OF HEADER       \n\r\n", 1))
            manifest = conv.convert("Data12_20230104", folder, Path(tmp)/"out")
            out = (Path(tmp)/"out"/"smartpnt"/"Data12_20230104_rove03_stim300"/"rover.obs").read_bytes()
        self.assertEqual(manifest["stats"]["rover.obs"]["blank_lines_dropped"], 1)
        self.assertNotIn(b"\r\n", out)
        self.assertEqual(manifest["stats"]["rover.obs"]["epochs_out"], SECONDS + 1)

    def test_repeated_rover_epochs_are_dropped(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = make_run(Path(tmp)/"raw")
            rover = folder/"ROVE_03.23O"
            data = rover.read_bytes()
            epoch = obs_file(T0 + 10, T0 + 10)            # the epoch at T0 + 10 once more, after T0 + 10
            marker = obs_file(T0 + 11, T0 + 11)
            rover.write_bytes(data.replace(marker, epoch + marker, 1))
            manifest = conv.convert("Data12_20230104", folder, Path(tmp)/"out")
        self.assertEqual(manifest["stats"]["rover.obs"]["non_increasing_epochs_dropped"], 1)
        self.assertEqual(manifest["stats"]["rover.obs"]["epochs_out"], SECONDS + 1)

    def test_the_truth_point_must_be_the_antenna_within_the_run_tolerance(self):
        offset_lever = "x=-0.512, y=-0.254, z=0.187"          # 3 cm from the antenna lever arm
        with tempfile.TemporaryDirectory() as tmp:
            folder = make_run(Path(tmp)/"raw", truth=truth_text(T0, T0 + SECONDS, sensor_lever=offset_lever))
            with self.assertRaisesRegex(ValueError, "not exported at the GNSS antenna"):
                conv.convert("Data12_20230104", folder, Path(tmp)/"out")
            spec = dict(conv.RUNS["Data12_20230104"], truth_point_tolerance_m=0.05)
            with mock.patch.dict(conv.RUNS, {"Data12_20230104": spec}):
                manifest = conv.convert("Data12_20230104", folder, Path(tmp)/"out2")
        self.assertAlmostEqual(manifest["stats"]["reference.csv"]["export_point_offset_from_antenna_m"], 0.03)

    def test_a_rove_run_with_another_imu_uses_its_own_spec(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = make_generic_run(Path(tmp)/"raw")
            out = Path(tmp)/"out"
            manifest = conv.convert("Data13_20211118", folder, out)
            readme = conv.read_readme(folder/"README.xml", "ISA-100C")
            target = out/"smartpnt"/"Data13_20211118_rove_isa100c"
            self.assertTrue((out/"smartpnt"/"Data13_20211118_rove_isa100c.manifest.json").is_file())
            self.assertEqual(sorted(p.name for p in target.iterdir()),
                             ["base.nav", "base.obs", "imu.csv", "reference.csv", "rover.obs"])
        self.assertEqual(manifest["contract"], "docs/smartpnt_reverse_start_holdout.md")
        self.assertEqual(manifest["raw_inputs"]["rover_obs"]["name"], "ROVE.21O")
        self.assertEqual(manifest["raw_inputs"]["imu"]["name"], "ISA-100C.imr")
        self.assertEqual(manifest["raw_inputs"]["reference"]["name"], "ROVE_GroundTruth.txt")
        self.assertEqual(manifest["replay"]["lever_arm_flu_m"], [-0.284, 0.512, 0.187])
        self.assertTrue(manifest["replay"]["note"].startswith("ISA-100C to the ROVE antenna;"))
        self.assertEqual(manifest["stats"]["imu.csv"]["body_to_flu"], [[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
        self.assertEqual(manifest["stats"]["rover.obs"]["epochs_out"], SECONDS + 1)
        self.assertGreater(manifest["stats"]["imu_axis_check"]["gyro_z_vs_heading_rate"], 0.99)
        self.assertEqual(manifest["conventions"]["truth"], conv.ROVER_ANTENNA_TRUTH)
        self.assertEqual(manifest["stats"]["base.obs"]["approx_position_ecef"], [0.0, 0.0, 0.0])
        want = conv.base_phase_centre(readme["base_marker_ecef"], 1.401)
        self.assertEqual(manifest["stats"]["base.obs"]["written_position_ecef"], want)
        with tempfile.TemporaryDirectory() as tmp:
            folder = make_generic_run(Path(tmp)/"raw")
            with self.assertRaises(ValueError):                    # the folder must match the run
                conv.convert("Data14_20200828", folder, Path(tmp)/"out")

    def test_the_two_fuse_output_runs_keep_their_spec(self):
        self.assertEqual(conv.output_name("Data11_20221230"), "Data11_20221230_rove03_stim300")
        self.assertEqual(conv.output_name("Data12_20230104"), "Data12_20230104_rove03_stim300")
        for run in ("Data11_20221230", "Data12_20230104"):
            spec = conv.RUNS[run]
            self.assertEqual((spec["rover"], spec["imu"], spec["truth"], spec["contract"]),
                             ("ROVE_03", "STIM300", "Rove_03_GroundTruth.txt", "docs/smartpnt_fuse_output_holdout.md"))
        reverse = [run for run, spec in conv.RUNS.items() if spec["contract"] == conv.REVERSE_START_CONTRACT]
        self.assertEqual(len(reverse), 8)     # Data04, excluded there before any conversion, moved to the next
        realign = sorted(run for run, spec in conv.RUNS.items() if spec["contract"] == conv.PRELATCH_REALIGN_CONTRACT)
        self.assertEqual(realign, ["Data03_20150722", "Data04_20220315", "Data05_20201128", "Data17_20200617",
                                   "Data18_20200618"])
        self.assertEqual(conv.RUNS["Data03_20150722"]["truth_point_tolerance_m"], 0.05)
        self.assertEqual(sum(spec["truth_point_tolerance_m"] > 0 for spec in conv.RUNS.values()), 1)

    def test_epochs_without_imu_are_dropped(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = make_run(Path(tmp)/"raw", imu_first=T0 + 10)
            manifest = conv.convert("Data12_20230104", folder, Path(tmp)/"out")
        rover = manifest["stats"]["rover.obs"]
        self.assertEqual(rover["first_tow_out"], T0 + 11)
        self.assertEqual(rover["epochs_with_truth_row_without_imu_sample"], 11)

    def test_rejections(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = make_run(Path(tmp)/"a", truth=truth_text(T0, T0 + SECONDS, sensor_lever="x=-0.012, y=-0.002, z=0.244"))
            with self.assertRaises(ValueError):                    # truth exported at the IMU, not the antenna
                conv.convert("Data12_20230104", folder, Path(tmp)/"out_a")
            marker = list(conv.geodetic_to_ecef(*MASTER)); marker[0] += 0.02
            folder = make_run(Path(tmp)/"b", readme_marker=marker)
            with self.assertRaises(ValueError):                    # README base and truth master disagree
                conv.convert("Data12_20230104", folder, Path(tmp)/"out_b")
            with self.assertRaises(ValueError):
                conv.convert("Data13_20211118", folder, Path(tmp)/"out_c")

    def test_cli_exit_codes(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = make_run(Path(tmp)/"raw")
            with contextlib.redirect_stdout(io.StringIO()) as stdout:
                self.assertEqual(conv.main(["--run", "Data12_20230104", "--run-dir", str(folder),
                                            "--output-root", str(Path(tmp)/"out")]), 0)
            self.assertIn("lever_arm_flu_m", stdout.getvalue())
            with contextlib.redirect_stderr(io.StringIO()) as stderr:
                self.assertEqual(conv.main(["--run", "Data12_20230104", "--run-dir", str(folder),
                                            "--output-root", str(Path(tmp)/"out")]), 2)
            self.assertIn("output already exists", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
