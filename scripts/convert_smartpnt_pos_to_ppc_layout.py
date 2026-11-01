#!/usr/bin/env python3
"""Convert a SmartPNT-POS car run (Wuhan) to the PPC run layout.

Implements the "Conversion" section of docs/smartpnt_fuse_output_holdout.md. Pure standard library.
The raw files are read as data only; nothing from them is executed.

Input: --run-dir, the run folder of the Kaggle dataset fengzhusgg/smartpnt-pos
(Dataset/Data11_20221230_ISA-100C_CAR_Complex or Dataset/Data12_20230104_ISA-100C_CAR_Complex), with
    README.xml               base marker position, antenna height, IMU lever arms and mounting angles
    ROVE_03.<yy>O            Septentrio mosaic-X5 rover, RINEX 3.04, 1 Hz (the GNSS receiver used here)
    BASE.<yy>O / Base.<yy>O  Trimble Alloy base, RINEX 3.04, 1 Hz
    brdm<doy>0.<yy>p         merged broadcast navigation, RINEX 3.04
    STIM300.imr              STIM300 IMU, Inertial Explorer IMR, 200 Hz delta-theta/delta-v, GPS time
    Rove_03_GroundTruth.txt  Inertial Explorer smoothed GNSS/INS solution at the ROVE_01/ROVE_03 antenna

Output: <output-root>/smartpnt/<run>_rove03_stim300/{rover.obs,base.obs,base.nav,imu.csv,reference.csv}
plus <run>_rove03_stim300.manifest.json beside that directory (SHA256 of every raw input and output).
"""
import argparse
from fractions import Fraction
import json
import math
from pathlib import Path
import re
import shutil
import struct
import sys
import xml.etree.ElementTree as ElementTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
from convert_urbannav_to_ppc_layout import PPC_IMU_HEADER, pin  # noqa: E402
from convert_urbannav_hk_to_ppc_layout import (  # noqa: E402  shared, unchanged helpers of the HK converter
    REFERENCE_HEADER, assemble_base, filter_rover_obs, format_tow, velocity_text, verify_converted)

# Each run and the contract that governs its use, recorded in the manifest.
RUNS = {
    "Data11_20221230": "docs/smartpnt_fuse_output_holdout.md",
    "Data12_20230104": "docs/smartpnt_fuse_output_holdout.md",
}
RUN_FOLDER_SUFFIX = "_ISA-100C_CAR_Complex"
IMU_TYPE = "STIM300"

GRID_CS = 1                          # 100 Hz grid in centiseconds of GPS time of week, as the PPC imu.csv
MAX_PAIR_GAP_S = 0.1                 # raw samples further apart than this are not bridged (as the other converters)
IMR_HEADER_BYTES = 512
IMR_RECORD = struct.Struct("<d6i")   # time of week, then gx gy gz ax ay az increments
MIN_AXIS_CORRELATION = 0.9           # gyro z against the truth heading rate, see check_imu_axes()

# WGS84
A_WGS84 = 6378137.0
E2_WGS84 = 0.00669437999014


# ---------------------------------------------------------------------------
# geometry


def geodetic_to_ecef(lat_deg, lon_deg, height):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = A_WGS84/math.sqrt(1 - E2_WGS84*math.sin(lat)**2)
    return ((n + height)*math.cos(lat)*math.cos(lon), (n + height)*math.cos(lat)*math.sin(lon),
            (n*(1 - E2_WGS84) + height)*math.sin(lat))


def ecef_to_geodetic(x, y, z):
    """(lat, lon) in degrees and ellipsoidal height, iterated to well below a millimetre."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p*(1 - E2_WGS84))
    for _ in range(10):
        n = A_WGS84/math.sqrt(1 - E2_WGS84*math.sin(lat)**2)
        height = p/math.cos(lat) - n
        lat = math.atan2(z, p*(1 - E2_WGS84*n/(n + height)))
    n = A_WGS84/math.sqrt(1 - E2_WGS84*math.sin(lat)**2)
    return math.degrees(lat), math.degrees(lon), p/math.cos(lat) - n


def mounting_matrix(x_deg, y_deg, z_deg):
    """Vehicle (x right, y forward, z up) to IMU body, R_vb = R_Y(y) R_X(x) R_Z(z).

    "Coordinate frame and mounting parameters.pdf" of the dataset: the mounting angle (X, Y, Z) rotates the
    vehicle frame first about Z, then X, then Y, with frame-rotation (passive) single-axis matrices.
    """
    def rx(a):
        c, s = math.cos(a), math.sin(a)
        return [[1, 0, 0], [0, c, s], [0, -s, c]]

    def ry(a):
        c, s = math.cos(a), math.sin(a)
        return [[c, 0, -s], [0, 1, 0], [s, 0, c]]

    def rz(a):
        c, s = math.cos(a), math.sin(a)
        return [[c, s, 0], [-s, c, 0], [0, 0, 1]]

    def mul(a, b):
        return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    x, y, z = map(math.radians, (x_deg, y_deg, z_deg))
    return mul(ry(y), mul(rx(x), rz(z)))


def body_to_flu_matrix(mounting_deg):
    """IMU body to FLU: v = R_vb^T b (vehicle right/forward/up), then FLU = (v_forward, -v_right, v_up)."""
    r_vb = mounting_matrix(*mounting_deg)
    r_bv = [[r_vb[j][i] for j in range(3)] for i in range(3)]
    vehicle_to_flu = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
    m = [[sum(vehicle_to_flu[i][k]*r_bv[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    # The STIM300 angles give a signed permutation; round away the 1e-16 cosine residue.
    return [[float(round(v)) if abs(v - round(v)) < 1e-12 else v for v in row] for row in m]


# ---------------------------------------------------------------------------
# README.xml


def read_readme(path):
    """Base marker ECEF, antenna height, and the STIM300 lever arm (vehicle frame) and mounting angles."""
    root = ElementTree.parse(path).getroot()
    base = root.find("BASE")
    marker = [float(v) for v in base.findtext("GNSS_BaseXYZ").split()]
    antenna_height = float(base.findtext("GNSS_AntHeight"))
    rovers = [r for r in root.findall("ROVE") if r.findtext("SINS_IMUType", "").strip() == IMU_TYPE]
    if len(rovers) != 1:
        raise ValueError(f"{path}: expected exactly one ROVE entry with SINS_IMUType {IMU_TYPE}")
    lever = [float(v) for v in rovers[0].findtext("SINS_LeverArm_GNSS").split()]
    mounting = [float(v) for v in rovers[0].findtext("SINS_RotAngle_IMU").split()]
    if len(marker) != 3 or len(lever) != 3 or len(mounting) != 3:
        raise ValueError(f"{path}: malformed base position, lever arm or mounting angles")
    return dict(base_marker_ecef=marker, base_antenna_height_m=antenna_height,
                stim300_lever_arm_vehicle_m=lever, stim300_mounting_deg=mounting)


def base_phase_centre(marker, antenna_height):
    """Marker plus the antenna height along the local ellipsoidal up (the truth states "to L1PC")."""
    lat, lon, _ = ecef_to_geodetic(*marker)
    la, lo = math.radians(lat), math.radians(lon)
    up = (math.cos(la)*math.cos(lo), math.cos(la)*math.sin(lo), math.sin(la))
    return [marker[i] + antenna_height*up[i] for i in range(3)]


# ---------------------------------------------------------------------------
# reference.csv (Inertial Explorer text export)

DATE_TOKEN = re.compile(r"^\d{4}/\d{2}/\d{2}$")
TRUTH_LEVER = re.compile(r"x=\s*(-?[0-9.]+),\s*y=\s*(-?[0-9.]+),\s*z=\s*(-?[0-9.]+)")


def parse_truth(path):
    """Header facts and rows of an Inertial Explorer export (columns located by the date token)."""
    header = dict(antenna_lever_arm=None, sensor_lever_arm=None, master_lat_lon_height=None, names=None)
    rows, previous = [], None
    with Path(path).open("r", encoding="latin-1") as stream:
        lines = iter(stream)
        for line in lines:
            if line.startswith("IMU to GNSS Antenna Lever Arms"):
                header["antenna_lever_arm"] = [float(v) for v in TRUTH_LEVER.search(next(lines)).groups()]
            elif line.startswith("IMU->Secondary Sensor Lever Arms"):
                header["sensor_lever_arm"] = [float(v) for v in TRUTH_LEVER.search(next(lines)).groups()]
            elif "Lat, Lon, El Hgt" in line and header["master_lat_lon_height"] is None:
                tokens = line.split("Lat, Lon, El Hgt", 1)[1].replace(",", " ").split()
                d, m, s, d2, m2, s2, h = (float(v) for v in tokens[:7])
                header["master_lat_lon_height"] = [d + m/60 + s/3600, d2 + m2/60 + s2/3600, h]
            elif line.startswith("Week"):
                header["names"] = line.split()
                next(lines)  # units
                break
        if header["names"] is None:
            raise ValueError(f"{path}: no column header line")
        for number, line in enumerate(lines, start=1):
            tokens = line.split()
            if not tokens:
                continue
            if tokens[0] == "Errors/warnings:":
                break                       # the processing-message trailer of the export
            dates = [i for i, t in enumerate(tokens) if DATE_TOKEN.match(t)]
            if len(dates) != 1 or dates[0] < 7:
                raise ValueError(f"{path}: data row {number} has no single date token")
            d = dates[0]
            values = tokens[d + 2:d + 17]   # ECEF xyz, local-level xyz, ECEF velocity, VE VN VU, heading pitch roll
            if len(values) != 15:
                raise ValueError(f"{path}: data row {number} is too short")
            tow = Fraction(tokens[1])
            if previous is not None and tow <= previous:
                raise ValueError(f"{path}: truth time is not strictly increasing at row {number}")
            previous = tow
            rows.append(dict(week=int(tokens[0]), tow=tow, lat=tokens[2], lon=tokens[3], height=tokens[4],
                             q=tokens[5], ecef=values[0:3], vel_enu=values[9:12],
                             heading=values[12], pitch=values[13], roll=values[14]))
    if not rows:
        raise ValueError(f"{path}: no data rows")
    if len({row["week"] for row in rows}) != 1:
        raise ValueError(f"{path}: truth crosses a GPS week boundary")
    return header, rows


def write_reference(rows, keep, dst):
    """PPC header names plus the truth quality, rows at kept epochs only; values as exported."""
    stats = dict(rows_in=len(rows), rows_out=0, quality_counts={})
    with Path(dst).open("w", encoding="utf-8", newline="") as out:
        out.write(",".join(REFERENCE_HEADER) + "\n")
        for row in rows:
            if row["tow"] not in keep:
                continue
            ve, vn, vu = (float(v) for v in row["vel_enu"])
            out.write(",".join([format_tow(row["tow"]), str(row["week"]), row["lat"], row["lon"], row["height"],
                                *row["ecef"], row["roll"], row["pitch"], row["heading"],
                                velocity_text(ve), velocity_text(vn), velocity_text(vu), row["q"]]) + "\n")
            stats["rows_out"] += 1
            stats["quality_counts"][row["q"]] = stats["quality_counts"].get(row["q"], 0) + 1
    return stats


# ---------------------------------------------------------------------------
# imu.csv


def read_imr_header(data):
    if len(data) < IMR_HEADER_BYTES or data[:8] != b"$IMURAW\0":
        raise ValueError("not an Inertial Explorer IMR file")
    endian, = struct.unpack_from("<b", data, 8)
    delta_theta, delta_velocity = struct.unpack_from("<ii", data, 17)
    rate, gyro_scale, accel_scale = struct.unpack_from("<ddd", data, 25)
    time_type, = struct.unpack_from("<i", data, 49)
    time_bias_ms, = struct.unpack_from("<d", data, 57)
    name = data[65:97].split(b"\0", 1)[0].decode("ascii", "replace").strip()
    if endian != 0 or delta_theta != 1 or delta_velocity != 1 or time_type != 2 or time_bias_ms != 0.0:
        raise ValueError(f"unsupported IMR header (endian {endian}, delta flags {delta_theta}/{delta_velocity}, "
                         f"time type {time_type}, time bias {time_bias_ms} ms)")
    if not (rate > 0 and gyro_scale > 0 and accel_scale > 0):
        raise ValueError("IMR header lacks the data rate or scale factors")
    return dict(imu_name=name, rate_hz=rate, gyro_scale_deg=gyro_scale, accel_scale_mps=accel_scale)


def convert_imu(src, dst, week, mounting_deg):
    """STIM300 increments -> rates at the interval midpoints, IMU body -> FLU, linear resample onto the 100 Hz grid.

    An IMR record at time t holds the increments of (t_previous, t]; the rate is their quotient by the
    measured interval and is placed at its midpoint. Returns (stats, grid, one_second_means) where
    one_second_means maps an integer second to the mean FLU (ax, ay, az, gx, gy, gz) of its grid samples.
    """
    rotation = body_to_flu_matrix(mounting_deg)
    data = Path(src).read_bytes()
    header = read_imr_header(data)
    body = memoryview(data)[IMR_HEADER_BYTES:]
    if len(body) % IMR_RECORD.size:
        raise ValueError("IMR body is not a whole number of records")
    gscale, ascale = header["gyro_scale_deg"], header["accel_scale_mps"]
    stats = dict(header, raw_records=len(body)//IMR_RECORD.size, rate_samples=0, grid_samples=0, pairs_over_gap=0,
                 first_raw_tow=None, last_raw_tow=None, first_grid_tow=None, last_grid_tow=None, max_raw_dt_s=0.0,
                 body_to_flu=rotation)
    grid, sums = set(), {}
    with Path(dst).open("w", encoding="utf-8", newline="") as out:
        out.write(PPC_IMU_HEADER)
        t_prev, previous = None, None   # previous = (midpoint time, FLU values)
        for t, gx, gy, gz, ax, ay, az in IMR_RECORD.iter_unpack(body):
            if stats["first_raw_tow"] is None:
                stats["first_raw_tow"] = t
            stats["last_raw_tow"] = t
            if t_prev is None:
                t_prev = t
                continue
            dt = t - t_prev
            if dt <= 0:
                raise ValueError(f"IMU time is not strictly increasing at {t}")
            stats["max_raw_dt_s"] = max(stats["max_raw_dt_s"], dt)
            if dt > MAX_PAIR_GAP_S:
                # The increments of a record that ends a gap cover the whole gap: no usable rate, no bridge.
                stats["pairs_over_gap"] += 1
                t_prev, previous = t, None
                continue
            gyro_b = (gx*gscale/dt, gy*gscale/dt, gz*gscale/dt)          # deg/s
            accel_b = (ax*ascale/dt, ay*ascale/dt, az*ascale/dt)         # m/s^2
            gyro = [sum(rotation[i][j]*gyro_b[j] for j in range(3)) for i in range(3)]
            accel = [sum(rotation[i][j]*accel_b[j] for j in range(3)) for i in range(3)]
            current = (t - dt/2, accel + gyro)
            t_prev = t
            stats["rate_samples"] += 1
            if previous is not None:
                t0, v0 = previous
                t1, v1 = current
                if t1 - t0 > MAX_PAIR_GAP_S:
                    stats["pairs_over_gap"] += 1
                else:
                    first_cs = math.floor(t0*100) + 1        # first grid point strictly after t0
                    last_cs = math.floor(t1*100 + 1e-9)       # last grid point at or before t1
                    for cs in range(first_cs, last_cs + 1):
                        g = cs/100
                        weight = (g - t0)/(t1 - t0)
                        row = [a + (b - a)*weight + 0.0 for a, b in zip(v0, v1)]
                        out.write(f"{cs//100}.{cs % 100:02d}, {week}, " + ", ".join(f"{v:11.8f}" for v in row) + "\n")
                        grid.add(cs)
                        second = (cs + 50)//100
                        acc = sums.setdefault(second, [0.0]*7)
                        for i, v in enumerate(row):
                            acc[i] += v
                        acc[6] += 1
                        stats["grid_samples"] += 1
                        if stats["first_grid_tow"] is None:
                            stats["first_grid_tow"] = g
                        stats["last_grid_tow"] = g
            previous = current
    if stats["grid_samples"] == 0:
        raise ValueError("IMU file gives no grid samples")
    means = {s: [v/acc[6] for v in acc[:6]] for s, acc in sums.items() if acc[6] >= 90}
    return stats, grid, means


def check_imu_axes(means, rows, keep):
    """Correlate the FLU IMU with the truth at 1 Hz: gyro z with the heading rate, accel x with the speed change,
    accel y with the centripetal acceleration. Only the truth heading and ENU velocity are used."""
    by_second = {int(row["tow"]): row for row in rows if row["tow"] in keep and row["tow"].denominator == 1}
    yaw, gz, fwd, ax, lat, ay = [], [], [], [], [], []
    for s, row in by_second.items():
        before, after = by_second.get(s - 1), by_second.get(s + 1)
        if before is None or after is None or s not in means:
            continue
        dh = (float(after["heading"]) - float(before["heading"]) + 540.0) % 360.0 - 180.0
        yaw_rate = -dh/2.0                                         # azimuth clockwise -> rotation about up, deg/s
        speed = lambda r: math.hypot(float(r["vel_enu"][0]), float(r["vel_enu"][1]))
        v = speed(row)
        if v < 3.0:
            continue
        yaw.append(yaw_rate); gz.append(means[s][5])
        fwd.append((speed(after) - speed(before))/2.0); ax.append(means[s][0])
        lat.append(v*math.radians(yaw_rate)); ay.append(means[s][1])

    def corr(a, b):
        n = len(a)
        if n < 30:
            return None
        ma, mb = sum(a)/n, sum(b)/n
        sa = math.sqrt(sum((x - ma)**2 for x in a)); sb = math.sqrt(sum((y - mb)**2 for y in b))
        return sum((x - ma)*(y - mb) for x, y in zip(a, b))/(sa*sb) if sa > 0 and sb > 0 else None

    result = dict(epochs_used=len(yaw), gyro_z_vs_heading_rate=corr(gz, yaw), accel_x_vs_speed_change=corr(ax, fwd),
                  accel_y_vs_centripetal=corr(ay, lat))
    if result["gyro_z_vs_heading_rate"] is None or result["gyro_z_vs_heading_rate"] < MIN_AXIS_CORRELATION:
        raise ValueError(f"IMU axes do not match the truth heading rate: {result}")
    for key in ("accel_x_vs_speed_change", "accel_y_vs_centripetal"):
        if result[key] is None or result[key] <= 0:
            raise ValueError(f"IMU axes do not match the truth ({key}): {result}")
    return result


# ---------------------------------------------------------------------------
# base.obs header


def rewrite_base_position(path, xyz):
    """Set APPROX POSITION XYZ to `xyz` and ANTENNA: DELTA H/E/N to zero; every other byte is kept."""
    lines = Path(path).read_bytes().splitlines(keepends=True)
    done = set()
    for i, line in enumerate(lines):
        label = line[60:].rstrip(b"\r\n").strip()
        ending = line[len(line.rstrip(b"\r\n")):]
        if label == b"APPROX POSITION XYZ":
            lines[i] = ("".join(f"{v:14.4f}" for v in xyz).ljust(60) + "APPROX POSITION XYZ").encode("ascii") + ending
            done.add(label)
        elif label == b"ANTENNA: DELTA H/E/N":
            lines[i] = ("".join(f"{0.0:14.4f}" for _ in range(3)).ljust(60) + "ANTENNA: DELTA H/E/N").encode("ascii") + ending
            done.add(label)
        elif label == b"END OF HEADER":
            break
    if done != {b"APPROX POSITION XYZ", b"ANTENNA: DELTA H/E/N"}:
        raise ValueError(f"{path}: base header lacks APPROX POSITION XYZ or ANTENNA: DELTA H/E/N")
    Path(path).write_bytes(b"".join(lines))


# ---------------------------------------------------------------------------


def find_one(directory, pattern):
    matches = sorted(p for p in Path(directory).iterdir() if re.fullmatch(pattern, p.name, re.IGNORECASE))
    if len(matches) != 1:
        raise ValueError(f"{directory}: expected one file matching {pattern}, found {[p.name for p in matches]}")
    return matches[0]


def convert(run, run_dir, output_root):
    if run not in RUNS:
        raise ValueError(f"run must be one of {tuple(RUNS)}")
    run_dir = Path(run_dir)
    if run_dir.name != run + RUN_FOLDER_SUFFIX:
        raise ValueError(f"run folder must be named {run + RUN_FOLDER_SUFFIX}")
    yy = run[9:11]
    inputs = dict(readme=find_one(run_dir, r"README\.xml"), rover_obs=find_one(run_dir, rf"ROVE_03\.{yy}O"),
                  base_obs=find_one(run_dir, rf"BASE\.{yy}O"), nav=find_one(run_dir, rf"brdm\d{{4}}\.{yy}p"),
                  imu=find_one(run_dir, r"STIM300\.imr"), reference=find_one(run_dir, r"Rove_03_GroundTruth\.txt"))
    target = Path(output_root)/"smartpnt"/f"{run}_rove03_stim300"
    manifest_path = target.with_name(target.name + ".manifest.json")
    if target.exists() or manifest_path.exists():
        raise ValueError(f"output already exists: {target}")

    readme = read_readme(inputs["readme"])
    truth_header, truth = parse_truth(inputs["reference"])
    # The truth must be at the primary antenna (the one ROVE_03 uses) and computed from the README base.
    if truth_header["antenna_lever_arm"] is None or truth_header["sensor_lever_arm"] != truth_header["antenna_lever_arm"]:
        raise ValueError(f"truth is not exported at the GNSS antenna: {truth_header}")
    master = geodetic_to_ecef(*truth_header["master_lat_lon_height"])
    marker_offset = math.dist(master, readme["base_marker_ecef"])
    if marker_offset > 0.005:
        raise ValueError(f"README base marker and the truth master position differ by {marker_offset:.4f} m")
    week = truth[0]["week"]

    target.mkdir(parents=True)
    stats = {}
    stats["imu.csv"], grid, means = convert_imu(inputs["imu"], target/"imu.csv", week, readme["stim300_mounting_deg"])
    # A rover epoch is kept iff it has an exact truth row and an exact IMU grid sample (then base, below).
    truth_times = {row["tow"] for row in truth}
    keep = {t for t in truth_times if (t*100).denominator == 1 and int(t*100) in grid}
    stats["rover.obs"], kept = filter_rover_obs(inputs["rover_obs"], target/"rover.obs", keep, truth_times,
                                              drop_blank_lines_before_first_epoch=True)   # Data11 has one
    if not kept:
        raise ValueError("no rover epoch has both a truth row and an IMU sample")
    if stats["rover.obs"]["week"] != week:
        raise ValueError("rover and truth are in different GPS weeks")
    stats["base.obs"], base_times = assemble_base([inputs["base_obs"]], target/"base.obs", (kept[0], kept[-1]))
    base_xyz = base_phase_centre(readme["base_marker_ecef"], readme["base_antenna_height_m"])
    rewrite_base_position(target/"base.obs", base_xyz)
    stats["base.obs"]["written_position_ecef"] = base_xyz
    kept_set = set(kept)
    stats["reference.csv"] = write_reference(truth, kept_set, target/"reference.csv")
    shutil.copyfile(inputs["nav"], target/"base.nav")
    stats["imu_axis_check"] = check_imu_axes(means, truth, kept_set)
    stats["verification"] = verify_converted(target)
    if (stats["verification"]["rover_epochs_without_truth_row"] or stats["verification"]["rover_epochs_without_imu_sample"]):
        raise ValueError(f"converted rover epochs lack truth or IMU samples: {stats['verification']}")
    lever = readme["stim300_lever_arm_vehicle_m"]
    names = ("rover.obs", "base.obs", "base.nav", "imu.csv", "reference.csv")
    manifest = dict(
        schema="libgnsspp.smartpnt_pos_ppc_conversion.v1", contract=RUNS[run], run=run, converter=pin(__file__),
        raw_inputs={key: pin(path) for key, path in inputs.items()},
        outputs={name: pin(target/name) for name in names},
        replay=dict(lever_arm_flu_m=[lever[1], -lever[0], lever[2]],
                    note="STIM300 to the ROVE_03 antenna; README vehicle frame (x right, y forward, z up) -> FLU"),
        conventions=dict(
            time="GPST throughout (IMR time type 2, RINEX GPS, truth GPS week and seconds); no offset applied",
            imu_axes=f"README mounting {readme['stim300_mounting_deg']} deg (Z-X-Y, vehicle to body) gives the "
                     f"body-to-FLU matrix {stats['imu.csv']['body_to_flu']}; checked against the truth heading rate "
                     "and speed change (imu_axis_check)",
            imu_units="IMR increments times the scale factor over the measured interval: deg/s and m/s^2, "
                      "placed at the interval midpoint and linearly resampled onto the 100 Hz grid",
            truth="Inertial Explorer smoothed GNSS/INS solution exported at the primary antenna (ROVE_01/ROVE_03 "
                  "share it); latitude, longitude, height, ECEF and ENU velocity as exported; heading is the "
                  "azimuth clockwise from north, roll and pitch as exported",
            base_position="README base marker plus the antenna height along the ellipsoidal up, i.e. the L1 phase "
                          "centre the truth used; written to APPROX POSITION XYZ with ANTENNA: DELTA H/E/N zeroed",
            navigation="brdm file copied unchanged"),
        stats=stats)
    manifest_path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", choices=RUNS, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        manifest = convert(args.run, args.run_dir, args.output_root)
    except (ValueError, OSError) as error:
        print(f"convert_smartpnt_pos_to_ppc_layout: {error}", file=sys.stderr)
        return 2
    print(json.dumps(dict(run=args.run, replay=manifest["replay"], stats=manifest["stats"]), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
