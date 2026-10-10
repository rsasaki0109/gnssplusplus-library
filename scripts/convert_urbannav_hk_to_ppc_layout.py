#!/usr/bin/env python3
"""Convert raw UrbanNav Hong Kong files (NovAtel Flexpak6 rover, HKSC base) to the PPC run layout.

Implements the "Conversion to the PPC run layout" section of
docs/online_pva_default_switch_holdout_v2.md. Pure standard library (the optional
``hatanaka`` package is needed only to unpack Compact RINEX base files). The raw files are
read as data only; nothing from them is executed.

Inputs (explicit paths, any location):
    --rover-obs   UrbanNav-HK-<Run>.novatel.flexpak6.obs          (RINEX 3, GPS time, 1 Hz)
    --reference   UrbanNav-HK-<Run> ground-truth text file         (1 Hz, D M S positions, body-frame velocity)
    --imu         Xsens IMU rosbag CSV (xsense_imu_*.csv)
    --base-obs    HKSC hourly 1 Hz observation file(s) covering the run (.rnx, .rnx.gz, .crx.gz or .crx);
                  the output keeps the epochs from the first to the last kept rover epoch
    --nav         HKSC daily broadcast files GN RN EN CN (.rnx or .rnx.gz)

Output: <output-root>/urbannav/<run>_novatel/{rover.obs,base.obs,base.nav,imu.csv,reference.csv}
plus <run>_novatel.manifest.json beside that directory (SHA256 of every raw input and output).
"""
import argparse
import gzip
import json
import math
from fractions import Fraction
from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from convert_urbannav_to_ppc_layout import (  # noqa: E402  shared, unchanged helpers of the Tokyo converter
    PPC_IMU_HEADER, PPC_REFERENCE_HEADER, gps_tow, pin, split_rinex_header)

RUNS = ("HKDeepUrban1", "HKHarshUrban1")
ROVERS = ("novatel",)
CONTRACT = "docs/online_pva_default_switch_holdout_v2.md"

LEAP_SECONDS = 18                    # GPST - UTC from 2017-01-01 to 2021; checked against the truth time columns
UNIX_TO_GPS_NS = (315964800 - LEAP_SECONDS)*10**9  # gps_ns = unix_ns - UNIX_TO_GPS_NS
WEEK_NS = 604800*10**9
GRID_NS = 10_000_000                 # 100 Hz IMU grid, t = k * 0.01 s of GPS time of week, as the PPC imu.csv
MAX_PAIR_GAP_NS = 100_000_000        # raw samples further apart than this are not bridged (as the Tokyo converter)

IMU_RAW_COLUMNS = ("field.header.stamp", "field.header.frame_id",
                   "field.angular_velocity.x", "field.angular_velocity.y", "field.angular_velocity.z",
                   "field.linear_acceleration.x", "field.linear_acceleration.y", "field.linear_acceleration.z")
TRUTH_HEADER = ("UTCTime", "Week", "GPSTime", "Latitude", "Longitude", "H-Ell", "VelBdyX", "VelBdyY", "VelBdyZ",
                "AccBdyX", "AccBdyY", "AccBdyZ", "Roll", "Pitch", "Heading", "Q")
TRUTH_FIELDS = 20                    # Latitude and Longitude are three tokens each (D M S)
REFERENCE_HEADER = PPC_REFERENCE_HEADER + ("Truth Quality (Q)",)

# WGS84
A_WGS84 = 6378137.0
E2_WGS84 = 0.00669437999014


# ---------------------------------------------------------------------------
# reference.csv


def dms_to_degrees(degrees, minutes, seconds):
    """Exact (Fraction) decimal degrees of a signed D M S triple; the sign is the sign of the D token."""
    negative = degrees.strip().startswith("-")
    value = abs(Fraction(degrees)) + Fraction(minutes)/60 + Fraction(seconds)/3600
    return -value if negative else value


def euler_matrix(roll, pitch, heading):
    """FRD body to NED, 3-2-1, angles in degrees, heading clockwise from north (same as gnss_pva_metrics)."""
    r, p, h = map(math.radians, (roll, pitch, heading))
    cr, sr, cp, sp, ch, sh = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(h), math.sin(h)
    return [[ch*cp, ch*sp*sr-sh*cr, ch*sp*cr+sh*sr],
            [sh*cp, sh*sp*sr+ch*cr, sh*sp*cr-ch*sr], [-sp, cp*sr, cp*cr]]


def body_velocity_to_enu(vel_x, vel_y, vel_z, roll, pitch, heading):
    """Truth body velocity to ENU.

    The truth body frame is x right, y forward, z up (proven truth-vs-truth against the differentiated
    truth position, see the contract), so FRD = (y, x, -z); then the 3-2-1 attitude rotates FRD to NED.
    """
    frd = (vel_y, vel_x, -vel_z)
    matrix = euler_matrix(roll, pitch, heading)
    north, east, down = (sum(matrix[i][j]*frd[j] for j in range(3)) for i in range(3))
    return east, north, -down


def geodetic_to_ecef(lat_deg, lon_deg, height):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = A_WGS84/math.sqrt(1 - E2_WGS84*math.sin(lat)**2)
    return ((n + height)*math.cos(lat)*math.cos(lon), (n + height)*math.cos(lat)*math.sin(lon),
            (n*(1 - E2_WGS84) + height)*math.sin(lat))


def velocity_text(value):
    """Six decimals; a value that rounds to zero is written without a sign."""
    return "0.000000" if abs(value) < 5e-7 else f"{value:.6f}"


def format_tow(value):
    """Exact decimal text of a Fraction time of week (integer seconds are written with one decimal)."""
    if value.denominator == 1:
        return f"{int(value)}.0"
    text = f"{float(value):.6f}".rstrip("0")
    if Fraction(text) != value:
        raise ValueError(f"time {value} is not a whole number of microseconds")
    return text


def parse_truth(path):
    """Parse the raw truth text file into rows (exact times, raw text for angles) and check its time columns."""
    rows, previous, week = [], None, None
    with Path(path).open("r", encoding="utf-8-sig") as stream:
        header = stream.readline().split()
        if tuple(header) != TRUTH_HEADER:
            raise ValueError(f"unexpected truth header: {header}")
        stream.readline()  # units line
        for number, line in enumerate(stream, start=3):
            if not line.strip():
                continue
            f = line.split()
            if len(f) != TRUTH_FIELDS:
                raise ValueError(f"truth line {number}: expected {TRUTH_FIELDS} fields, got {len(f)}")
            utc, tow = Fraction(f[0]), Fraction(f[2])
            row_week = Fraction(f[1])
            if row_week.denominator != 1:
                raise ValueError(f"truth line {number}: non-integer GPS week")
            row_week = int(row_week)
            if week is None:
                week = row_week
            elif row_week != week:
                raise ValueError(f"truth line {number}: GPS week changes")
            if previous is not None and tow <= previous:
                raise ValueError(f"truth line {number}: time is not strictly increasing")
            previous = tow
            # The GPS columns must be the UTC column shifted by the GPS-UTC leap seconds (18 s in 2021).
            if utc - 315964800 + LEAP_SECONDS != row_week*604800 + tow:
                raise ValueError(f"truth line {number}: UTCTime and GPSTime disagree by other than {LEAP_SECONDS} leap seconds")
            rows.append(dict(
                tow=tow, week=row_week, lat=dms_to_degrees(*f[3:6]), lon=dms_to_degrees(*f[6:9]), height=f[9],
                velocity=tuple(float(v) for v in f[10:13]), roll=f[16], pitch=f[17], heading=f[18], quality=f[19]))
    if not rows:
        raise ValueError("truth file has no rows")
    return rows


def write_reference(rows, keep, dst):
    """PPC header names, rows at kept epochs only; positions as decimal degrees, velocity rotated to ENU."""
    stats = dict(rows_in=len(rows), rows_out=0, rows_dropped=0, first_tow_out=None, last_tow_out=None, week=rows[0]["week"],
                 quality_counts={})
    with Path(dst).open("w", encoding="utf-8", newline="") as out:
        out.write(",".join(REFERENCE_HEADER) + "\n")
        for row in rows:
            if row["tow"] not in keep:
                stats["rows_dropped"] += 1
                continue
            lat, lon = float(row["lat"]), float(row["lon"])
            x, y, z = geodetic_to_ecef(lat, lon, float(row["height"]))
            east, north, up = body_velocity_to_enu(*row["velocity"], float(row["roll"]), float(row["pitch"]), float(row["heading"]))
            out.write(",".join((format_tow(row["tow"]), str(row["week"]), f"{lat:.10f}", f"{lon:.10f}", row["height"],
                                f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", row["roll"], row["pitch"], row["heading"],
                                velocity_text(east), velocity_text(north), velocity_text(up), row["quality"])) + "\n")
            stats["rows_out"] += 1
            stats["quality_counts"][row["quality"]] = stats["quality_counts"].get(row["quality"], 0) + 1
            if stats["first_tow_out"] is None:
                stats["first_tow_out"] = float(row["tow"])
            stats["last_tow_out"] = float(row["tow"])
    return stats


# ---------------------------------------------------------------------------
# imu.csv


def convert_imu(src, dst):
    """Xsens RFU -> FLU, rad/s -> deg/s, linear resample onto the 100 Hz TOW grid (no extrapolation).

    Returns (stats, grid) where grid is the set of written grid times in centiseconds of GPS time of week.
    The raw sensor frame is x right, y forward, z up (proven against the truth, see the contract), so
    FLU = (y, -x, z) for the accelerometer and the gyro.
    """
    stats = dict(raw_samples=0, grid_samples=0, grid_points_skipped_gap=0, grid_points_skipped_on_raw_sample=0,
                 pairs_over_gap=0, week=None, first_raw_tow=None, last_raw_tow=None, first_grid_tow=None, last_grid_tow=None,
                 max_raw_dt_s=0.0)
    grid, to_deg = set(), math.degrees(1.0)
    with Path(src).open("r", encoding="utf-8", newline="") as stream, Path(dst).open("w", encoding="utf-8", newline="") as out:
        header = stream.readline().rstrip("\r\n").split(",")
        index = {}
        for name in IMU_RAW_COLUMNS:
            if header.count(name) != 1:
                raise ValueError(f"raw IMU header lacks exactly one column {name!r}")
            index[name] = header.index(name)
        i_stamp, i_frame = index["field.header.stamp"], index["field.header.frame_id"]
        i_gyro = [index[f"field.angular_velocity.{a}"] for a in "xyz"]
        i_acc = [index[f"field.linear_acceleration.{a}"] for a in "xyz"]
        out.write(PPC_IMU_HEADER)
        previous = None  # (t_ns, [ax, ay, az, gx, gy, gz]) in FLU, deg/s, t_ns = ns of GPS time since the GPS epoch
        for number, line in enumerate(stream, start=2):
            if not line.strip():
                continue
            fields = line.rstrip("\r\n").split(",")
            if len(fields) != len(header):
                raise ValueError(f"IMU line {number}: expected {len(header)} fields")
            if fields[i_frame] != "/imu":
                raise ValueError(f"IMU line {number}: unexpected frame_id {fields[i_frame]!r}")
            gps_ns = int(fields[i_stamp]) - UNIX_TO_GPS_NS
            week, t_ns = divmod(gps_ns, WEEK_NS)
            if stats["week"] is None:
                stats["week"] = week
            elif week != stats["week"]:
                raise ValueError(f"IMU line {number}: GPS week changes")
            gx, gy, gz = (float(fields[i]) for i in i_gyro)
            ax, ay, az = (float(fields[i]) for i in i_acc)
            values = [ay, -ax, az, gy*to_deg, -gx*to_deg, gz*to_deg]  # RFU -> FLU
            stats["raw_samples"] += 1
            if previous is None:
                stats["first_raw_tow"] = t_ns/1e9
                if t_ns % GRID_NS == 0:
                    stats["grid_points_skipped_on_raw_sample"] += 1  # first sample: no pair ends on it
            else:
                t0, v0 = previous
                if t_ns <= t0:
                    raise ValueError(f"IMU line {number}: IMU time is not strictly increasing")
                dt = t_ns - t0
                stats["max_raw_dt_s"] = max(stats["max_raw_dt_s"], dt/1e9)
                first_k = t0//GRID_NS + 1   # first grid point strictly after t0
                last_k = t_ns//GRID_NS      # last grid point at or before t_ns
                if dt > MAX_PAIR_GAP_NS:
                    stats["pairs_over_gap"] += 1
                    stats["grid_points_skipped_gap"] += max(0, last_k - first_k + 1)
                else:
                    for k in range(first_k, last_k + 1):
                        g = k*GRID_NS
                        weight = (g - t0)/dt
                        row = [a + (b - a)*weight + 0.0 for a, b in zip(v0, values)]
                        centiseconds = g//GRID_NS
                        out.write(f"{centiseconds//100}.{centiseconds % 100:02d}, {week}, " + ", ".join(f"{v:11.8f}" for v in row) + "\n")
                        grid.add(centiseconds)
                        stats["grid_samples"] += 1
                        if stats["first_grid_tow"] is None:
                            stats["first_grid_tow"] = g/1e9
                        stats["last_grid_tow"] = g/1e9
            previous = (t_ns, values)
            stats["last_raw_tow"] = t_ns/1e9
    if stats["raw_samples"] < 2:
        raise ValueError("IMU file has fewer than two samples")
    first_ns, last_ns = round(stats["first_raw_tow"]*1e9), round(stats["last_raw_tow"]*1e9)
    stats["grid_points_in_raw_span"] = last_ns//GRID_NS - (-(-first_ns//GRID_NS)) + 1
    stats["grid_points_skipped"] = stats["grid_points_in_raw_span"] - stats["grid_samples"]
    if stats["grid_points_skipped"] != stats["grid_points_skipped_gap"] + stats["grid_points_skipped_on_raw_sample"]:
        raise ValueError("internal error: skipped grid point accounting does not close")
    stats["week_of_samples"] = stats.pop("week")
    return stats, grid


# ---------------------------------------------------------------------------
# rover.obs and base.obs


def is_epoch_line(line):
    """True for a RINEX 3 epoch line with a date; an event record with a blank time (">   4  87") is not one."""
    fields = line[1:].split()
    return line.startswith(b">") and len(fields) >= 7 and len(fields[0]) == 4 and fields[0].isdigit()


def epoch_time(line):
    """(week, tow Fraction) of a RINEX 3 epoch line given as bytes."""
    if not is_epoch_line(line):
        raise ValueError(f"not an observation epoch line: {line[:60]!r}")
    fields = line[1:].split()
    year, month, day, hour, minute = (int(v) for v in fields[:5])
    return gps_tow(year, month, day, hour, minute, fields[5].decode("ascii"))


def epoch_flag(line):
    """Epoch flag of a RINEX 3 epoch line (0 ok, 1 power failure; 2-6 are event records)."""
    return int(line[1:].split()[6])


def filter_rover_obs(src, dst, keep, truth_times):
    """Copy the epochs whose GPS time of week is in `keep`; header and records byte-preserved.

    `truth_times` only attributes dropped epochs to a reason. Returns (stats, kept) with kept the
    list of kept epoch times in order.
    """
    stats = dict(epochs_in=0, epochs_out=0, epochs_dropped=0, epochs_without_truth_row=0,
                 epochs_with_truth_row_without_imu_sample=0, first_tow_in=None, last_tow_in=None,
                 first_tow_out=None, last_tow_out=None, week=None, max_gap_out_s=0.0)
    kept, previous = [], None
    with Path(src).open("rb") as stream, Path(dst).open("wb") as out:
        header, time_system = split_rinex_header(stream)
        if time_system != "GPS":
            raise ValueError(f"expected GPS time system in TIME OF FIRST OBS, got {time_system!r}")
        out.writelines(header)
        keeping = False
        for line in stream:
            if line.startswith(b">"):
                week, tow = epoch_time(line)
                if epoch_flag(line) not in (0, 1):
                    raise ValueError(f"event record in the rover observations at week {week} tow {float(tow)}")
                if previous is not None and (week, tow) <= previous:
                    raise ValueError(f"non-monotonic observation epoch at week {week} tow {float(tow)}")
                previous = (week, tow)
                if stats["week"] is None:
                    stats["week"] = week
                elif stats["week"] != week:
                    raise ValueError("observation epochs cross a GPS week boundary")
                stats["epochs_in"] += 1
                if stats["first_tow_in"] is None:
                    stats["first_tow_in"] = float(tow)
                stats["last_tow_in"] = float(tow)
                keeping = tow in keep
                if keeping:
                    if kept:
                        stats["max_gap_out_s"] = max(stats["max_gap_out_s"], float(tow - kept[-1]))
                    kept.append(tow)
                else:
                    stats["epochs_dropped"] += 1
                    stats["epochs_without_truth_row" if tow not in truth_times
                          else "epochs_with_truth_row_without_imu_sample"] += 1
            elif previous is None:
                raise ValueError("observation data before the first epoch line")
            if keeping:
                out.write(line)
    stats["epochs_out"] = len(kept)
    if kept:
        stats["first_tow_out"], stats["last_tow_out"] = float(kept[0]), float(kept[-1])
    return stats, kept


def read_text_file(path):
    """Bytes of a RINEX file: .gz is inflated, Compact RINEX (CRINEX) is expanded with the `hatanaka` package."""
    data = Path(path).read_bytes()
    while data[:2] == b"\x1f\x8b":
        data = gzip.decompress(data)
    if b"CRINEX VERS" in data[:100]:
        try:
            import hatanaka
        except ImportError as error:
            raise ValueError("Compact RINEX input needs the `hatanaka` package (pip install hatanaka)") from error
        data = hatanaka.decompress(data)
    return data


def parse_obs_header(data):
    """(lines, index of the first record line, time system of TIME OF FIRST OBS, APPROX POSITION XYZ)."""
    lines = data.splitlines(keepends=True)
    time_system, approx = None, None
    for number, line in enumerate(lines):
        label = line[60:].strip()
        if label == b"RINEX VERSION / TYPE" and b"OBSERVATION DATA" not in line:
            raise ValueError("base file is not an observation file")
        if label == b"TIME OF FIRST OBS":
            time_system = line[48:51].decode("ascii", "replace").strip()
        if label == b"APPROX POSITION XYZ":
            approx = [float(v) for v in line[:42].split()]
        if label == b"END OF HEADER":
            return lines, number + 1, time_system, approx
    raise ValueError("RINEX header has no END OF HEADER")


def observation_blocks(lines, path):
    """Split the record lines of an observation file into blocks, one per epoch or event record."""
    blocks = []
    for line in lines:
        if line.startswith(b">"):
            blocks.append([line])
        elif blocks:
            blocks[-1].append(line)
        else:
            raise ValueError(f"{path}: observation data before the first epoch line")
    return blocks


def assemble_base(paths, dst, span):
    """base.obs from one or more hourly files: the header of the first file and the epoch records, byte for
    byte, whose GPS time of week lies in `span` = (first, last) inclusive; several files must continue
    strictly in time. Everything else is dropped: epochs outside the span, because the replay buffers at
    most 16 pending base epochs and reads every base epoch up to the first rover epoch before it starts;
    and event records (flag other than 0 or 1, e.g. the closing "header information follows" block of an
    hourly file), because the replay's RINEX reader treats one as the end of the file."""
    stats = dict(files=len(paths), epochs_in_files=0, epochs_out=0, epochs_dropped_before_span=0, epochs_dropped_after_span=0,
                 event_records_dropped=0, first_tow_out=None, last_tow_out=None, week=None, approx_position_ecef=None)
    first_lines, kept, previous = None, [], None
    for path in paths:
        lines, start, time_system, approx = parse_obs_header(read_text_file(path))
        if time_system != "GPS":
            raise ValueError(f"{path}: expected GPS time system in TIME OF FIRST OBS, got {time_system!r}")
        if approx is None or len(approx) != 3 or not any(approx):
            raise ValueError(f"{path}: base header has no APPROX POSITION XYZ")
        if first_lines is None:
            first_lines, stats["approx_position_ecef"] = lines[:start], approx
        elif approx != stats["approx_position_ecef"]:
            raise ValueError(f"{path}: base position differs between files")
        if lines[-1:] and not lines[-1].endswith((b"\n", b"\r")):
            lines[-1] += b"\n"
        for block in observation_blocks(lines[start:], path):
            if not is_epoch_line(block[0]) or epoch_flag(block[0]) not in (0, 1):
                stats["event_records_dropped"] += 1
                continue
            week, tow = epoch_time(block[0])
            if stats["week"] is None:
                stats["week"] = week
            elif stats["week"] != week:
                raise ValueError("base epochs cross a GPS week boundary")
            if previous is not None and tow <= previous:
                raise ValueError(f"{path}: base epochs are not strictly increasing")
            previous = tow
            stats["epochs_in_files"] += 1
            if tow < span[0]:
                stats["epochs_dropped_before_span"] += 1
            elif tow > span[1]:
                stats["epochs_dropped_after_span"] += 1
            else:
                kept.append((tow, block))
    with Path(dst).open("wb") as out:
        out.writelines(first_lines)
        for _, block in kept:
            out.writelines(block)
    stats["epochs_out"] = len(kept)
    if kept:
        stats["first_tow_out"], stats["last_tow_out"] = float(kept[0][0]), float(kept[-1][0])
    return stats, {tow for tow, _ in kept}


# ---------------------------------------------------------------------------
# base.nav

NAV_ORDER = "GRECJIS"
NAV_KEEP_LABELS = (b"IONOSPHERIC CORR", b"TIME SYSTEM CORR")
NAV_RECORD = re.compile(rb"^([GRECJIS])[ 0-9][0-9] ")


def merge_navigation(paths, dst):
    """One RINEX 3 mixed navigation file from the per-system daily files: a new type line, the
    ionosphere and time-system header lines of every input, then the record bodies unchanged."""
    files = []
    for path in paths:
        lines = read_text_file(path).splitlines(keepends=True)
        header, body, in_header = [], [], True
        for line in lines:
            if in_header:
                header.append(line)
                in_header = line[60:].strip() != b"END OF HEADER"
            else:
                body.append(line if line.endswith(b"\n") else line + b"\n")
        if in_header:
            raise ValueError(f"{path}: navigation header has no END OF HEADER")
        first = header[0]
        if first[60:].strip() != b"RINEX VERSION / TYPE" or first[20:21] != b"N":
            raise ValueError(f"{path}: not a RINEX navigation file")
        system = first[40:41].decode("ascii")
        if system not in NAV_ORDER:
            raise ValueError(f"{path}: unsupported navigation system {system!r}")
        files.append((NAV_ORDER.index(system), system, first[:9].decode("ascii"), header, body, str(path)))
    files.sort(key=lambda item: item[0])
    systems = [item[1] for item in files]
    if len(set(systems)) != len(systems):
        raise ValueError(f"navigation systems repeat: {systems}")
    if len({item[2] for item in files}) != 1:
        raise ValueError("navigation files have different RINEX versions")
    version = files[0][2]
    out_header = [f"{version:>9}{'':11}{'N: GNSS NAV DATA':<20}{'M: MIXED':<20}RINEX VERSION / TYPE\n".encode("ascii")]
    program = next((l for l in files[0][3] if l[60:].strip() == b"PGM / RUN BY / DATE"), None)
    if program is not None:
        out_header.append(program.rstrip(b"\r\n") + b"\n")
    out_header.append(f"{'Merged ' + '/'.join(systems) + ' broadcast files; record bodies unchanged':<60}COMMENT\n".encode("ascii"))
    leap = None
    for _, _, _, header, _, _ in files:
        for line in header:
            label = line[60:].strip()
            if label in NAV_KEEP_LABELS:
                out_header.append(line.rstrip(b"\r\n") + b"\n")
            elif label == b"LEAP SECONDS" and leap is None:
                leap = line.rstrip(b"\r\n") + b"\n"
    if leap is not None:
        out_header.append(leap)
    out_header.append(files[0][3][-1].rstrip(b"\r\n") + b"\n")  # END OF HEADER
    records = {}
    with Path(dst).open("wb") as out:
        out.writelines(out_header)
        for _, system, _, _, body, name in files:
            count = 0
            for line in body:
                m = NAV_RECORD.match(line)
                if m:
                    if m.group(1).decode() != system:
                        raise ValueError(f"{name}: record of system {m.group(1).decode()} in a {system} file")
                    count += 1
            records[system] = count
            out.writelines(body)
    return dict(files=len(paths), systems=systems, records_per_system=records, header_lines=len(out_header))


# ---------------------------------------------------------------------------
# offline verification of a converted directory (no estimator)


def verify_converted(directory):
    """Re-read the converted files: every rover epoch must have an exact truth row and an IMU grid sample."""
    directory = Path(directory)
    reference = set()
    with (directory/"reference.csv").open("r", encoding="utf-8") as stream:
        if tuple(stream.readline().rstrip("\n").split(",")) != REFERENCE_HEADER:
            raise ValueError("converted reference header is not the frozen one")
        for line in stream:
            reference.add(Fraction(line.split(",", 1)[0]))
    imu = set()
    with (directory/"imu.csv").open("r", encoding="utf-8") as stream:
        if stream.readline() != PPC_IMU_HEADER:
            raise ValueError("converted IMU header is not the PPC one")
        for line in stream:
            whole, frac = line.split(",", 1)[0].split(".")
            imu.add(int(whole)*100 + int(frac))
    rover = []
    with (directory/"rover.obs").open("rb") as stream:
        split_rinex_header(stream)
        for line in stream:
            if line.startswith(b">"):
                rover.append(epoch_time(line)[1])
    base = set()
    with (directory/"base.obs").open("rb") as stream:
        for line in stream:
            if is_epoch_line(line):
                base.add(epoch_time(line)[1])
    without_truth = [float(t) for t in rover if t not in reference]
    without_imu = [float(t) for t in rover if (t*100).denominator != 1 or int(t*100) not in imu]
    without_base = [float(t) for t in rover if t not in base]
    return dict(rover_epochs=len(rover), reference_rows=len(reference), imu_grid_samples=len(imu),
                rover_epochs_without_truth_row=len(without_truth), rover_epochs_without_imu_sample=len(without_imu),
                rover_epochs_without_exact_base_epoch=len(without_base), epochs_without_exact_base_epoch=without_base[:20],
                reference_rows_without_rover_epoch=len(reference - set(rover)))


# ---------------------------------------------------------------------------


def convert(run, rover, rover_obs, reference, imu, base_obs, nav, output_root):
    if run not in RUNS or rover not in ROVERS:
        raise ValueError(f"run must be one of {RUNS} and rover one of {ROVERS}")
    base_obs, nav = [Path(p) for p in base_obs], [Path(p) for p in nav]
    inputs = dict(rover_obs=Path(rover_obs), reference=Path(reference), imu=Path(imu))
    missing = [str(p) for p in [*inputs.values(), *base_obs, *nav] if not p.is_file()]
    if missing or not base_obs or not nav:
        raise ValueError(f"missing raw input(s): {missing or 'base-obs/nav lists are empty'}")
    output_root = Path(output_root)
    target = output_root/"urbannav"/f"{run}_{rover}"
    manifest_path = target.with_name(target.name + ".manifest.json")
    if target.exists() or manifest_path.exists():
        raise ValueError(f"output already exists: {target}")
    truth = parse_truth(inputs["reference"])
    target.mkdir(parents=True)
    stats = {}
    stats["imu.csv"], grid = convert_imu(inputs["imu"], target/"imu.csv")
    if stats["imu.csv"]["week_of_samples"] != truth[0]["week"]:
        raise ValueError("IMU and truth are in different GPS weeks")
    # A rover epoch is kept iff it has an exact truth row and an exact IMU grid sample.
    truth_times = {row["tow"] for row in truth}
    keep = {t for t in truth_times if (t*100).denominator == 1 and int(t*100) in grid}
    stats["rover.obs"], kept = filter_rover_obs(inputs["rover_obs"], target/"rover.obs", keep, truth_times)
    kept_set = set(kept)
    stats["reference.csv"] = write_reference(truth, kept_set, target/"reference.csv")
    stats["base.obs"], _ = assemble_base(base_obs, target/"base.obs", (kept[0], kept[-1]))
    stats["base.nav"] = merge_navigation(nav, target/"base.nav")
    stats["verification"] = verify_converted(target)
    names = ("rover.obs", "base.obs", "base.nav", "imu.csv", "reference.csv")
    if stats["verification"]["rover_epochs_without_truth_row"] or stats["verification"]["rover_epochs_without_imu_sample"]:
        raise ValueError(f"converted rover epochs lack truth or IMU samples: {stats['verification']}")
    manifest = dict(
        schema="libgnsspp.urbannav_hk_ppc_conversion.v1", contract=CONTRACT, run=run, rover=rover, converter=pin(__file__),
        raw_inputs=dict(zip(names, [pin(inputs["rover_obs"]), [pin(p) for p in base_obs], [pin(p) for p in nav],
                                    pin(inputs["imu"]), pin(inputs["reference"])])),
        outputs={name: pin(target/name) for name in names},
        conventions=dict(
            time="GPST; truth UTC column minus 315964800 plus 18 leap seconds equals the truth GPS columns (checked per row); "
                 "IMU header.stamp is Unix UTC and is converted with the same 18 s; no time offset applied",
            imu_axes="raw Xsens x right, y forward, z up (proven against truth); written FLU = (y, -x, z) for accelerometer and gyro",
            imu_units="accelerometer m/s^2 unchanged; gyro rad/s -> deg/s",
            truth_body_frame="x right, y forward, z up (proven against differentiated truth position); FRD = (y, x, -z), "
                             "3-2-1 attitude to NED, ENU = (E, N, -D)",
            truth_ecef="WGS84 from latitude, longitude and ellipsoidal height",
            lever_arm="none applied here; the replay uses zero for urbannav"),
        stats=stats)
    manifest_path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", choices=RUNS, required=True)
    parser.add_argument("--rover", choices=ROVERS, default="novatel")
    parser.add_argument("--rover-obs", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--base-obs", type=Path, nargs="+", required=True)
    parser.add_argument("--nav", type=Path, nargs="+", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        manifest = convert(args.run, args.rover, args.rover_obs, args.reference, args.imu, args.base_obs, args.nav, args.output_root)
    except (ValueError, OSError) as error:
        print(f"convert_urbannav_hk_to_ppc_layout: {error}", file=sys.stderr)
        return 2
    print(json.dumps(dict(run=args.run, rover=args.rover, stats=manifest["stats"]), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
