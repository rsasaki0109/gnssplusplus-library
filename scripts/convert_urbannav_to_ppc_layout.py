#!/usr/bin/env python3
"""Convert raw UrbanNav Tokyo files to the PPC run layout.

Implements the "Conversion to the PPC run layout" section of
docs/online_pva_default_switch_holdout_v1.md. Pure standard library. The raw
files are read as data only; nothing from them is executed.

Input names under --raw-dir (``<run>`` is Odaiba or Shinjuku):
    <run>_rover_trimble.obs  <run>_base_trimble.obs  <run>_base.nav
    <run>_imu.csv  <run>_reference.csv

Output: <output-root>/urbannav/<run>_trimble/{rover.obs,base.obs,base.nav,
imu.csv,reference.csv} plus <run>_trimble.manifest.json beside that directory
(SHA256 of every raw input and output file). Only the Trimble rover is used; the
u-blox rover is excluded by the contract (its epochs are off the 0.2 s grid).
"""
import argparse
import datetime
import hashlib
import json
import math
from fractions import Fraction
from pathlib import Path
import shutil
import sys

RUNS = ("Odaiba", "Shinjuku")
ROVERS = ("trimble",)
GPS_EPOCH = datetime.date(1980, 1, 6)

GRID_US = 20_000          # 50 Hz IMU grid, t = k * 0.02 s of GPS time of week
MAX_PAIR_GAP_US = 100_000  # raw samples further apart than this are not bridged
EPOCH_STEP = Fraction(1, 5)  # 0.2 s decimation grid for rover epochs and truth rows

IMU_RAW_HEADER = ("GPS TOW (s)", "GPS Week", "Acceleration X (m/s^2)", "Acceleration Y (m/s^2)",
                  "Acceleration Z (m/s^2)", "Angular rate X (rad/s)", "Angular rate Y (rad/s)",
                  "Angular rate Z (rad/s)", "Wheel velocity (m/s)")
# Byte-exact header of the PPC imu.csv (note the double space before "Ang Rate Y/Z").
PPC_IMU_HEADER = ("GPS TOW (s), GPS Week, Acc X (m/s^2), Acc Y (m/s^2), Acc Z (m/s^2), "
                  "Ang Rate X (deg/s),  Ang Rate Y (deg/s),  Ang Rate Z (deg/s)\n")
# The first 14 UrbanNav reference columns are the PPC reference columns, in PPC order,
# except for the velocity names. Further columns (acceleration, angular rate) are kept.
PPC_REFERENCE_HEADER = (
    "GPS TOW (s)", "GPS Week", "Latitude (deg)", "Longitude (deg)", "Ellipsoid Height (m)",
    "ECEF X (m)", "ECEF Y (m)", "ECEF Z (m)", "Roll (deg)", "Pitch (deg)", "Heading (deg)",
    "East Velocity (m/s)", "North Velocity (m/s)", "Up Velocity (m/s)")
VELOCITY_RENAME = {"Velocity X (m/s)": "East Velocity (m/s)", "Velocity Y (m/s)": "North Velocity (m/s)",
                   "Velocity Z (m/s)": "Up Velocity (m/s)"}


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def pin(path):
    path = Path(path)
    return dict(name=path.name, bytes=path.stat().st_size, sha256=sha256_file(path))


def gps_tow(year, month, day, hour, minute, seconds_text):
    """Exact (week, time of week in seconds as a Fraction) of a calendar GPS time."""
    days = (datetime.date(year, month, day) - GPS_EPOCH).days
    week, weekday = divmod(days, 7)
    return week, weekday*86400 + hour*3600 + minute*60 + Fraction(seconds_text)


def is_multiple(value, step):
    return (value/step).denominator == 1


def micros(text):
    """Exact microsecond count of a decimal-seconds string; sub-microsecond digits are an error."""
    value = Fraction(text)*1_000_000
    if value.denominator != 1:
        raise ValueError(f"time {text!r} is not a whole number of microseconds")
    return int(value)


def format_grid_tow(us):
    return f"{us//1_000_000}.{us % 1_000_000//10_000:02d}"


# ---------------------------------------------------------------------------
# rover.obs


def split_rinex_header(stream):
    """Return (header lines as bytes, time-system tag of TIME OF FIRST OBS) and leave stream at the first record."""
    header, time_system = [], None
    for line in stream:
        header.append(line)
        label = line[60:].strip()
        if label == b"TIME OF FIRST OBS":
            time_system = line[48:51].decode("ascii", "replace").strip()
        if label == b"END OF HEADER":
            return header, time_system
    raise ValueError("RINEX header has no END OF HEADER")


def decimate_rinex_obs(src, dst, span=None):
    """Copy epochs whose GPS time of week is a multiple of 0.2 s and, if `span` is given, lies within
    the inclusive (first, last) TOW span of the converted truth; header and records byte-preserved."""
    stats = dict(epochs_in=0, epochs_out=0, epochs_dropped_outside_truth_span=0,
                 first_tow_out=None, last_tow_out=None, week=None)
    with Path(src).open("rb") as stream, Path(dst).open("wb") as out:
        header, time_system = split_rinex_header(stream)
        if time_system != "GPS":
            raise ValueError(f"expected GPS time system in TIME OF FIRST OBS, got {time_system!r}")
        out.writelines(header)
        keep, previous = False, None
        for line in stream:
            if line.startswith(b">"):
                fields = line[1:].split()
                year, month, day, hour, minute = (int(v) for v in fields[:5])
                week, tow = gps_tow(year, month, day, hour, minute, fields[5].decode("ascii"))
                if previous is not None and (week, tow) <= previous:
                    raise ValueError(f"non-monotonic observation epoch at week {week} tow {float(tow)}")
                previous = (week, tow)
                if stats["week"] is None:
                    stats["week"] = week
                elif stats["week"] != week:
                    raise ValueError("observation epochs cross a GPS week boundary")
                stats["epochs_in"] += 1
                keep = is_multiple(tow, EPOCH_STEP)
                if keep and span is not None and not span[0] <= tow <= span[1]:
                    keep = False
                    stats["epochs_dropped_outside_truth_span"] += 1
                if keep:
                    stats["epochs_out"] += 1
                    if stats["first_tow_out"] is None:
                        stats["first_tow_out"] = tow
                    stats["last_tow_out"] = tow
            elif previous is None:
                raise ValueError("observation data before the first epoch line")
            if keep:
                out.write(line)
    for key in ("first_tow_out", "last_tow_out"):
        stats[key] = None if stats[key] is None else float(stats[key])
    return stats


# ---------------------------------------------------------------------------
# imu.csv


def convert_imu(src, dst):
    """FRD -> FLU, rad/s -> deg/s, wheel dropped, linear resample onto the 50 Hz TOW grid."""
    stats = dict(raw_samples=0, grid_samples=0, grid_points_skipped_gap=0, grid_points_skipped_on_raw_sample=0,
                 pairs_over_gap=0, week=None, first_raw_tow=None, last_raw_tow=None,
                 first_grid_tow=None, last_grid_tow=None)
    to_deg = math.degrees(1.0)
    with Path(src).open("r", encoding="utf-8", newline="") as stream, \
            Path(dst).open("w", encoding="utf-8", newline="") as out:
        header = tuple(name.strip() for name in stream.readline().rstrip("\r\n").split(","))
        if header != IMU_RAW_HEADER:
            raise ValueError(f"unexpected raw IMU header: {header}")
        out.write(PPC_IMU_HEADER)
        previous = None  # (t_us, [ax, ay, az, gx, gy, gz]) in FLU, deg/s
        for number, line in enumerate(stream, start=2):
            if not line.strip():
                continue
            fields = [f.strip() for f in line.split(",")]
            if len(fields) != len(IMU_RAW_HEADER):
                raise ValueError(f"line {number}: expected {len(IMU_RAW_HEADER)} fields")
            t_us, week = micros(fields[0]), int(fields[1])
            if stats["week"] is None:
                stats["week"] = week
            elif week != stats["week"]:
                raise ValueError(f"line {number}: GPS week changes")
            ax, ay, az, gx, gy, gz = (float(v) for v in fields[2:8])
            # FRD -> FLU: x -> x, y -> -y, z -> -z for accelerometer and gyro; gyro rad/s -> deg/s.
            values = [ax, -ay, -az, gx*to_deg, -gy*to_deg, -gz*to_deg]
            stats["raw_samples"] += 1
            if previous is None:
                stats["first_raw_tow"] = t_us/1e6
            else:
                t0, v0 = previous
                if t_us <= t0:
                    raise ValueError(f"line {number}: IMU time is not strictly increasing")
                dt = t_us - t0
                first_k = t0//GRID_US + 1                  # first grid point strictly after t0
                last_k = t_us//GRID_US                     # last grid point at or before t_us
                if dt > MAX_PAIR_GAP_US:
                    stats["pairs_over_gap"] += 1
                    stats["grid_points_skipped_gap"] += max(0, last_k - first_k + 1)
                else:
                    for k in range(first_k, last_k + 1):
                        g = k*GRID_US
                        weight = (g - t0)/dt
                        row = [a + (b - a)*weight + 0.0 for a, b in zip(v0, values)]
                        out.write(f"{format_grid_tow(g)}, {week}, " + ", ".join(f"{v:11.8f}" for v in row) + "\n")
                        stats["grid_samples"] += 1
                        if stats["first_grid_tow"] is None:
                            stats["first_grid_tow"] = g/1e6
                        stats["last_grid_tow"] = g/1e6
            if previous is None and t_us % GRID_US == 0:
                stats["grid_points_skipped_on_raw_sample"] += 1  # first sample: no pair ends on it
            previous = (t_us, values)
            stats["last_raw_tow"] = t_us/1e6
    if stats["raw_samples"] < 2:
        raise ValueError("IMU file has fewer than two samples")
    first_us, last_us = round(stats["first_raw_tow"]*1e6), round(stats["last_raw_tow"]*1e6)
    stats["grid_points_in_raw_span"] = last_us//GRID_US - (-(-first_us//GRID_US)) + 1
    stats["grid_points_skipped"] = stats["grid_points_in_raw_span"] - stats["grid_samples"]
    if stats["grid_points_skipped"] != stats["grid_points_skipped_gap"] + stats["grid_points_skipped_on_raw_sample"]:
        raise ValueError("internal error: skipped grid point accounting does not close")
    return stats


# ---------------------------------------------------------------------------
# reference.csv


def convert_reference(src, dst):
    """PPC header names; rows at TOW multiples of 0.2 s, copied unchanged."""
    stats = dict(rows_in=0, rows_out=0, first_tow_out=None, last_tow_out=None, week=None)
    with Path(src).open("rb") as stream, Path(dst).open("wb") as out:
        raw = [name.strip() for name in stream.readline().decode("utf-8-sig").rstrip("\r\n").split(",")]
        renamed = [VELOCITY_RENAME.get(name, name) for name in raw]
        if tuple(renamed[:len(PPC_REFERENCE_HEADER)]) != PPC_REFERENCE_HEADER:
            raise ValueError(f"unexpected raw reference header: {raw}")
        out.write((",".join(renamed) + "\n").encode("utf-8"))
        previous = None
        for number, line in enumerate(stream, start=2):
            if not line.strip():
                continue
            fields = line.decode("utf-8").split(",")
            if len(fields) != len(raw):
                raise ValueError(f"line {number}: expected {len(raw)} fields")
            tow, week = Fraction(fields[0].strip()), int(fields[1])
            if previous is not None and tow <= previous:
                raise ValueError(f"line {number}: reference time is not strictly increasing")
            previous = tow
            if stats["week"] is None:
                stats["week"] = week
            elif week != stats["week"]:
                raise ValueError(f"line {number}: GPS week changes")
            stats["rows_in"] += 1
            if is_multiple(tow, EPOCH_STEP):
                out.write(line if line.endswith(b"\n") else line + b"\n")
                stats["rows_out"] += 1
                if stats["first_tow_out"] is None:
                    stats["first_tow_out"] = float(tow)
                stats["last_tow_out"] = float(tow)
    return stats


# ---------------------------------------------------------------------------


def reference_span(path):
    """Exact inclusive (first, last) TOW of a converted reference.csv."""
    first = last = None
    with Path(path).open("rb") as stream:
        stream.readline()
        for line in stream:
            if line.strip():
                last = Fraction(line.split(b",", 1)[0].decode().strip())
                if first is None:
                    first = last
    if first is None:
        raise ValueError("converted reference has no rows")
    return first, last


def copy_unchanged(src, dst):
    shutil.copyfile(src, dst)
    if sha256_file(src) != sha256_file(dst):
        raise ValueError(f"copy of {src} is not byte-identical")


def convert(raw_dir, run, rover, output_root):
    if run not in RUNS or rover not in ROVERS:
        raise ValueError(f"run must be one of {RUNS} and rover one of {ROVERS}")
    raw_dir, output_root = Path(raw_dir), Path(output_root)
    names = dict(rover=f"{run}_rover_{rover}.obs", base=f"{run}_base_trimble.obs", nav=f"{run}_base.nav",
                 imu=f"{run}_imu.csv", reference=f"{run}_reference.csv")
    raw = {key: raw_dir/name for key, name in names.items()}
    missing = [str(p) for p in raw.values() if not p.is_file()]
    if missing:
        raise ValueError(f"missing raw input(s): {missing}")
    target = output_root/"urbannav"/f"{run}_{rover}"
    manifest_path = target.with_name(target.name + ".manifest.json")
    if target.exists() or manifest_path.exists():
        raise ValueError(f"output already exists: {target}")
    target.mkdir(parents=True)
    stats = {}
    # Truth first: rover epochs are kept only inside the converted truth's TOW span.
    stats["reference.csv"] = convert_reference(raw["reference"], target/"reference.csv")
    stats["rover.obs"] = decimate_rinex_obs(raw["rover"], target/"rover.obs", reference_span(target/"reference.csv"))
    copy_unchanged(raw["base"], target/"base.obs")
    copy_unchanged(raw["nav"], target/"base.nav")
    stats["imu.csv"] = convert_imu(raw["imu"], target/"imu.csv")
    manifest = dict(
        schema="libgnsspp.urbannav_ppc_conversion.v1", contract="docs/online_pva_default_switch_holdout_v1.md",
        run=run, rover=rover, converter=pin(__file__),
        raw_inputs={name: pin(path) for name, path in zip(("rover.obs", "base.obs", "base.nav", "imu.csv", "reference.csv"),
                                                          (raw["rover"], raw["base"], raw["nav"], raw["imu"], raw["reference"]))},
        outputs={name: pin(target/name) for name in ("rover.obs", "base.obs", "base.nav", "imu.csv", "reference.csv")},
        stats=stats)
    manifest_path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--raw-dir", type=Path, required=True)
    parser.add_argument("--run", choices=RUNS, required=True)
    parser.add_argument("--rover", choices=ROVERS, default="trimble")
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        manifest = convert(args.raw_dir, args.run, args.rover, args.output_root)
    except (ValueError, OSError) as error:
        print(f"convert_urbannav_to_ppc_layout: {error}", file=sys.stderr)
        return 2
    print(json.dumps(dict(run=args.run, rover=args.rover, stats=manifest["stats"]), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
