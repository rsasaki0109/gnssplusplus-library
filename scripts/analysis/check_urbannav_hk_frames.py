#!/usr/bin/env python3
"""Frame and time checks of raw UrbanNav Hong Kong files, truth against truth and IMU against truth.

Never runs an estimator and never compares an estimate to the truth. Pure standard library.
Documents the conventions fixed in docs/online_pva_default_switch_holdout_v2.md:

* truth body frame: which signed axis permutation of VelBdyX/Y/Z, rotated with roll/pitch/heading,
  reproduces the velocity obtained by differentiating the truth positions;
* IMU frame: which signed axis permutation of the Xsens accelerometer reproduces the specific force
  implied by the truth (forward and lateral acceleration, attitude);
* IMU time: gyro-z against the truth heading rate, as a lag scan (information only, never applied).
"""
import argparse
import bisect
import itertools
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import convert_urbannav_hk_to_ppc_layout as conv

GRAVITY = 9.79
MOVING_MPS = 3.0


def proper_signed_permutations():
    """The 24 rotation matrices with entries 0, +-1 as (rows, label); row i = output axis i picks input perm[i]."""
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            matrix = [[0]*3 for _ in range(3)]
            for i, (j, s) in enumerate(zip(perm, signs)):
                matrix[i][j] = s
            det = (matrix[0][0]*(matrix[1][1]*matrix[2][2]-matrix[1][2]*matrix[2][1])
                   - matrix[0][1]*(matrix[1][0]*matrix[2][2]-matrix[1][2]*matrix[2][0])
                   + matrix[0][2]*(matrix[1][0]*matrix[2][1]-matrix[1][1]*matrix[2][0]))
            if det == 1:
                yield matrix, f"perm={perm} signs={signs}"


def apply(matrix, vector):
    return [sum(matrix[i][j]*vector[j] for j in range(3)) for i in range(3)]


def correlation(x, y):
    n = len(x)
    mx, my = sum(x)/n, sum(y)/n
    sxy = sum((a-mx)*(b-my) for a, b in zip(x, y))
    sxx, syy = sum((a-mx)**2 for a in x), sum((b-my)**2 for b in y)
    return sxy/math.sqrt(sxx*syy) if sxx > 0 and syy > 0 else float("nan")


def enu_basis(lat, lon):
    lat, lon = math.radians(lat), math.radians(lon)
    return [[-math.sin(lon), math.cos(lon), 0.0],
            [-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)],
            [math.cos(lat)*math.cos(lon), math.cos(lat)*math.sin(lon), math.sin(lat)]]


def truth_arrays(rows):
    t = [float(r["tow"]) for r in rows]
    lat, lon = [float(r["lat"]) for r in rows], [float(r["lon"]) for r in rows]
    ecef = [conv.geodetic_to_ecef(a, b, float(r["height"])) for a, b, r in zip(lat, lon, rows)]
    rpy = [(float(r["roll"]), float(r["pitch"]), float(r["heading"])) for r in rows]
    return t, lat, lon, ecef, [r["velocity"] for r in rows], rpy


def velocity_convention(rows):
    """Residual RMS (m/s) of each body-axis convention against the differentiated truth position."""
    t, lat, lon, ecef, body, rpy = truth_arrays(rows)
    epochs = []
    for i in range(1, len(rows)-1):
        if t[i+1]-t[i] == 1.0 and t[i]-t[i-1] == 1.0:
            dp = [(ecef[i+1][k]-ecef[i-1][k])/2.0 for k in range(3)]
            enu = apply(enu_basis(lat[i], lon[i]), dp)
            if math.hypot(enu[0], enu[1]) > MOVING_MPS:
                epochs.append((i, enu))
    results = []
    for matrix, label in proper_signed_permutations():  # matrix maps the truth body axes to FRD
        total = 0.0
        for i, enu in epochs:
            frd = apply(matrix, body[i])
            n_, e_, d_ = apply(conv.euler_matrix(*rpy[i]), frd)
            total += (e_-enu[0])**2 + (n_-enu[1])**2 + (-d_-enu[2])**2
        results.append((math.sqrt(total/len(epochs)), label))
    results.sort()
    course = []
    for i, enu in epochs:
        course.append((rpy[i][2] - math.degrees(math.atan2(enu[0], enu[1])) + 180) % 360 - 180)
    course.sort()
    return dict(moving_epochs=len(epochs), ranked_rms_mps=[dict(rms=round(r, 3), convention=c) for r, c in results[:4]],
                worst_rms_mps=round(results[-1][0], 3), heading_minus_course_median_deg=round(course[len(course)//2], 2))


def read_imu(path):
    """(times in s of GPS time of week, gyro rows rad/s, accel rows m/s^2) in the raw sensor frame."""
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        header = stream.readline().rstrip("\r\n").split(",")
        i_stamp = header.index("field.header.stamp")
        i_gyro = [header.index(f"field.angular_velocity.{a}") for a in "xyz"]
        i_acc = [header.index(f"field.linear_acceleration.{a}") for a in "xyz"]
        t, gyro, acc = [], [], []
        for line in stream:
            f = line.rstrip("\r\n").split(",")
            if len(f) != len(header):
                continue
            t.append((int(f[i_stamp]) - conv.UNIX_TO_GPS_NS) % conv.WEEK_NS/1e9)
            gyro.append([float(f[i]) for i in i_gyro])
            acc.append([float(f[i]) for i in i_acc])
    return t, gyro, acc


def window_means(times, values, centers, half=0.5):
    """Mean of `values` over [c-half, c+half) for each center, via prefix sums; None where fewer than 20 samples."""
    prefix = [[0.0]*3]
    for v in values:
        prefix.append([a+b for a, b in zip(prefix[-1], v)])
    out = []
    for c in centers:
        i0, i1 = bisect.bisect_left(times, c-half), bisect.bisect_left(times, c+half)
        out.append(None if i1-i0 < 20 else [(prefix[i1][k]-prefix[i0][k])/(i1-i0) for k in range(3)])
    return out


def imu_convention(rows, imu_times, gyro, acc, lag_span=0.5, lag_step=0.01):
    t, lat, lon, ecef, body, rpy = truth_arrays(rows)
    forward = [b[1] for b in body]  # the truth body frame is x right, y forward, z up (velocity_convention)
    heading = []
    for h in (r[2] for r in rpy):   # unwrap
        heading.append(h if not heading else heading[-1] + (h - heading[-1] + 180) % 360 - 180)
    seconds = [i for i in range(1, len(rows)-1) if t[i+1]-t[i] == 1.0 and t[i]-t[i-1] == 1.0]
    means = dict(zip(seconds, window_means(imu_times, acc, [t[i] for i in seconds])))
    seconds = [i for i in seconds if means[i] is not None]
    model = []
    for i in seconds:
        a_fwd = (forward[i+1]-forward[i-1])/2.0
        yaw_ccw = -math.radians(heading[i+1]-heading[i-1])/2.0
        r, p = math.radians(rpy[i][0]), math.radians(rpy[i][1])
        model.append([a_fwd + GRAVITY*math.sin(p), forward[i]*yaw_ccw + GRAVITY*math.cos(p)*math.sin(r),
                      GRAVITY*math.cos(p)*math.cos(r)])
    results = []
    for matrix, label in proper_signed_permutations():  # matrix maps the raw sensor axes to FLU
        total = sum(sum((a-b)**2 for a, b in zip(apply(matrix, means[i]), m)) for i, m in zip(seconds, model))
        results.append((math.sqrt(total/(3*len(seconds))), label, matrix))
    results.sort(key=lambda r: r[0])
    identity = next(r for r in results if r[2] == [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    best = results[0][2]
    # Gyro z (FLU, counter-clockwise positive) against the truth heading (clockwise): 1 s integrals, lag scan.
    gz = [apply(best, g)[2] for g in gyro]
    cumulative = [0.0]
    for k in range(1, len(imu_times)):
        cumulative.append(cumulative[-1] + (gz[k]+gz[k-1])/2.0*(imu_times[k]-imu_times[k-1]))
    def at(x):
        k = bisect.bisect_left(imu_times, x)
        if k <= 0 or k >= len(imu_times): return None
        w = (x-imu_times[k-1])/(imu_times[k]-imu_times[k-1])
        return cumulative[k-1] + w*(cumulative[k]-cumulative[k-1])
    scan = []
    for step in range(-round(lag_span/lag_step), round(lag_span/lag_step)+1):
        lag = step*lag_step
        x, y = [], []
        for i in range(len(rows)-1):
            if t[i+1]-t[i] != 1.0: continue
            a, b = at(t[i]+lag), at(t[i+1]+lag)
            d = -math.radians(heading[i+1]-heading[i])
            if a is not None and b is not None and abs(d) > math.radians(1.0):
                x.append(b-a); y.append(d)
        if len(x) > 20:
            scan.append((correlation(x, y), lag))
    best_corr = max(scan)
    plateau = [lag for c, lag in scan if c > best_corr[0]-5e-4]
    return dict(seconds_compared=len(seconds),
                ranked_rms_mps2=[dict(rms=round(r, 3), convention=c, raw_to_flu=m) for r, c, m in results[:3]],
                identity_rms_mps2=round(identity[0], 3), worst_rms_mps2=round(results[-1][0], 3),
                gyro_z_vs_heading_rate=dict(best_corr=round(best_corr[0], 5), best_lag_s=round(best_corr[1], 3),
                                            plateau_lag_range_s=[round(min(plateau), 3), round(max(plateau), 3)],
                                            lag_convention="IMU stamp = truth time + lag; information only, never applied"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--truth", type=Path, required=True)
    parser.add_argument("--imu", type=Path, help="Xsens rosbag CSV; omit to check the truth only")
    args = parser.parse_args(argv)
    rows = conv.parse_truth(args.truth)
    report = dict(truth_rows=len(rows), leap_seconds_checked=conv.LEAP_SECONDS, velocity=velocity_convention(rows))
    if args.imu:
        times, gyro, acc = read_imu(args.imu)
        report["imu_samples"] = len(times)
        report["imu"] = imu_convention(rows, times, gyro, acc)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
