#!/usr/bin/env python3
"""Offline P/V/A scoring. No truth-dependent alignment or interpolation."""
from __future__ import annotations

import csv
import math
from pathlib import Path

P = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]  # ENU -> NED
D = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]  # FLU -> FRD


def transpose(a):
    return list(map(list, zip(*a)))


def multiply(a, b):
    return [[sum(x*y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def rotate(a, v):
    return [sum(x*y for x, y in zip(row, v)) for row in a]


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def circular(a):
    return (a + 180) % 360 - 180


def enu(lat, lon):
    p, l = math.radians(lat), math.radians(lon)
    return [[-math.sin(l), math.cos(l), 0],
            [-math.sin(p)*math.cos(l), -math.sin(p)*math.sin(l), math.cos(p)],
            [math.cos(p)*math.cos(l), math.cos(p)*math.sin(l), math.sin(p)]]


def euler_matrix(roll, pitch, heading):
    r, p, h = map(math.radians, (roll, pitch, heading))
    cr, sr, cp, sp, ch, sh = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(h), math.sin(h)
    return [[ch*cp, ch*sp*sr-sh*cr, ch*sp*cr+sh*sr],
            [sh*cp, sh*sp*sr+ch*cr, sh*sp*cr-ch*sr], [-sp, cp*sr, cp*cr]]


def quaternion_matrix(q):
    if not all(math.isfinite(v) for v in q) or abs(norm(q)-1) >= 1e-6:
        raise ValueError("available attitude must have a finite unit quaternion")
    w, x, y, z = q
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def validate_rotation(a):
    if not all(math.isfinite(x) for row in a for x in row):
        raise ValueError("missing attitude frame rotation")
    product = multiply(a, transpose(a))
    det = (a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])
           -a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])
           +a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]))
    if abs(det-1) > 1e-6 or any(abs(product[i][j]-(i == j)) > 1e-6 for i in range(3) for j in range(3)):
        raise ValueError("attitude frame must be a proper orthonormal rotation")


def matrix_euler(a):
    return [math.degrees(math.atan2(a[2][1], a[2][2])),
            math.degrees(math.atan2(-a[2][0], math.hypot(a[2][1], a[2][2]))),
            math.degrees(math.atan2(a[1][0], a[0][0])) % 360]


def rotation_error(a, b):
    relative = multiply(transpose(b), a)
    return math.degrees(math.acos(max(-1, min(1, (sum(relative[i][i] for i in range(3))-1)/2))))


def stats(values):
    values = sorted(abs(v) for v in values)
    if not values:
        return dict(count=0, rmse=None, p50=None, p95=None, max=None)
    def percentile(p):
        k = (len(values)-1)*p
        i = int(k)
        return values[i] + (values[min(i+1, len(values)-1)]-values[i])*(k-i)
    return dict(count=len(values), rmse=norm(values)/math.sqrt(len(values)),
                p50=percentile(.5), p95=percentile(.95), max=values[-1])


def number(row, key):
    v = float(row[key])
    if not math.isfinite(v):
        raise ValueError(f"nonfinite required field {key}")
    return v


def flag(row, key):
    if row[key] not in ("0", "1"):
        raise ValueError(f"invalid boolean {key}")
    return row[key] == "1"


def timestamp(row, week, tow):
    w, t = number(row, week), number(row, tow)
    if w != int(w) or w < 0 or not 0 <= t < 604800:
        raise ValueError("invalid GPST week/tow")
    return int(w)*604800000000 + round(t*1e6)


def read_rows(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(line for line in stream if not line.startswith("%")))


def index_rows(rows, week, tow):
    result, previous = {}, None
    for row in rows:
        key = timestamp(row, week, tow)
        if previous is not None and key <= previous:
            raise ValueError("duplicate or backward timestamps")
        result[key], previous = row, key
    if not result:
        raise ValueError("empty input")
    return result


def score(estimate, reference):
    rows, truth_rows = read_rows(estimate), read_rows(reference)
    estimates = index_rows(rows, "rover_week", "rover_tow")
    truth = index_rows(truth_rows, "GPS Week", "GPS TOW (s)")
    # Reference-only scene labels. No truth reaches the estimator.
    labels, prev = {}, None
    for key, row in truth.items():
        velocity = [number(row, k) for k in ("East Velocity (m/s)", "North Velocity (m/s)", "Up Velocity (m/s)")]
        rpy = [number(row, k) for k in ("Roll (deg)", "Pitch (deg)", "Heading (deg)")]
        speed = math.hypot(*velocity[:2])
        yaw_rate = 0 if prev is None or key-prev[0] > 1000000 else circular(rpy[2]-prev[1])/((key-prev[0])/1e6)
        body_velocity = rotate(transpose(euler_matrix(*rpy)), [velocity[1], velocity[0], -velocity[2]])
        scene = ["all"]
        if speed < .2: scene.append("stop")
        if .2 <= speed < 2: scene.append("low_speed")
        if speed >= 1 and abs(yaw_rate) > 5: scene.append("turn")
        if body_velocity[0] < -.5: scene.append("reverse")
        labels[key] = scene
        prev = key, rpy[2]
    errors, unmatched, generations = [], 0, {}
    start = next(iter(estimates))
    for key, row in estimates.items():
        item = dict(elapsed_s=(key-start)/1e6, reset_generation=int(row["reset_generation"]), matched=key in truth)
        for name in ("fusion_initialized", "heading_aligned", "heading_converged", "attitude_available", "gnss_position_updated"):
            item[name] = flag(row, name)
        generation = generations.setdefault(item["reset_generation"], dict(start_s=item["elapsed_s"], first_fresh_s=None, first_heading_s=None))
        for field, valid in (("first_fresh_s", item["attitude_available"]), ("first_heading_s", item["attitude_available"] and item["heading_aligned"])):
            if valid and generation[field] is None:
                generation[field] = item["elapsed_s"]-generation["start_s"]
        for prefix in ("rtk", "fused"):
            status = int(row[prefix+"_status"])
            if not 0 <= status <= 7: raise ValueError("invalid solution status")
            item[prefix+"_available"] = status > 0
            item[prefix+"_velocity_available"] = flag(row, prefix+"_has_velocity") and item[prefix+"_available"]
            if item[prefix+"_available"]:
                age = (key-timestamp(row, prefix+"_week", prefix+"_tow"))/1e6
                if not 0 <= age <= .020001:
                    raise ValueError(f"stale or future {prefix} output marked available")
        if item["attitude_available"]:
            age = (key-timestamp(row, "attitude_week", "attitude_tow"))/1e6
            if not 0 <= age <= .020001: raise ValueError("stale or future attitude marked available")
            q = [number(row, k) for k in ("qw", "qx", "qy", "qz")]
            fixed = [[number(row, f"ecef_to_enu_{i}{j}") for j in range(3)] for i in range(3)]
            validate_rotation(fixed)
            body_to_fixed = quaternion_matrix(q)
        item["processing_ms"] = number(row, "processing_ms")
        if item["processing_ms"] < 0: raise ValueError("negative processing time")
        if key not in truth:
            unmatched += 1
            item["scenes"] = []
            errors.append(item)
            continue
        ref = truth[key]
        current = enu(number(ref, "Latitude (deg)"), number(ref, "Longitude (deg)"))
        position = [number(ref, k) for k in ("ECEF X (m)", "ECEF Y (m)", "ECEF Z (m)")]
        velocity = [number(ref, k) for k in ("East Velocity (m/s)", "North Velocity (m/s)", "Up Velocity (m/s)")]
        for prefix in ("rtk", "fused"):
            if item[prefix+"_available"]:
                delta = rotate(current, [number(row, prefix+"_"+axis+"_m")-position[i] for i, axis in enumerate("xyz")])
                item[prefix+"_position_m"] = norm(delta)
                item[prefix+"_horizontal_m"] = math.hypot(*delta[:2])
                item[prefix+"_vertical_m"] = abs(delta[2])
                for i, axis in enumerate("enu"): item[prefix+"_position_"+axis+"_m"] = delta[i]
            if item[prefix+"_velocity_available"]:
                v = rotate(current, [number(row, prefix+"_v"+axis+"_mps") for axis in "xyz"])
                dv = [a-b for a, b in zip(v, velocity)]
                item[prefix+"_velocity_mps"] = norm(dv)
                for i, axis in enumerate("enu"): item[prefix+"_velocity_"+axis+"_mps"] = dv[i]
        if item["attitude_available"]:
            estimate_ned = multiply(multiply(P, multiply(multiply(current, transpose(fixed)), body_to_fixed)), D)
            ref_rpy = [number(ref, k) for k in ("Roll (deg)", "Pitch (deg)", "Heading (deg)")]
            delta = [circular(a-b) for a, b in zip(matrix_euler(estimate_ned), ref_rpy)]
            item["roll_deg"], item["pitch_deg"] = delta[:2]
            if item["heading_aligned"]:
                item["heading_deg"] = delta[2]
                item["rotation_deg"] = rotation_error(estimate_ned, euler_matrix(*ref_rpy))
        item["scenes"] = labels[key]
        errors.append(item)
    metric_keys = sorted({key for row in errors for key in row if key.endswith(("_m", "_mps", "_deg"))})
    scenes = {}
    for scene in ("all", "stop", "low_speed", "turn", "reverse"):
        subset = [r for r in errors if scene in r["scenes"]]
        scenes[scene] = dict(epochs=len(subset),
            available_epochs={key: sum(r[key] for r in subset) for key in ("rtk_available", "fused_available", "attitude_available", "heading_aligned", "heading_converged")},
            metrics={key: stats([r[key] for r in subset if key in r]) for key in metric_keys})
    matched = len(rows)-unmatched
    duration_s = (next(reversed(estimates))-start)/1e6
    sample_intervals = [(b-a)/1e6 for a, b in zip(list(estimates), list(estimates)[1:])]
    period_s = sorted(sample_intervals)[len(sample_intervals)//2] if sample_intervals else None
    coverage = {name: sum(r[name] for r in errors)/len(rows) for name in
                ("rtk_available", "fused_available", "rtk_velocity_available", "fused_velocity_available", "attitude_available")}
    coverage.update(heading_available=sum(r["attitude_available"] and r["heading_aligned"] for r in errors)/len(rows),
                    heading_healthy=sum(r["attitude_available"] and r["heading_aligned"] and r["heading_converged"] for r in errors)/len(rows))
    report = dict(schema="libgnsspp.pva_score.v1", state="passed", epochs=len(rows), reference_epochs=len(truth),
        matched_epochs=matched, match_fraction=matched/len(rows), truth_coverage_fraction=matched/len(truth),
        missing_truth_epochs=unmatched, coverage=coverage, scenes=scenes, generations=generations,
        uninitialized_epochs=sum(not r["fusion_initialized"] for r in errors),
        missing_attitude_epochs=sum(not r["attitude_available"] for r in errors),
        unlatched_heading_epochs=sum(not r["heading_aligned"] for r in errors),
        duration_s=duration_s, median_input_period_s=period_s,
        output_rate_hz={key: value*len(rows)/(duration_s+(period_s or 0)) if duration_s+(period_s or 0) else None
                        for key, value in coverage.items()},
        processing_ms=stats([r["processing_ms"] for r in errors]),
        heading_error_over_90_epochs=sum(abs(r.get("heading_deg", 0)) > 90 for r in errors),
        heading_error_over_150_epochs=sum(abs(r.get("heading_deg", 0)) > 150 for r in errors),
        healthy_secondary={key: stats([r[key] for r in errors if r["heading_converged"] and key in r]) for key in metric_keys},
        conventions=dict(time="GPST; exact timestamp rounded to microsecond; no fitted shift",
            quaternion="wxyz; body FLU to fixed filter ENU; transported to reference local ENU",
            euler="FRD to NED, 3-2-1; heading clockwise from north; wrapped differences",
            position_velocity="antenna ECEF transported to reference ENU; published PPC lever, no fitted offset",
            primary="all fresh outputs; heading/rotation require first heading latch, never health filtering",
            scenes="truth-only labels: stop speed<0.2; low 0.2<=speed<2; turn speed>=1 and |yaw_rate|>5deg/s; reverse body-forward velocity<-0.5; overlapping"))
    if not matched:
        raise ValueError("no estimate timestamps match reference")
    return report, errors


def write_errors(path, rows):
    fields = sorted({k for row in rows for k in row})
    with Path(path).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(dict(row, scenes=";".join(row["scenes"])))


def plot_errors(path, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    x = [r["elapsed_s"] for r in rows]
    for ax, keys, unit in zip(axes, (("rtk_position_m", "fused_position_m"), ("rtk_velocity_mps", "fused_velocity_mps"),
                                   ("roll_deg", "pitch_deg", "heading_deg", "rotation_deg"),
                                   ("attitude_available", "heading_aligned", "heading_converged")), ("m", "m/s", "deg", "state")):
        for key in keys: ax.plot(x, [r.get(key, math.nan) for r in rows], label=key, linewidth=.8)
        ax.set_ylabel(unit)
        ax.grid(alpha=.25)
        ax.legend(loc="upper right", fontsize=8)
        for i, r in enumerate(rows):
            if i and r["reset_generation"] != rows[i-1]["reset_generation"]:
                ax.axvline(r["elapsed_s"], color="black", linestyle=":", alpha=.4)
    axes[-1].set_xlabel("Elapsed GPST seconds (gaps remain missing)")
    fig.savefig(path, dpi=130)
    plt.close(fig)
