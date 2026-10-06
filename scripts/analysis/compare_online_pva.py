#!/usr/bin/env python3
"""Apply the frozen PVA candidate gate to six normal and twelve scenario runs."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"apps/commands/benchmarks"))
from gnss_pva_evaluate import pin, dump
from gnss_pva_metrics import stats, read_rows

METRICS = ("rtk_position_m", "fused_position_m", "rtk_velocity_mps", "fused_velocity_mps", "rotation_deg")
COVERAGE = ("rtk_available", "fused_available", "rtk_velocity_available", "fused_velocity_available", "attitude_available", "heading_available")


def load(directory):
    manifest = json.loads((directory/"manifest.json").read_text(encoding="utf-8"))
    report = json.loads((directory/"score.json").read_text(encoding="utf-8"))
    if manifest["state"] != "passed" or report["state"] != "passed" or manifest["replay"]["max_epochs"] != 0:
        raise ValueError("need completed full replays")
    for name in ("score.json", "errors.csv"):
        recorded = manifest["outputs"][name]
        if pin(directory/name)["sha256"] != recorded["sha256"]: raise ValueError(f"modified {name}")
    if pin(directory/"replay/pva.csv")["sha256"] != manifest["estimate"]["sha256"]:
        raise ValueError("modified native CSV")
    with (directory/"errors.csv").open(encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    return manifest, report, rows


def compare(control, candidate, label):
    am, a, ar = load(control)
    bm, b, br = load(candidate)
    if bm["replay"].get("candidate") != "vehicle_nhc_latched_v1": raise ValueError("unexpected candidate")
    for name in ("rover.obs", "base.obs", "base.nav", "imu.csv", "reference.csv"):
        if am["inputs"][name]["sha256"] != bm["inputs"][name]["sha256"]: raise ValueError("different raw inputs")
    for key in ("scenario", "scenario_start_s", "scenario_duration_s", "epochs", "start_week", "start_tow", "base_ecef", "lever_arm_flu_m", "navigation_policy"):
        if am["replay"][key] != bm["replay"][key]: raise ValueError(f"different replay contract {key}")
    if a["match_fraction"] != 1 or b["match_fraction"] != 1: raise ValueError("need complete exact truth matches")
    amap, bmap = {r["elapsed_s"]: r for r in ar}, {r["elapsed_s"]: r for r in br}
    if set(amap) != set(bmap): raise ValueError("different emitted timestamps")
    gates, paired, improvement = [], {}, False
    def gate(name, av, bv, allowed, passed):
        gates.append(dict(name=name, control=av, candidate=bv, allowed=allowed, passed=bool(passed)))
    for key in METRICS:
        common = [(float(amap[t][key]), float(bmap[t][key])) for t in amap if amap[t].get(key) and bmap[t].get(key)]
        if any(not math.isfinite(v) for pair in common for v in pair): raise ValueError("nonfinite common-cohort error")
        paired[key] = dict(control=stats([x for x, _ in common]), candidate=stats([y for _, y in common]))
        for cohort, av, bv in (("all", a["scenes"]["all"]["metrics"].get(key, {}), b["scenes"]["all"]["metrics"].get(key, {})),
                              ("common", paired[key]["control"], paired[key]["candidate"])):
            for statistic in ("rmse", "p95"):
                x, y = av.get(statistic), bv.get(statistic)
                passed = x is not None and y is not None and y <= x*1.01+1e-9
                gate(f"{cohort}.{key}.{statistic}", x, y, "candidate <= 1.01 * control", passed)
                if key == "rotation_deg" and cohort == "all" and x is not None and y is not None and y < x-1e-9:
                    improvement = True
    for key in COVERAGE:
        gate("coverage."+key, a["coverage"][key], b["coverage"][key], "loss <= 0.001", b["coverage"][key] >= a["coverage"][key]-.001-1e-12)
    for key in ("first_fresh_s", "first_heading_s"):
        x, y = a["generations"]["0"][key], b["generations"]["0"][key]
        gate("initial."+key, x, y, "no later; both censored allowed", (x is None and y is None) or (y is not None and (x is None or y <= x+1e-6)))
    gate("processing.p95_ms", a["processing_ms"]["p95"], b["processing_ms"]["p95"], "candidate <= 2 * control; host contention", b["processing_ms"]["p95"] <= 2*a["processing_ms"]["p95"])
    if am["replay"]["scenario"] in ("gnss_outage", "imu_gap"):
        for key in ("recovery_gnss_update_s", "recovery_fresh_attitude_s", "recovery_heading_s"):
            x, y = a["scenario"][key], b["scenario"][key]
            gate("scenario."+key, x, y, "no later; both censored allowed", (x is None and y is None) or (y is not None and (x is None or y <= x+1e-6)))
    first_latch = a["generations"]["0"]["first_heading_s"]
    # Direct raw snapshot parity before the candidate's causal gate can fire.
    raw_a = read_rows(control/"replay/pva.csv")
    raw_b = read_rows(candidate/"replay/pva.csv")
    keys = (set(raw_a[0]) & set(raw_b[0]))-{"processing_ms"}
    prefix = [i for i, row in enumerate(ar) if first_latch is None or float(row["elapsed_s"]) <= first_latch]
    parity = all(all(raw_a[i][k] == raw_b[i][k] for k in keys) for i in prefix)
    gate("before_latch.numeric_parity", len(prefix), parity, "all common CSV fields except processing_ms exactly equal", parity)
    return dict(name=label, gates=gates, all_output_control=a, all_output_candidate=b, common_valid=paired,
                control_manifest=pin(control/"manifest.json"), candidate_manifest=pin(candidate/"manifest.json")), improvement


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--baseline-dir", type=Path, required=True)
    p.add_argument("--candidate-dir", type=Path, required=True)
    p.add_argument("--baseline-scenario-dir", type=Path, required=True)
    p.add_argument("--candidate-scenario-dir", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    args = p.parse_args()
    if args.output_dir.exists(): p.error("output directory must be new")
    args.output_dir.mkdir(parents=True)
    report = dict(schema="libgnsspp.pva_candidate_decision.v1", state="running", adoption="No-Go", default_changed=False,
                  contract=pin(ROOT/"docs/online_pva_candidate_v1.md"), comparison_source=pin(__file__), runs=[])
    try:
        improved = False
        for city in ("tokyo", "nagoya"):
            for run in (1, 2, 3):
                name = f"{city}{run}"
                result, improvement = compare(args.baseline_dir/name, args.candidate_dir/name, name)
                report["runs"].append(result)
                improved |= improvement
                for scenario in ("gnss_outage", "imu_gap"):
                    label = name+"-"+scenario
                    result, _ = compare(args.baseline_scenario_dir/label, args.candidate_scenario_dir/label, label)
                    report["runs"].append(result)
        failures = [dict(run=r["name"], gate=g) for r in report["runs"] for g in r["gates"] if not g["passed"]]
        if not improved: failures.append(dict(run="all_normal", gate=dict(name="targeted_rotation_improvement", passed=False)))
        report.update(state="passed", failures=failures, adoption="Go" if not failures else "No-Go", improved_rotation=improved)
        dump(args.output_dir/"decision.json", report)
        print(json.dumps(dict(state=report["state"], adoption=report["adoption"], failed_gates=len(failures), runs=len(report["runs"]))))
        return 0
    except (ValueError, OSError, KeyError) as error:
        report.update(state="failed", error=str(error))
        dump(args.output_dir/"decision.json", report)
        print(str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
