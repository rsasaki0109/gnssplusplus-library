#!/usr/bin/env python3
"""Apply the frozen PVA candidate gate to six normal and twelve scenario runs.

``--gate-set default`` (the default) is the per-run/scenario gate of the v1-v10 contracts.
``--gate-set holdout_v2`` is the pooled gate set of docs/online_pva_default_switch_holdout_v2.md.
``--gate-set holdout_v3`` is holdout_v2 plus the absolute attitude-integrity gate H8, for
velocity_consistency_v10 (docs/online_pva_default_switch_holdout_v3.md).
``--gate-set holdout_v4`` is holdout_v2 plus the relative attitude non-inferiority gate H8r, for
independent_doppler_v1 (docs/online_pva_default_switch_holdout_v4.md).
"""
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
PPC_RUNS = tuple(f"{city}{run}" for city in ("tokyo", "nagoya") for run in (1, 2, 3))
# Gate 8 (opt-in, --attitude-integrity): absolute, not relative to the control.
ATTITUDE_INTEGRITY_ROTATION_DEG = 90.0
ATTITUDE_INTEGRITY_MAX_FRACTION = 0.01
COVERAGE = ("rtk_available", "fused_available", "rtk_velocity_available", "fused_velocity_available", "attitude_available", "heading_available")

# Fixed gate set of docs/online_pva_default_switch_holdout_v2.md. Do not tune.
GATE_SETS = ("default", "holdout_v2", "holdout_v3", "holdout_v4")
HOLDOUT_V2_CONTRACT = "docs/online_pva_default_switch_holdout_v2.md"
HOLDOUT_V2_CANDIDATE = "velocity_consistency_v9"
# holdout_v3 = holdout_v2 (H1-H7, same thresholds) + H8, for velocity_consistency_v10. Do not tune.
HOLDOUT_V3_CONTRACT = "docs/online_pva_default_switch_holdout_v3.md"
HOLDOUT_V3_CANDIDATE = "velocity_consistency_v10"
# holdout_v4 = holdout_v2 (H1-H7, same thresholds) + H8r, for independent_doppler_v1. Do not tune.
# H8r is relative: per candidate scenario replay, the fraction of scored epochs with rotation_deg > 90
# must be <= the control's fraction of the same replay + H8R_MARGIN. This candidate does not claim to fix attitude.
HOLDOUT_V4_CONTRACT = "docs/online_pva_default_switch_holdout_v4.md"
HOLDOUT_V4_CANDIDATE = "independent_doppler_v1"
H8R_MARGIN = 0.01
HOLDOUT_CONTRACTS = {"holdout_v2": HOLDOUT_V2_CONTRACT, "holdout_v3": HOLDOUT_V3_CONTRACT, "holdout_v4": HOLDOUT_V4_CONTRACT}
HOLDOUT_CANDIDATES = {"holdout_v2": HOLDOUT_V2_CANDIDATE, "holdout_v3": HOLDOUT_V3_CANDIDATE, "holdout_v4": HOLDOUT_V4_CANDIDATE}
# Gate sets that refuse any candidate name other than their own.
CANDIDATE_LOCKED_GATE_SETS = ("holdout_v3", "holdout_v4")
HOLDOUT_V2_SCENARIOS = (("normal", None, None), ("gnss_outage", 60, 10), ("imu_gap", 60, 4))  # name, start_s, duration_s
H1_METRICS = ("fused_position_m", "rotation_deg")                         # primary: candidate <= 1.00 x control
H2_METRICS = ("rtk_position_m", "rtk_velocity_mps", "fused_velocity_mps")  # secondary: candidate <= 1.10 x control
H3_METRICS = ("fused_position_m", "rotation_deg")                         # tail: P99 candidate <= 1.25 x control
H1_RATIO, H2_RATIO, H3_RATIO = 1.00, 1.10, 1.25
H4_COVERAGE_LOSS = 0.005
H5_TIMING_SLACK_S = 1.0
H6_PROCESSING_RATIO = 2.0
EPSILON = 1e-9


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


def rotation_flip_fraction(rows):
    """(count above 90 deg, scored epochs, fraction) over rows with a scored rotation_deg."""
    values = [float(r["rotation_deg"]) for r in rows if r.get("rotation_deg")]
    if any(not math.isfinite(v) for v in values): raise ValueError("nonfinite rotation_deg")
    above = sum(1 for v in values if v > ATTITUDE_INTEGRITY_ROTATION_DEG)
    return above, len(values), (above/len(values) if values else 0.0)


INPUT_NAMES = ("rover.obs", "base.obs", "base.nav", "imu.csv", "reference.csv")


def check_pair(am, a, ar, bm, b, br, candidate_name):
    """Integrity checks shared by every gate set; returns the elapsed_s-keyed rows of both arms."""
    if bm["replay"].get("candidate") != candidate_name: raise ValueError("unexpected candidate")
    for name in INPUT_NAMES:
        if am["inputs"][name]["sha256"] != bm["inputs"][name]["sha256"]: raise ValueError("different raw inputs")
    for key in ("scenario", "scenario_start_s", "scenario_duration_s", "epochs", "start_week", "start_tow", "base_ecef", "lever_arm_flu_m", "navigation_policy"):
        if am["replay"][key] != bm["replay"][key]: raise ValueError(f"different replay contract {key}")
    if a["match_fraction"] != 1 or b["match_fraction"] != 1: raise ValueError("need complete exact truth matches")
    amap, bmap = {r["elapsed_s"]: r for r in ar}, {r["elapsed_s"]: r for r in br}
    if set(amap) != set(bmap): raise ValueError("different emitted timestamps")
    return amap, bmap


def compare(control, candidate, label, candidate_name="vehicle_nhc_latched_v1", attitude_integrity=False):
    am, a, ar = load(control)
    bm, b, br = load(candidate)
    amap, bmap = check_pair(am, a, ar, bm, b, br, candidate_name)
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
    # Only the NHC candidate is causally inert before the latch. A candidate
    # that changes inference from the first epoch (velocity_consistency_v1)
    # is instead guarded by the separate candidate-none bit-identity check.
    if candidate_name == "vehicle_nhc_latched_v1":
        gate("before_latch.numeric_parity", len(prefix), parity, "all common CSV fields except processing_ms exactly equal", parity)
    if attitude_integrity:
        # Gate 8: absolute, applies to the candidate run alone (the control value is informational).
        _, _, control_fraction = rotation_flip_fraction(ar)
        _, _, fraction = rotation_flip_fraction(br)
        gate("attitude_integrity.rotation_gt_90deg_fraction", control_fraction, fraction,
             f"candidate fraction of scored epochs with rotation_deg > {ATTITUDE_INTEGRITY_ROTATION_DEG:g} <= {ATTITUDE_INTEGRITY_MAX_FRACTION:g} (absolute)",
             fraction <= ATTITUDE_INTEGRITY_MAX_FRACTION)
    return dict(name=label, gates=gates, all_output_control=a, all_output_candidate=b, common_valid=paired,
                control_manifest=pin(control/"manifest.json"), candidate_manifest=pin(candidate/"manifest.json")), improvement


def percentile(sorted_values, p):
    """Same linear interpolation as gnss_pva_metrics.stats, for P99."""
    k = (len(sorted_values)-1)*p
    i = int(k)
    return sorted_values[i] + (sorted_values[min(i+1, len(sorted_values)-1)]-sorted_values[i])*(k-i)


def pooled_stats(values):
    """gnss_pva_metrics.stats plus P99; an empty cohort has all statistics None (a missing metric)."""
    result = stats(values)
    result["p99"] = percentile(sorted(abs(v) for v in values), .99) if values else None
    return result


def timing_passed(x, y):
    """Null means never (censored), never zero. Candidate must not be later than control + 1 s."""
    if x is None: return True   # control never got there: a candidate that does (or does not) is no worse
    if y is None: return False
    return y <= x+H5_TIMING_SLACK_S+1e-6


def holdout_run(name, args, attitude_gate=False, relative_attitude_gate=False):
    """Pooled holdout gates for one run: its normal, gnss_outage and imu_gap replays, both arms.

    attitude_gate adds H8 (holdout_v3): per candidate scenario replay, the absolute fraction of scored
    epochs with rotation_deg > 90 must be <= 0.01. The control's fraction is information only.
    relative_attitude_gate adds H8r (holdout_v4): per scenario replay, the candidate fraction must be
    <= the control fraction of the same replay + 0.01."""
    arms = ("control", "candidate")
    binaries, replays, gates, first_inputs = set(), [], [], None
    keys = sorted(set(H1_METRICS+H2_METRICS+H3_METRICS))
    values = {(cohort, arm): {k: [] for k in keys} for cohort in ("all", "common") for arm in arms}
    epochs = {arm: 0 for arm in arms}
    covered = {arm: {k: 0 for k in COVERAGE} for arm in arms}
    def gate(hypothesis, gate_name, av, bv, allowed, passed):
        gates.append(dict(name=gate_name, hypothesis=hypothesis, control=av, candidate=bv, allowed=allowed, passed=bool(passed)))
    for scenario, start_s, duration_s in HOLDOUT_V2_SCENARIOS:
        label = name if scenario == "normal" else f"{name}-{scenario}"
        base_dir, cand_dir = ((args.baseline_dir, args.candidate_dir) if scenario == "normal"
                              else (args.baseline_scenario_dir, args.candidate_scenario_dir))
        am, a, ar = load(base_dir/label)
        bm, b, br = load(cand_dir/label)
        amap, bmap = check_pair(am, a, ar, bm, b, br, args.candidate_name)
        if am["replay"].get("candidate") != "none": raise ValueError("unexpected control")
        if am["replay"]["scenario"] != scenario: raise ValueError(f"{label}: expected the {scenario} scenario")
        if start_s is not None and (am["replay"]["scenario_start_s"], am["replay"]["scenario_duration_s"]) != (start_s, duration_s):
            raise ValueError(f"{label}: expected scenario window {start_s}+{duration_s} s")
        inputs = {n: am["inputs"][n]["sha256"] for n in INPUT_NAMES}
        if first_inputs is None: first_inputs = inputs
        elif inputs != first_inputs: raise ValueError(f"{label}: the scenarios of one run must share the raw inputs")
        binaries.update(manifest.get("binary", {}).get("sha256") for manifest in (am, bm))
        # Pooling: every scored epoch of the three replays; the paired cohort is keyed (scenario, elapsed_s).
        for key in keys:
            for t in amap:
                x, y = amap[t].get(key), bmap[t].get(key)
                if x: values[("all", "control")][key].append(float(x))
                if y: values[("all", "candidate")][key].append(float(y))
                if x and y:
                    values[("common", "control")][key].append(float(x))
                    values[("common", "candidate")][key].append(float(y))
        for arm, report, rows in (("control", a, ar), ("candidate", b, br)):
            epochs[arm] += report["epochs"]
            for key in COVERAGE:
                covered[arm][key] += round(report["coverage"][key]*report["epochs"])
            for key in keys:
                if report["scenes"]["all"]["metrics"].get(key, {}).get("count", 0) != sum(1 for r in rows if r.get(key)):
                    raise ValueError(f"{label}: errors.csv disagrees with score.json for {key}")
        if scenario == "normal":
            for key in ("first_fresh_s", "first_heading_s"):
                x, y = a["generations"]["0"][key], b["generations"]["0"][key]
                gate("H5", f"{scenario}.initial.{key}", x, y, "candidate <= control + 1.0 s; a null candidate fails if control is non-null", timing_passed(x, y))
        else:
            for key in ("recovery_gnss_update_s", "recovery_fresh_attitude_s", "recovery_heading_s"):
                x, y = a["scenario"][key], b["scenario"][key]
                gate("H5", f"{scenario}.scenario.{key}", x, y, "candidate <= control + 1.0 s; a null candidate fails if control is non-null", timing_passed(x, y))
        px, py = a["processing_ms"]["p95"], b["processing_ms"]["p95"]
        gate("H6", f"{scenario}.processing.p95_ms", px, py, "candidate <= 2 * control; host contention", py <= H6_PROCESSING_RATIO*px)
        if attitude_gate:
            _, _, control_fraction = rotation_flip_fraction(ar)
            _, _, fraction = rotation_flip_fraction(br)
            gate("H8", f"H8.{scenario}.attitude_integrity.rotation_gt_90deg_fraction", control_fraction, fraction,
                 f"candidate fraction of scored epochs with rotation_deg > {ATTITUDE_INTEGRITY_ROTATION_DEG:g} <= {ATTITUDE_INTEGRITY_MAX_FRACTION:g} (absolute; control is information only)",
                 fraction <= ATTITUDE_INTEGRITY_MAX_FRACTION)
        if relative_attitude_gate:
            _, _, control_fraction = rotation_flip_fraction(ar)
            _, _, fraction = rotation_flip_fraction(br)
            gate("H8r", f"H8r.{scenario}.attitude_non_inferiority.rotation_gt_90deg_fraction", control_fraction, fraction,
                 f"candidate fraction of scored epochs with rotation_deg > {ATTITUDE_INTEGRITY_ROTATION_DEG:g} <= control fraction + {H8R_MARGIN:g} (relative)",
                 fraction <= control_fraction+H8R_MARGIN+EPSILON)
        replays.append(dict(scenario=scenario, epochs=a["epochs"], control_manifest=pin(base_dir/label/"manifest.json"),
                            candidate_manifest=pin(cand_dir/label/"manifest.json")))
    cohorts = {cohort: {arm: {k: pooled_stats(v) for k, v in values[(cohort, arm)].items()} for arm in arms}
               for cohort in ("all", "common")}
    def ratio_gates(hypothesis, metrics, ratio, statistics, cohort_names):
        for cohort in cohort_names:
            for key in metrics:
                for statistic in statistics:
                    x, y = cohorts[cohort]["control"][key][statistic], cohorts[cohort]["candidate"][key][statistic]
                    gate(hypothesis, f"{hypothesis}.{cohort}.{key}.{statistic}", x, y, f"candidate <= {ratio:.2f} * control",
                         x is not None and y is not None and y <= x*ratio+EPSILON)
    ratio_gates("H1", H1_METRICS, H1_RATIO, ("rmse", "p95"), ("all", "common"))
    ratio_gates("H2", H2_METRICS, H2_RATIO, ("rmse", "p95"), ("all", "common"))
    ratio_gates("H3", H3_METRICS, H3_RATIO, ("p99",), ("all",))
    coverage = {arm: {k: covered[arm][k]/epochs[arm] for k in COVERAGE} for arm in arms}
    for key in COVERAGE:
        x, y = coverage["control"][key], coverage["candidate"][key]
        gate("H4", f"H4.coverage.{key}", x, y, f"candidate >= control - {H4_COVERAGE_LOSS}", y >= x-H4_COVERAGE_LOSS-1e-12)
    return dict(name=name, gates=gates, pooled=cohorts, pooled_coverage=coverage, pooled_epochs=epochs, replays=replays), binaries


def run_holdout(args, report):
    if args.gate_set in CANDIDATE_LOCKED_GATE_SETS and args.candidate_name != HOLDOUT_CANDIDATES[args.gate_set]:
        raise ValueError(f"{args.gate_set} requires the candidate {HOLDOUT_CANDIDATES[args.gate_set]}")
    binaries = set()
    for name in args.runs:
        result, used = holdout_run(name, args, attitude_gate=args.gate_set == "holdout_v3",
                                   relative_attitude_gate=args.gate_set == "holdout_v4")
        report["runs"].append(result)
        binaries |= used
    if len(binaries) != 1 or None in binaries: raise ValueError("all replays must record one and the same binary")
    failures = [dict(run=r["name"], gate=g) for r in report["runs"] for g in r["gates"] if not g["passed"]]
    report.update(state="passed", failures=failures, adoption="Go" if not failures else "No-Go", binary_sha256=next(iter(binaries)))
    return failures


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--baseline-dir", type=Path, required=True)
    p.add_argument("--candidate-dir", type=Path, required=True)
    p.add_argument("--baseline-scenario-dir", type=Path, required=True)
    p.add_argument("--candidate-scenario-dir", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--candidate-name", default=None, help="Default: vehicle_nhc_latched_v1 (default gate set), velocity_consistency_v9 (holdout_v2), velocity_consistency_v10 (holdout_v3) or independent_doppler_v1 (holdout_v4); holdout_v3 and holdout_v4 accept no other name")
    p.add_argument("--contract", type=Path, default=None, help="Default: docs/online_pva_candidate_v1.md (default gate set), the contract document of the holdout gate set otherwise")
    p.add_argument("--gate-set", choices=GATE_SETS, default="default",
                   help="default: per-run/scenario gates of the v1-v10 contracts; holdout_v2: pooled gates H1-H6 of the holdout v2 contract (H7 is the integrity failure of any check); holdout_v3: holdout_v2 plus H8 attitude integrity (holdout v3 contract); holdout_v4: holdout_v2 plus H8r relative attitude non-inferiority (holdout v4 contract)")
    p.add_argument("--runs", nargs="+", default=list(PPC_RUNS), metavar="RUN",
                   help="Run directory names under each input dir (default: the six PPC runs, tokyo1..nagoya3)")
    p.add_argument("--attitude-integrity", action="store_true",
                   help="Add gate 8 to every compared run: at most 1%% of the candidate's scored epochs may have rotation_deg > 90 (absolute)")
    args = p.parse_args()
    if len(set(args.runs)) != len(args.runs): p.error("--runs must not repeat a run name")
    holdout = args.gate_set in HOLDOUT_CONTRACTS
    if args.gate_set == "holdout_v3" and args.attitude_integrity: p.error("--attitude-integrity is part of holdout_v3 (H8); do not pass it")
    if args.gate_set == "holdout_v4" and args.attitude_integrity: p.error("--attitude-integrity (absolute) is not part of holdout_v4, which has the relative gate H8r; do not pass it")
    if args.candidate_name is None: args.candidate_name = HOLDOUT_CANDIDATES[args.gate_set] if holdout else "vehicle_nhc_latched_v1"
    if args.contract is None: args.contract = ROOT/(HOLDOUT_CONTRACTS[args.gate_set] if holdout else "docs/online_pva_candidate_v1.md")
    if args.output_dir.exists(): p.error("output directory must be new")
    args.output_dir.mkdir(parents=True)
    report = dict(schema="libgnsspp.pva_candidate_decision.v1", state="running", adoption="No-Go", default_changed=False,
                  contract=pin(args.contract), candidate=args.candidate_name, comparison_source=pin(__file__), runs=[])
    if args.attitude_integrity: report["attitude_integrity"] = True
    if holdout: report.update(schema="libgnsspp.pva_candidate_decision.v2", gate_set=args.gate_set)
    try:
        if holdout:
            failures = run_holdout(args, report)
            dump(args.output_dir/"decision.json", report)
            print(json.dumps(dict(state=report["state"], adoption=report["adoption"], failed_gates=len(failures),
                                  gates=sum(len(r["gates"]) for r in report["runs"]), runs=len(report["runs"]))))
            return 0
        improved = False
        for name in args.runs:
            result, improvement = compare(args.baseline_dir/name, args.candidate_dir/name, name, args.candidate_name, args.attitude_integrity)
            report["runs"].append(result)
            improved |= improvement
            for scenario in ("gnss_outage", "imu_gap"):
                label = name+"-"+scenario
                result, _ = compare(args.baseline_scenario_dir/label, args.candidate_scenario_dir/label, label, args.candidate_name, args.attitude_integrity)
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
