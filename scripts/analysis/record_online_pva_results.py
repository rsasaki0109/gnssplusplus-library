#!/usr/bin/env python3
"""Preserve the frozen decision/provenance and independently derive bridge drift."""
import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"apps/commands/benchmarks"))
from gnss_pva_evaluate import dump, pin
from gnss_pva_metrics import score, scenario_summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--experiment-dir", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    args = p.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    decision = args.experiment_dir/"decision-v1/decision.json"
    shutil.copyfile(decision, args.output_dir/"online_pva_decision_v1.json")
    record = dict(schema="libgnsspp.pva_development_evidence.v1", state="running", decision=pin(decision),
        scoring_source=pin(ROOT/"apps/commands/benchmarks/gnss_pva_metrics.py"), manifests={}, scenario_drift={})
    for root_name in ("baseline", "candidate", "baseline-scenarios", "candidate-scenarios"):
        for directory in sorted((args.experiment_dir/root_name).iterdir()):
            manifest_path = directory/"manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            key = root_name+"/"+directory.name
            if manifest["state"] != "passed": raise ValueError("incomplete evaluation")
            record["manifests"][key] = dict(artifact=pin(manifest_path), content=manifest)
            if "scenarios" in root_name:
                reference = manifest["inputs"]["reference.csv"]["path"]
                estimate = directory/"replay/pva.csv"
                if pin(estimate)["sha256"] != manifest["estimate"]["sha256"]: raise ValueError("modified native CSV")
                if pin(reference)["sha256"] != manifest["inputs"]["reference.csv"]["sha256"]: raise ValueError("modified truth")
                recomputed, rows = score(estimate, reference)
                original = json.loads((directory/"score.json").read_text(encoding="utf-8"))
                # Adding the independent drift diagnostic must preserve the
                # frozen primary metrics exactly; never replace its decision.
                for scene in original["scenes"]:
                    if original["scenes"][scene]["metrics"] != recomputed["scenes"][scene]["metrics"]:
                        raise ValueError("primary score changed while deriving drift")
                replay = manifest["replay"]
                record["scenario_drift"][key] = scenario_summary(rows, replay["scenario"], replay["scenario_start_s"], replay["scenario_duration_s"])
    record["state"] = "passed"
    record["scope"] = "six existing PPC development/regression runs; fixed scenarios; no application holdout, sensor-axis fit or truth-assisted inference"
    dump(args.output_dir/"online_pva_provenance_v1.json", record)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = [f"{city}{run}" for city in ("tokyo", "nagoya") for run in (1,2,3)]
    reports = {kind: [json.loads((args.experiment_dir/kind/n/"score.json").read_text(encoding="utf-8")) for n in names] for kind in ("baseline", "candidate")}
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    for ax, metric, unit in zip(axes, ("fused_position_m", "rotation_deg"), ("Position norm RMSE (m, log scale)", "Full rotation RMSE (degrees)")):
        for kind, offset, color in (("baseline", -.18, "#5473a3"), ("candidate", .18, "#cf7c40")):
            values = [r["scenes"]["all"]["metrics"][metric]["rmse"] for r in reports[kind]]
            ax.bar([i+offset for i in range(6)], values, width=.35, color=color, label=kind)
            for i, value in enumerate(values): ax.annotate(f"{value:.1f}", (i+offset, value), xytext=(0,4), textcoords="offset points", ha="center", fontsize=8)
        ax.set_xticks(range(6), names)
        ax.set_ylabel(unit)
        ax.grid(axis="y", alpha=.2)
        ax.set_axisbelow(True)
        if metric == "fused_position_m": ax.set_yscale("log"); ax.set_ylim(20,10000)
        else: ax.set_ylim(0,190)
        ax.legend()
    fig.suptitle("PPC development data: latched vehicle NHC v1 is No-Go\nAll fresh outputs; unhealthy heading retained; no fitted alignment", fontsize=12)
    fig.savefig(args.output_dir/"online_pva_development_scorecard.png", dpi=140)
    plt.close(fig)
    print(json.dumps(dict(state="passed", manifests=len(record["manifests"]), drift_cases=len(record["scenario_drift"]))))


if __name__ == "__main__":
    main()
