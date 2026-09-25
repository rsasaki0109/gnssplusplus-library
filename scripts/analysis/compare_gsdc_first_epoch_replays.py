"""Verify that first-epoch inclusion preserves frozen native output rows."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
from audit_gsdc_native_outputs import audit, digest, load_official_keys


def normalized(argv: list[str]) -> list[str]:
    result = list(argv)
    for flag in ("--out", "--summary-json"):
        result[result.index(flag) + 1] = "output"
    if "--android-include-first-native-epoch" in result:
        result.remove("--android-include-first-native-epoch")
    return result


def compare(folder: Path, official: dict[str, list[int]]) -> dict:
    run = json.loads((folder / "run.json").read_text())
    if run["state"] != "complete":
        return {"status": "pending", "recorded_pid": run.get("pid")}
    reference = Path(run["reference_run"])
    old = json.loads(reference.read_text())
    if ("--android-include-first-native-epoch" not in run["argv"] or
            "--android-include-first-native-epoch" in old["argv"] or
            normalized(run["argv"]) != normalized(old["argv"]) or
            run["binary_sha256"] != old["binary_sha256"]):
        raise ValueError(f"{folder}: replay changed more than output inclusion")
    case = run["argv"][run["argv"].index("--dataset-id") + 1]
    result = audit(case, {"folder": str(folder), "argv": run["argv"]}, official)
    if result.get("returncode") != 0:
        return {"status": "execution-failed", "audit": result}
    previous = reference.parent / "solution.csv"
    if digest(previous) != old["outputs"]["solution.csv"]:
        raise ValueError(f"{folder}: reference solution changed")
    with previous.open(newline="", encoding="utf-8-sig") as stream:
        before = list(csv.DictReader(stream))
    with (folder / "solution.csv").open(newline="", encoding="utf-8-sig") as stream:
        after = list(csv.DictReader(stream))
    unchanged = bool(after) and before == after[1:]
    return {"status": "verified" if unchanged and result.get("official_key_coverage", {}).get(
                "all_required_keys_from_native_states") else "comparison-failed",
            "dataset_id": case, "reference_run": str(reference),
            "same_binary_and_other_arguments": True,
            "previous_rows_identical": unchanged,
            "before_rows": len(before), "after_rows": len(after), "audit": result}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--official-sample", type=Path, required=True)
    parser.add_argument("--official-sample-sha256", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    args = parser.parse_args()
    official = load_official_keys(args.official_sample, args.official_sample_sha256)
    results = {str(p.parent): compare(p.parent, official)
               for p in sorted(args.root.rglob("run.json"))}
    report = {"schema": "gsdc-first-epoch-comparison.v1", "runs": results,
              "pending_records_do_not_prove_process_liveness": True,
              "official_sample_normalized_sha256": args.official_sample_sha256,
              "verified": sum(r["status"] == "verified" for r in results.values())}
    args.output_json.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"verified": report["verified"], "observed_runs": len(results)}))


if __name__ == "__main__":
    main()
