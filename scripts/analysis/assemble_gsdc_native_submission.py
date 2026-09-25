"""Assemble a local all-native submission only from a complete frozen 40-drive plan."""
from __future__ import annotations

import argparse
import csv
import io
import json
import math
from pathlib import Path

from audit_gsdc_native_outputs import audit, digest, load_official_keys


def select_rows(rows_by_drive: dict, ordered_keys: list[tuple[str, int]]) -> list[dict]:
    """Select native coordinates by exact official keys, discarding only extra keys."""
    if not ordered_keys or len(set(ordered_keys)) != len(ordered_keys):
        raise ValueError("official keys must be nonempty and unique")
    if set(rows_by_drive) != {case for case, _ in ordered_keys}:
        raise ValueError("native drive set differs from official drive set")
    index = {}
    for case, rows in rows_by_drive.items():
        for row in rows:
            key = (case, int(row["UnixTimeMillis"]))
            lat, lon = float(row["LatitudeDegrees"]), float(row["LongitudeDegrees"])
            if (row["phone"] != case or key in index or not math.isfinite(lat) or
                    not math.isfinite(lon) or not -90 <= lat <= 90 or not -180 <= lon <= 180):
                raise ValueError(f"invalid native row or duplicate: {key}")
            index[key] = row
    missing = [key for key in ordered_keys if key not in index]
    if missing:
        raise ValueError(f"missing native official keys: {len(missing)}; first={missing[0]}")
    return [{"tripId": case, "UnixTimeMillis": stamp,
             "LatitudeDegrees": index[case, stamp]["LatitudeDegrees"],
             "LongitudeDegrees": index[case, stamp]["LongitudeDegrees"]}
            for case, stamp in ordered_keys]


def assemble(plan_path: Path, plan_sha256: str, sample: Path, sample_sha256: str,
             output: Path) -> dict:
    if output.exists():
        raise ValueError("output directory already exists")
    if digest(plan_path) != plan_sha256:
        raise ValueError("frozen plan hash mismatch")
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    official = load_official_keys(sample, sample_sha256)
    if len(official) != 40 or set(plan["runs"]) != set(official):
        raise ValueError("plan must cover exactly the 40 official drives")
    # Read sample keys only: sample coordinates never supply an output position.
    ordered = [(row["tripId"], int(row["UnixTimeMillis"])) for row in csv.DictReader(
        io.StringIO(sample.read_text(encoding="utf-8-sig")))]
    load_official_keys(sample, sample_sha256)  # Recheck the second key read.
    records = {}
    incomplete = []
    for case, entry in plan["runs"].items():
        record = Path(entry["folder"]) / "run.json"
        run = json.loads(record.read_text()) if record.exists() else {}
        if run.get("state") != "complete" or run.get("returncode") != 0:
            incomplete.append(case)
        records[case] = run
    if incomplete:
        raise ValueError(f"incomplete or failed native drives: {len(incomplete)}; no CSV written")
    hashes = {}

    def checked_hash(path: Path) -> str:
        path = path.resolve()
        if path not in hashes:
            hashes[path] = digest(path)
        return hashes[path]

    rows, proofs = {}, {}
    for case, entry in plan["runs"].items():
        folder = Path(entry["folder"])
        run = records[case]
        if (run["argv"] != entry["argv"] or
                run["binary_sha256"] != plan["binary_sha256"] or
                checked_hash(Path(entry["argv"][0])) != plan["binary_sha256"]):
            raise ValueError(f"{case}: fixed executable/arguments mismatch")
        input_paths = {Path(item["path"]).resolve() for item in entry["inputs"]}
        required = {Path(entry["argv"][entry["argv"].index(flag) + 1]).resolve()
                    for flag in ("--android-gnss", "--android-imu", "--nav", "--native-base-rinex")}
        if input_paths != required:
            raise ValueError(f"{case}: input hash inventory does not match arguments")
        for item in entry["inputs"]:
            if checked_hash(Path(item["path"])) != item["sha256"]:
                raise ValueError(f"{case}: frozen input hash mismatch")
        result = audit(case, entry, official)
        if not result.get("official_key_coverage", {}).get("all_required_keys_from_native_states"):
            raise ValueError(f"{case}: native official-key audit failed")
        with (folder / "solution.csv").open(newline="", encoding="utf-8-sig") as stream:
            rows[case] = list(csv.DictReader(stream))
        # Outputs must remain identical to those audited immediately above.
        for name in ("solution.csv", "summary.json"):
            if digest(folder / name) != run["outputs"][name]:
                raise ValueError(f"{case}: output changed during assembly")
        proofs[case] = {"audit": result, "run_record": str(folder / "run.json"),
                        "run_record_sha256": digest(folder / "run.json"),
                        "solution_sha256": run["outputs"]["solution.csv"],
                        "summary_sha256": run["outputs"]["summary.json"],
                        "inputs": entry["inputs"]}
    selected = select_rows(rows, ordered)
    output.mkdir(parents=True, exist_ok=False)
    csv_path = output / "submission.csv"
    with csv_path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(selected[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(selected)
    manifest = {"schema": "gsdc-native-local-submission.v1", "submitted": False,
                "plan_sha256": plan_sha256, "official_sample_normalized_sha256": sample_sha256,
                "binary_sha256": plan["binary_sha256"], "drives": 40,
                "official_key_rows": len(selected), "submission_sha256": digest(csv_path),
                "sample_coordinates_used": False, "all_output_rows_native": True,
                "position_accuracy_evaluated": False,
                "dropped_nonofficial_rows": sum(map(len, rows.values())) - len(selected),
                "runs": proofs}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--official-sample", required=True, type=Path)
    parser.add_argument("--official-sample-sha256", required=True)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    result = assemble(args.plan, args.plan_sha256, args.official_sample,
                      args.official_sample_sha256, args.output_dir)
    print(json.dumps({"rows": result["official_key_rows"], "submitted": False}))


if __name__ == "__main__":
    main()
