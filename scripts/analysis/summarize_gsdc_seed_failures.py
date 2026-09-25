"""Summarize full-epoch raw-P diagnostics without treating failed rows as seeds."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def summarize(folder: Path) -> dict:
    path = folder / "summary.json"
    raw = path.read_bytes()
    data = json.loads(raw)
    if not data.get("collect_all_epochs_for_diagnostics"):
        raise ValueError(f"{folder.name}: requires full-epoch diagnostic mode")
    rows = data["epochs"]
    if len(rows) != data["input_epoch_count"] or any(
        row["input_epoch_index"] != i for i, row in enumerate(rows)
    ):
        raise ValueError(f"{folder.name}: incomplete or reordered epochs")
    spans = []
    i = 0
    while i < len(rows):
        if rows[i]["status"] == "accepted":
            i += 1
            continue
        start = i
        while i < len(rows) and rows[i]["status"] != "accepted":
            i += 1
        end = i - 1
        bracketed = start > 0 and i < len(rows)
        spans.append({
            "start_epoch": start,
            "end_epoch": end,
            "failed_epochs": i - start,
            "reasons": dict(Counter(r["reason"] for r in rows[start:i])),
            "bracketed_by_accepted_epochs": bracketed,
            "accepted_bracket_utc_span_ms": (
                rows[i]["raw_utc_time_millis"] - rows[start - 1]["raw_utc_time_millis"]
                if bracketed else None
            ),
        })
    return {
        "dataset_id": data["dataset_id"],
        "summary_sha256": hashlib.sha256(raw).hexdigest(),
        "input_epochs": len(rows),
        "accepted_epochs": sum(r["status"] == "accepted" for r in rows),
        "failed_epochs": sum(r["status"] != "accepted" for r in rows),
        "failure_spans": spans,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("audit_root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    audit = json.loads((args.audit_root / "audit.json").read_text())
    if not audit["complete"]:
        raise ValueError("Audit is still running")
    runs = [summarize(p.parent) for p in sorted(args.audit_root.glob("*/summary.json"))]
    if len(runs) != len(audit["runs"]):
        raise ValueError("Missing diagnostic summaries")
    result = {
        "schema_version": "gsdc2023-seed-failure-spans.v1",
        "diagnostic_only": True,
        "initialization": "independent cold-start per epoch; differs from sequential native run",
        "interpolated_seeds_created": False,
        "runs": runs,
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
