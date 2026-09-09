"""Aggregate raw GPS L5 metadata only; no positioning or truth access."""
import csv
import hashlib
import json
import math
import sys
from collections import Counter
from pathlib import Path


def audit(path):
    fields = ("CodeType", "SignalType", "FullInterSignalBiasNanos",
              "FullInterSignalBiasUncertaintyNanos", "SatelliteInterSignalBiasNanos",
              "SatelliteInterSignalBiasUncertaintyNanos")
    counts = {name: Counter() for name in fields[:2]}
    finite = Counter()
    rows = 0
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not {"ConstellationType", "CarrierFrequencyHz"}.issubset(reader.fieldnames or []):
            raise ValueError("missing raw signal identity fields")
        available = {name: name in reader.fieldnames for name in fields}
        for row in reader:
            try:
                gps = float(row["ConstellationType"]) == 1
                frequency = float(row["CarrierFrequencyHz"])
            except (ValueError, TypeError):
                continue
            if not gps or not math.isfinite(frequency) or abs(frequency - 1176450000) > 1000:
                continue
            rows += 1
            for name in fields[:2]:
                counts[name][row.get(name, "<absent>") or "<empty>"] += 1
            for name in fields[2:]:
                try:
                    finite[name] += math.isfinite(float(row.get(name, "")))
                except (ValueError, TypeError):
                    pass
    with path.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    return dict(raw_sha256=digest, gps_l5_raw_rows=rows, columns_present=available,
                signal_counts=counts, finite_bias_counts={k: finite[k] for k in fields[2:]},
                truth_reads=0, retained_factor_claim=False)


if __name__ == "__main__":
    for argument in sys.argv[1:]:
        print(json.dumps(audit(Path(argument)), sort_keys=True))
