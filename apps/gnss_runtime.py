#!/usr/bin/env python3
"""Runtime helpers shared by installed Python command wrappers."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import sys
from datetime import date


def resolve_gnss_command(root_dir: Path) -> list[str]:
    """Return a command prefix that launches the top-level gnss dispatcher."""
    source_dispatcher = root_dir / "apps" / "gnss.py"
    if source_dispatcher.exists():
        return [sys.executable, str(source_dispatcher)]

    sibling_dispatcher = Path(__file__).resolve().parent / "gnss"
    if sibling_dispatcher.exists():
        return [str(sibling_dispatcher)]

    installed_dispatcher = shutil.which("gnss")
    if installed_dispatcher is not None:
        return [installed_dispatcher]

    raise SystemExit(
        "Missing dispatcher: expected source-tree apps/gnss.py or installed gnss on PATH"
    )


def ensure_input_exists(path: Path, description: str, root_dir: Path) -> None:
    """Validate an input path and explain bundled-data behavior for installed prefixes."""
    if path.exists():
        return

    extra = ""
    bundled_roots = (
        root_dir / "data",
        root_dir / "scripts",
        root_dir / "configs",
    )
    if any(str(path).startswith(str(candidate)) for candidate in bundled_roots):
        extra = (
            " Installed prefixes do not ship the sample datasets; "
            "pass explicit --obs/--rover/--base/--nav paths from your own dataset or source tree."
        )

    raise SystemExit(f"Missing {description}: {path}.{extra}")


def parse_summary_metrics(line: str) -> dict[str, object]:
    """Parse a `summary:` line into a dict with int/float/string values."""
    metrics: dict[str, object] = {}
    if not line.startswith("summary:"):
        return metrics
    for token in line[len("summary:"):].strip().split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        try:
            if "." in value:
                metrics[key] = float(value)
            else:
                metrics[key] = int(value)
        except ValueError:
            metrics[key] = value
    return metrics


def _normalize_rinex_year(raw_year: int) -> int:
    if raw_year >= 100:
        return raw_year
    return 2000 + raw_year if raw_year < 80 else 1900 + raw_year


def infer_rinex_first_obs_date(path: Path) -> date:
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if "TIME OF FIRST OBS" in line:
                fields = line[:43].split()
                if len(fields) < 3:
                    break
                year = _normalize_rinex_year(int(float(fields[0])))
                month = int(float(fields[1]))
                day = int(float(fields[2]))
                return date(year, month, day)
            if "END OF HEADER" in line:
                break
    raise SystemExit(f"Failed to infer TIME OF FIRST OBS from {path}")


def run_fetch_products(
    root_dir: Path,
    obs_path: Path,
    product_specs: list[str],
    product_date_text: str | None = None,
    cache_dir: Path | None = None,
) -> dict[str, object]:
    if not product_specs:
        raise SystemExit("--fetch-products requires at least one --product KIND=SOURCE")

    effective_date = product_date_text or infer_rinex_first_obs_date(obs_path).isoformat()
    command = [*resolve_gnss_command(root_dir), "fetch-products", "--date", effective_date]
    for spec in product_specs:
        command.extend(["--product", spec])
    if cache_dir is not None:
        command.extend(["--cache-dir", str(cache_dir)])

    print("+", " ".join(command))
    completed = subprocess.run(
        command,
        cwd=root_dir,
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        raise SystemExit(
            f"gnss fetch-products failed with exit code {completed.returncode}\n"
            f"{completed.stdout}{completed.stderr}"
        )
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise SystemExit(
            "gnss fetch-products did not return valid JSON\n"
            f"{completed.stdout}{completed.stderr}"
        ) from exc
    assert isinstance(payload, dict)
    payload["effective_date"] = effective_date
    return payload
