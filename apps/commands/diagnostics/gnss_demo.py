#!/usr/bin/env python3
"""Run the tracked, offline libgnss++ first-run positioning demo."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

from support.gnss_runtime import application_root


ROOT_DIR = application_root(__file__)
FIXTURE_NAMES = {
    "obs": "synthetic_ppp.obs",
    "sp3": "synthetic_ppp.sp3",
    "clk": "synthetic_ppp.clk",
}
EXPECTED_EPOCHS = 8
BUILD_CONFIGS = ("Release", "RelWithDebInfo", "Debug", "MinSizeRel")


def fixture_root() -> Path:
    """Resolve source-tree and installed-prefix fixture layouts."""
    candidates = (
        ROOT_DIR / "demo" / "fixtures",
        ROOT_DIR / "share" / "libgnsspp" / "demo",
    )
    for candidate in candidates:
        if all((candidate / name).is_file() for name in FIXTURE_NAMES.values()):
            return candidate
    searched = ", ".join(str(candidate) for candidate in candidates)
    raise SystemExit(
        "Self-contained demo fixture is missing; expected synthetic_ppp.obs, "
        f"synthetic_ppp.sp3, and synthetic_ppp.clk under: {searched}"
    )


def find_ppp_binary() -> Path:
    """Resolve the matching source-tree, installed, or PATH PPP executable."""
    suffix = ".exe" if os.name == "nt" else ""
    filename = f"gnss_ppp{suffix}"
    build_root = ROOT_DIR / "build"
    candidates = [build_root / "apps" / filename]
    for config in BUILD_CONFIGS:
        candidates.extend(
            (
                build_root / "apps" / config / filename,
                build_root / config / "apps" / filename,
                build_root / config / filename,
            )
        )
    candidates.extend(
        (
            ROOT_DIR / "apps" / filename,
            Path(__file__).resolve().parent / filename,
        )
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    on_path = shutil.which(f"gnss_ppp{suffix}")
    if on_path:
        return Path(on_path).resolve()
    raise SystemExit(
        "Built gnss_ppp was not found. From a source checkout, run "
        "`cmake -S . -B build && cmake --build build --target gnss_ppp`, "
        "then retry `gnss demo`."
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description=(
            "Run the tracked eight-epoch synthetic PPP fixture entirely offline "
            "and emit a .pos file, KML track, and JSON summary."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.cwd() / "output" / "self-contained-demo",
        help="Directory for generated artifacts (default: ./output/self-contained-demo).",
    )
    return parser.parse_args()


def count_position_epochs(path: Path) -> int:
    return sum(
        1
        for line in path.read_text(encoding="ascii").splitlines()
        if line.strip() and not line.lstrip().startswith("%")
    )


def validate_outputs(
    output_dir: Path,
    summary_path: Path,
    pos_path: Path,
    kml_path: Path,
) -> dict[str, object]:
    for path in (summary_path, pos_path, kml_path):
        if not path.is_file() or path.stat().st_size == 0:
            raise SystemExit(f"PPP demo did not produce a non-empty artifact: {path}")

    try:
        payload = json.loads(summary_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SystemExit(f"PPP demo summary is not valid JSON: {summary_path}") from exc
    if not isinstance(payload, dict):
        raise SystemExit(f"PPP demo summary must be a JSON object: {summary_path}")

    processed_epochs = payload.get("processed_epochs")
    valid_solutions = payload.get("valid_solutions")
    if processed_epochs != EXPECTED_EPOCHS or valid_solutions != EXPECTED_EPOCHS:
        raise SystemExit(
            "PPP demo regression: expected "
            f"{EXPECTED_EPOCHS} processed/valid epochs, got "
            f"{processed_epochs}/{valid_solutions}"
        )
    pos_epochs = count_position_epochs(pos_path)
    if pos_epochs != EXPECTED_EPOCHS:
        raise SystemExit(
            f"PPP demo regression: expected {EXPECTED_EPOCHS} .pos epochs, got {pos_epochs}"
        )
    kml_text = kml_path.read_text(encoding="utf-8")
    if "<coordinates>" not in kml_text or "</coordinates>" not in kml_text:
        raise SystemExit(f"PPP demo KML has no coordinate track: {kml_path}")

    payload["demo"] = {
        "schema_version": "self-contained-demo.v1",
        "offline": True,
        "synthetic_fixture": True,
        "fixture_provenance": (
            "Project-authored deterministic PPP observations and precise products; "
            "distributed under the repository MIT license."
        ),
        "fixture_dir": str(fixture_root()),
        "artifacts": {
            "position": str(pos_path),
            "kml": str(kml_path),
            "summary": str(summary_path),
        },
        "output_dir": str(output_dir),
    }
    summary_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return payload


def main() -> int:
    args = parse_args()
    fixture_dir = fixture_root()
    ppp_binary = find_ppp_binary()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        "obs": fixture_dir / FIXTURE_NAMES["obs"],
        "sp3": fixture_dir / FIXTURE_NAMES["sp3"],
        "clk": fixture_dir / FIXTURE_NAMES["clk"],
        "pos": output_dir / "demo_solution.pos",
        "kml": output_dir / "demo_solution.kml",
        "summary": output_dir / "demo_summary.json",
    }
    command = [
        str(ppp_binary),
        "--static",
        "--obs",
        str(paths["obs"]),
        "--sp3",
        str(paths["sp3"]),
        "--clk",
        str(paths["clk"]),
        "--no-estimate-troposphere",
        "--out",
        str(paths["pos"]),
        "--kml",
        str(paths["kml"]),
        "--summary-json",
        str(paths["summary"]),
        "--max-epochs",
        str(EXPECTED_EPOCHS),
    ]

    print("Running self-contained offline PPP demo...", flush=True)
    completed = subprocess.run(command, cwd=ROOT_DIR, check=False)
    if completed.returncode != 0:
        return completed.returncode
    validate_outputs(
        output_dir,
        paths["summary"],
        paths["pos"],
        paths["kml"],
    )
    print("Self-contained offline demo complete:")
    print(f"  position: {paths['pos']}")
    print(f"  kml:      {paths['kml']}")
    print(f"  summary:  {paths['summary']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
