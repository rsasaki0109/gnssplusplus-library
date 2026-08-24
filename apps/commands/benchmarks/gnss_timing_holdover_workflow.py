#!/usr/bin/env python3
"""Acquire BRUX timing data, emit SPP clock telemetry, and score holdover."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
from typing import Any

from support.gnss_runtime import application_root
from benchmarks import gnss_japan_static_survey as survey

ROOT = application_root(__file__)
DISPATCHER = ROOT / "apps/gnss.py"
DEFAULT_PROFILE = ROOT / "configs/benchmarks/timing_holdover_r9_brux.json"
SCHEMA_VERSION = "libgnsspp.timing_holdover.workflow.v1"
API_SOURCE = {
    "url": "https://network.igs.org/api/public/stations/BRUX00BEL",
    "bytes": 1037,
    "sha256": "d95f06f084671e6f544c5132cda990c557ff8efe37bfd3328a2842e8af9df6f7",
}
DATASETS = {
    "development": {
        "day": "001", "role": "development",
        "obs": {
            "url": "https://igs.bkg.bund.de/root_ftp/IGS/obs/2024/001/BRUX00BEL_R_20240010000_01D_30S_MO.crx.gz",
            "bytes": 5268926,
            "sha256": "e13ba3261e07726cd621a9bd46bdfc46b03b8d58f52cabd39de5308a8169ebdc",
        },
        "nav": survey.SOURCES["nav_gz"], "clk": survey.SOURCES["clk_gz"],
    },
    "holdout": {
        "day": "002", "role": "sealed_holdout",
        "obs": {
            "url": "https://igs.bkg.bund.de/root_ftp/IGS/obs/2024/002/BRUX00BEL_R_20240020000_01D_30S_MO.crx.gz",
            "bytes": 5392461,
            "sha256": "91b1b0b32efea3b6795285fee5539a7842dd449c9ecfee6c7a38a07db71fc77c",
        },
        "nav": survey.HOLDOUT_SOURCES["nav_gz"], "clk": survey.HOLDOUT_SOURCES["clk_gz"],
    },
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME", "gnss timing-holdover-workflow"))
    parser.add_argument("--phase", choices=("development", "holdout"), required=True)
    parser.add_argument("--mode", choices=("smoke", "full"), default="smoke")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cache-dir", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args(argv)
    if args.phase == "holdout" and args.mode != "full":
        parser.error("sealed holdout requires --mode full")
    return args


def command(*parts: str | Path) -> list[str]:
    return [sys.executable, str(DISPATCHER), *[str(value) for value in parts]]


def run_step(argv: list[str], log_path: Path) -> dict[str, Any]:
    with log_path.open("a", encoding="utf-8") as log:
        log.write("\n$ " + shlex.join(argv) + "\n"); log.flush()
        completed = subprocess.run(argv, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=False)
    return {"argv": argv, "exit_status": completed.returncode, "status": "passed" if completed.returncode == 0 else "failed"}


def acquire(args: argparse.Namespace) -> tuple[dict[str, Path], dict[str, Any]]:
    dataset = DATASETS[args.phase]; cache = args.cache_dir.resolve()
    specs = {"obs": dataset["obs"], "nav": dataset["nav"], "clk": dataset["clk"], "api": API_SOURCE}
    if not (shutil.which("CRX2RNX") or shutil.which("crx2rnx")):
        specs["rnxcmp_linux_x86_64"] = survey.SOURCES["rnxcmp_linux_x86_64"]
    downloads = {
        name: survey.fetch_source(cache, spec, offline=args.offline, force=False, timeout=120.0)
        for name, spec in specs.items()
    }
    day = dataset["day"]; root = cache / "materialized" / f"r9-brux-2024-{day}"
    paths = {
        "obs": root / f"BRUX00BEL_R_2024{day}0000_01D_30S_MO.rnx",
        "nav": root / f"BRDC00IGS_R_2024{day}0000_01D_MN.rnx",
        "clk": root / f"IGS0OPSFIN_2024{day}0000_01D_30S_CLK.CLK",
        "api": root / "BRUX00BEL_network_api.json",
        "crx2rnx": cache / "tools/RNXCMP_4.2.0/CRX2RNX",
    }
    converter = survey.resolve_crx2rnx(downloads, paths)
    survey.convert_crx(downloads["obs"], paths["obs"], converter)
    survey.materialize_gzip(downloads["nav"], paths["nav"])
    survey.materialize_gzip(downloads["clk"], paths["clk"])
    survey.atomic_write(paths["api"], downloads["api"].read_bytes())
    metadata = {
        "dataset_role": dataset["role"], "day_of_year": day,
        "sources": {name: {**specs[name], "cache": survey.record(downloads[name])} for name in specs},
        "materialized": {name: survey.record(path) for name, path in paths.items()},
        "station_contract": json.loads(paths["api"].read_text(encoding="utf-8")),
    }
    return paths, metadata


def run(args: argparse.Namespace) -> int:
    output = args.output_dir.resolve(); manifest_path = output / "workflow_manifest.json"
    if args.phase == "holdout" and manifest_path.exists():
        print(f"Refusing to rerun opened R9 holdout: {manifest_path}", file=sys.stderr); return 2
    output.mkdir(parents=True, exist_ok=True); log_path = output / "workflow.log"
    log_path.write_text("libgnss++ R9 timing/holdover workflow\n", encoding="utf-8")
    profile_path = args.profile.resolve(); profile = json.loads(profile_path.read_text(encoding="utf-8"))
    if args.phase == "holdout" and profile.get("release_state") != "sealed":
        print("R9 holdout profile is not sealed", file=sys.stderr); return 2
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION, "phase": args.phase, "mode": args.mode,
        "status": "running", "profile": {"path": str(profile_path), "sha256": survey.sha256_file(profile_path)},
        "steps": [],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    try:
        paths, acquisition = acquire(args); manifest["acquisition"] = acquisition
    except Exception as exc:
        manifest.update(status="failed", failure_reason=f"acquisition:{exc}")
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"); return 1
    spp_out = output / "spp.pos"; clock_csv = output / "receiver_clock.csv"
    spp_summary = output / "spp_summary.json"
    spp = command(
        "spp", "--obs", paths["obs"], "--nav", paths["nav"], "--out", spp_out,
        "--clock-csv", clock_csv, "--summary-json", spp_summary,
        *(["--max-epochs", "120"] if args.mode == "smoke" else []),
    )
    step = run_step(spp, log_path); manifest["steps"].append(step)
    if step["status"] != "passed":
        manifest.update(status="failed", failure_reason="spp")
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"); return 1
    signoff = command(
        "timing-holdover-signoff", "--role", "development" if args.phase == "development" else "sealed_holdout",
        "--clock-csv", clock_csv, "--reference-clk", paths["clk"], "--station", "BRUX",
        "--output-dir", output / "signoff", *(["--profile", profile_path] if args.phase == "holdout" else []),
    )
    step = run_step(signoff, log_path); manifest["steps"].append(step)
    manifest.update(status=step["status"], exit_status=step["exit_status"])
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"R9 timing/holdover workflow: {manifest['status']} (service: No-Go)")
    print(f"Manifest: {manifest_path}")
    return int(step["exit_status"])


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
