#!/usr/bin/env python3
"""Run a reproducible PPC moving-CLAS candidate against cached SSR inputs."""

from __future__ import annotations

from _paths import ROOT_DIR

import argparse
import hashlib
import json
import os
import subprocess
from pathlib import Path

import generate_ppc_clas_scorecard as scorecard


DEFAULT_RUNS = (
    "tokyo_run1",
    "tokyo_run2",
    "tokyo_run3",
    "nagoya_run1",
    "nagoya_run2",
    "nagoya_run3",
)


def required_file(path: Path, label: str) -> Path:
    if not path.is_file():
        raise SystemExit(f"Missing {label}: {path}")
    return path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def git_source_state(path: Path) -> dict[str, object]:
    revision = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        check=False,
        capture_output=True,
        text=True,
    )
    status = subprocess.run(
        ["git", "-C", str(path), "status", "--porcelain", "--untracked-files=all"],
        check=False,
        capture_output=True,
        text=True,
    )
    return {
        "revision": revision.stdout.strip() if revision.returncode == 0 else None,
        "worktree_dirty": status.returncode != 0 or bool(status.stdout.strip()),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=ROOT_DIR / "data" / "PPC-Dataset",
    )
    parser.add_argument("--ssr-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--gnss-ppp", type=Path, required=True)
    parser.add_argument("--runs", nargs="+", default=list(DEFAULT_RUNS))
    parser.add_argument("--held-min-dd-rows", type=int, choices=range(2, 7), default=4)
    parser.add_argument(
        "--held-max-publication-streak", type=int, choices=range(1, 6), default=5
    )
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    gnss_ppp = required_file(args.gnss_ppp.resolve(), "gnss_ppp binary")
    dataset_root = args.dataset_root.resolve()
    ssr_root = args.ssr_root.resolve()
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.update(scorecard.PARITY_ENV)
    environment["GNSS_PPP_CLAS_AR_HELD_MIN_DD_ROWS"] = str(
        args.held_min_dd_rows
    )
    environment["GNSS_PPP_CLAS_AR_HELD_MAX_PUBLICATION_STREAK"] = str(
        args.held_max_publication_streak
    )

    manifest: dict[str, object] = {
        "gnss_ppp": str(gnss_ppp),
        "gnss_ppp_sha256": sha256_file(gnss_ppp),
        "source": git_source_state(ROOT_DIR),
        "dataset_root": str(dataset_root),
        "ssr_root": str(ssr_root),
        "held_min_dd_rows": args.held_min_dd_rows,
        "held_max_publication_streak": args.held_max_publication_streak,
        "parity_environment": {
            **scorecard.PARITY_ENV,
            "GNSS_PPP_CLAS_AR_HELD_MIN_DD_ROWS": str(args.held_min_dd_rows),
            "GNSS_PPP_CLAS_AR_HELD_MAX_PUBLICATION_STREAK": str(
                args.held_max_publication_streak
            ),
        },
        "runs": {},
    }

    for run_key in args.runs:
        if run_key not in DEFAULT_RUNS:
            raise SystemExit(f"Unknown PPC run: {run_key}")
        city, run = run_key.split("_", 1)
        run_dir = dataset_root / city / run
        rover_obs = required_file(run_dir / "rover.obs", "rover observation")
        base_nav = required_file(run_dir / "base.nav", "base navigation")
        ssr_csv = required_file(
            ssr_root / run_key / "ssr" / f"{run_key}_expanded.csv",
            "expanded SSR CSV",
        )
        pos_path = output_root / f"{run_key}.pos"
        log_path = output_root / f"{run_key}.log"
        command = scorecard.build_gnss_ppp_command(
            gnss_ppp_bin=gnss_ppp,
            rover_obs=rover_obs,
            base_nav=base_nav,
            ssr_csv=ssr_csv,
            out_pos=pos_path,
            parity=True,
        )
        manifest["runs"][run_key] = {
            "pos": str(pos_path.resolve()),
            "log": str(log_path.resolve()),
            "command": command,
        }
        print(f"=== {run_key} ===", flush=True)
        print(subprocess.list2cmdline(command), flush=True)
        should_run = not args.dry_run and not (
            args.skip_existing and pos_path.is_file()
        )
        if should_run:
            completed = scorecard.run_logged(
                command,
                env=environment,
                log_path=log_path,
            )
            if completed.returncode != 0:
                raise SystemExit(
                    f"gnss_ppp failed for {run_key} with exit "
                    f"{completed.returncode}; see {log_path}"
                )
        if pos_path.is_file():
            manifest["runs"][run_key].update({
                "pos_size_bytes": pos_path.stat().st_size,
                "pos_sha256": sha256_file(pos_path),
            })

    manifest_path = output_root / "candidate_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
