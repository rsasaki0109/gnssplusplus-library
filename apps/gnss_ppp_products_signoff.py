#!/usr/bin/env python3
"""Run PPP sign-off with fetched real-product style inputs."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess

from gnss_runtime import resolve_gnss_command


ROOT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_PRESETS = ["igs-final", "ionex", "dcb"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--profile", choices=("static", "kinematic"), default="static")
    parser.add_argument("--obs", type=Path, default=None)
    parser.add_argument("--base", type=Path, default=None)
    parser.add_argument("--nav", type=Path, default=None)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--reference-pos", type=Path, default=None)
    parser.add_argument("--max-epochs", type=int, default=120)
    parser.add_argument("--preset", action="append", default=[])
    parser.add_argument("--product", action="append", default=[], metavar="KIND=SOURCE")
    parser.add_argument("--product-date", default=None)
    parser.add_argument("--product-cache-dir", type=Path, default=None)
    parser.add_argument("--enable-ar", action="store_true")
    parser.add_argument("--ar-ratio-threshold", type=float, default=3.0)
    parser.add_argument("--low-dynamics", dest="low_dynamics", action="store_true")
    parser.add_argument("--no-low-dynamics", dest="low_dynamics", action="store_false")
    parser.add_argument("--malib-bin", type=Path, default=None)
    parser.add_argument("--malib-config", type=Path, default=None)
    parser.add_argument("--malib-pos", type=Path, default=None)
    parser.add_argument("--use-existing-malib", action="store_true")
    parser.add_argument("--use-existing-reference", action="store_true")
    parser.add_argument("--require-converged", action="store_true")
    parser.add_argument("--require-convergence-time-max", type=float, default=None)
    parser.add_argument("--require-ionex-corrections-min", type=int, default=None)
    parser.add_argument("--require-dcb-corrections-min", type=int, default=None)
    parser.add_argument("--require-ppp-solution-rate-min", type=float, default=None)
    parser.add_argument("--require-valid-epochs-min", type=int, default=None)
    parser.add_argument("--require-common-epoch-pairs-min", type=int, default=None)
    parser.add_argument("--require-reference-fix-rate-min", type=float, default=None)
    parser.add_argument("--require-mean-error-max", type=float, default=None)
    parser.add_argument("--require-p95-error-max", type=float, default=None)
    parser.add_argument("--require-max-error-max", type=float, default=None)
    parser.add_argument("--require-mean-sats-min", type=float, default=None)
    parser.add_argument("--require-lib-mean-error-vs-malib-max-delta", type=float, default=None)
    parser.add_argument("--require-lib-p95-error-vs-malib-max-delta", type=float, default=None)
    parser.add_argument("--require-lib-max-error-vs-malib-max-delta", type=float, default=None)
    parser.set_defaults(low_dynamics=True)
    return parser.parse_args()


def default_paths(profile: str) -> tuple[Path, Path, Path | None]:
    if profile == "static":
        return (
            ROOT_DIR / "output/ppp_static_products_solution.pos",
            ROOT_DIR / "output/ppp_static_products_summary.json",
            None,
        )
    return (
        ROOT_DIR / "output/ppp_kinematic_products_solution.pos",
        ROOT_DIR / "output/ppp_kinematic_products_summary.json",
        ROOT_DIR / "output/ppp_kinematic_products_reference.pos",
    )


def run_checked(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, cwd=ROOT_DIR, check=True)


def classify_comparison_status(*deltas: object) -> str | None:
    numeric = [float(value) for value in deltas if isinstance(value, (int, float))]
    if not numeric:
        return None
    worst = max(numeric)
    if worst <= 0.0:
        return "better"
    if worst <= 0.25:
        return "close"
    return "worse"


def enforce_comparison_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []
    checks = [
        ("libgnss_minus_malib_mean_error_m", args.require_lib_mean_error_vs_malib_max_delta, "mean"),
        ("libgnss_minus_malib_p95_error_m", args.require_lib_p95_error_vs_malib_max_delta, "p95"),
        ("libgnss_minus_malib_max_error_m", args.require_lib_max_error_vs_malib_max_delta, "max"),
    ]
    for field, threshold, label in checks:
        if threshold is None:
            continue
        value = payload.get(field)
        if not isinstance(value, (int, float)):
            failures.append(f"missing MALIB comparison metric for {label} error")
            continue
        if float(value) > float(threshold):
            failures.append(
                f"libgnss++ {label} error delta {float(value):.6f} m > {float(threshold):.6f} m"
            )
    if failures:
        raise SystemExit("PPP products comparison checks failed:\n  - " + "\n  - ".join(failures))


def main() -> int:
    args = parse_args()
    out_path, summary_path, reference_path = default_paths(args.profile)
    out_path = args.out or out_path
    summary_path = args.summary_json or summary_path
    if args.reference_pos is not None:
        reference_path = args.reference_pos

    if args.preset:
        presets = list(args.preset)
    elif args.product:
        presets = []
    else:
        presets = list(DEFAULT_PRESETS)
    command = [*resolve_gnss_command(ROOT_DIR)]
    if args.profile == "static":
        command.append("ppp-static-signoff")
        if args.obs is not None:
            command.extend(["--obs", str(args.obs)])
        if args.nav is not None:
            command.extend(["--nav", str(args.nav)])
        command.extend(["--out", str(out_path), "--summary-json", str(summary_path), "--max-epochs", str(args.max_epochs)])
        if args.enable_ar:
            command.append("--enable-ar")
        if args.ar_ratio_threshold != 3.0:
            command.extend(["--ar-ratio-threshold", str(args.ar_ratio_threshold)])
        if args.require_valid_epochs_min is not None:
            command.extend(["--require-valid-epochs-min", str(args.require_valid_epochs_min)])
    else:
        command.append("ppp-kinematic-signoff")
        if args.obs is not None:
            command.extend(["--obs", str(args.obs)])
        if args.base is not None:
            command.extend(["--base", str(args.base)])
        if args.nav is not None:
            command.extend(["--nav", str(args.nav)])
        command.extend([
            "--out", str(out_path),
            "--reference-pos", str(reference_path),
            "--summary-json", str(summary_path),
            "--max-epochs", str(args.max_epochs),
        ])
        if args.use_existing_reference:
            command.append("--use-existing-reference")
        if args.low_dynamics:
            command.append("--low-dynamics")
        else:
            command.append("--no-low-dynamics")
        if args.require_common_epoch_pairs_min is not None:
            command.extend(["--require-common-epoch-pairs-min", str(args.require_common_epoch_pairs_min)])
        if args.require_reference_fix_rate_min is not None:
            command.extend(["--require-reference-fix-rate-min", str(args.require_reference_fix_rate_min)])

    command.append("--fetch-products")
    for preset in presets:
        command.extend(["--preset", preset])
    for product in args.product:
        command.extend(["--product", product])
    if args.product_date is not None:
        command.extend(["--product-date", args.product_date])
    if args.product_cache_dir is not None:
        command.extend(["--product-cache-dir", str(args.product_cache_dir)])
    if args.malib_bin is not None:
        command.extend(["--malib-bin", str(args.malib_bin)])
    if args.malib_config is not None:
        command.extend(["--malib-config", str(args.malib_config)])
    if args.malib_pos is not None:
        command.extend(["--malib-pos", str(args.malib_pos)])
    if args.use_existing_malib:
        command.append("--use-existing-malib")
    if args.require_converged:
        command.append("--require-converged")
    if args.require_convergence_time_max is not None:
        command.extend(["--require-convergence-time-max", str(args.require_convergence_time_max)])
    if args.require_ionex_corrections_min is not None:
        command.extend(["--require-ionex-corrections-min", str(args.require_ionex_corrections_min)])
    if args.require_dcb_corrections_min is not None:
        command.extend(["--require-dcb-corrections-min", str(args.require_dcb_corrections_min)])
    if args.require_ppp_solution_rate_min is not None:
        command.extend(["--require-ppp-solution-rate-min", str(args.require_ppp_solution_rate_min)])
    if args.require_mean_error_max is not None:
        command.extend(["--require-mean-error-max", str(args.require_mean_error_max)])
    if args.require_p95_error_max is not None:
        command.extend(["--require-p95-error-max", str(args.require_p95_error_max)])
    if args.require_max_error_max is not None:
        command.extend(["--require-max-error-max", str(args.require_max_error_max)])
    if args.require_mean_sats_min is not None:
        command.extend(["--require-mean-sats-min", str(args.require_mean_sats_min)])

    run_checked(command)

    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    payload["products_signoff_profile"] = args.profile
    payload["product_presets"] = presets
    payload["product_specs"] = list(args.product)
    payload["comparison_target"] = "MALIB" if payload.get("malib_solution_pos") else None
    payload["comparison_status"] = classify_comparison_status(
        payload.get("libgnss_minus_malib_mean_error_m"),
        payload.get("libgnss_minus_malib_p95_error_m"),
        payload.get("libgnss_minus_malib_max_error_m"),
    )
    enforce_comparison_requirements(payload, args)
    summary_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print("Finished PPP products sign-off.")
    print(f"  profile: {args.profile}")
    print(f"  solution: {out_path}")
    print(f"  summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
