#!/usr/bin/env python3
"""Run PPP sign-off with fetched real-product style inputs."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

import gnss_ppc_demo as ppc_demo
from gnss_runtime import ensure_input_exists, resolve_gnss_command, run_fetch_products


ROOT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_PRESETS = ["igs-final", "ionex", "dcb"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--profile", choices=("static", "kinematic", "ppc"), default="static")
    parser.add_argument("--obs", type=Path, default=None)
    parser.add_argument("--base", type=Path, default=None)
    parser.add_argument("--nav", type=Path, default=None)
    parser.add_argument("--dataset-root", type=Path, default=None)
    parser.add_argument("--city", choices=("tokyo", "nagoya"), default=None)
    parser.add_argument("--run", default="run1")
    parser.add_argument("--run-dir", type=Path, default=None)
    parser.add_argument("--reference-csv", type=Path, default=None)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--reference-pos", type=Path, default=None)
    parser.add_argument("--max-epochs", type=int, default=120)
    parser.add_argument("--match-tolerance-s", type=float, default=0.25)
    parser.add_argument("--use-existing-solution", action="store_true")
    parser.add_argument("--solver-wall-time-s", type=float, default=None)
    parser.add_argument("--ppp-run-summary-json", type=Path, default=None)
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
    parser.add_argument("--require-matched-epochs-min", type=int, default=None)
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
    if profile == "ppc":
        return (
            ROOT_DIR / "output/ppp_ppc_products_solution.pos",
            ROOT_DIR / "output/ppp_ppc_products_summary.json",
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


def enforce_ppp_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []
    if args.require_converged and not bool(payload.get("ppp_converged", False)):
        failures.append("PPP run did not report convergence")
    if (
        args.require_convergence_time_max is not None
        and float(payload.get("ppp_convergence_time_s", 0.0)) > args.require_convergence_time_max
    ):
        failures.append(
            f"PPP convergence time {float(payload.get('ppp_convergence_time_s', 0.0)):.6f} s > "
            f"{args.require_convergence_time_max:.6f} s"
        )
    if (
        args.require_ionex_corrections_min is not None
        and int(payload.get("ionex_corrections", 0)) < args.require_ionex_corrections_min
    ):
        failures.append(
            f"IONEX corrections {int(payload.get('ionex_corrections', 0))} < {args.require_ionex_corrections_min}"
        )
    if (
        args.require_dcb_corrections_min is not None
        and int(payload.get("dcb_corrections", 0)) < args.require_dcb_corrections_min
    ):
        failures.append(
            f"DCB corrections {int(payload.get('dcb_corrections', 0))} < {args.require_dcb_corrections_min}"
        )
    if (
        args.require_ppp_solution_rate_min is not None
        and float(payload.get("ppp_solution_rate_pct", 0.0)) < args.require_ppp_solution_rate_min
    ):
        failures.append(
            f"PPP solution rate {float(payload.get('ppp_solution_rate_pct', 0.0)):.6f}% < "
            f"{args.require_ppp_solution_rate_min:.6f}%"
        )
    if failures:
        raise SystemExit("PPP products checks failed:\n  - " + "\n  - ".join(failures))


def enforce_ppc_metric_requirements(payload: dict[str, object], args: argparse.Namespace) -> None:
    failures: list[str] = []
    if args.require_valid_epochs_min is not None and int(payload.get("valid_epochs", 0)) < args.require_valid_epochs_min:
        failures.append(f"valid epochs {int(payload.get('valid_epochs', 0))} < {args.require_valid_epochs_min}")
    if args.require_matched_epochs_min is not None and int(payload.get("matched_epochs", 0)) < args.require_matched_epochs_min:
        failures.append(f"matched epochs {int(payload.get('matched_epochs', 0))} < {args.require_matched_epochs_min}")
    if (
        args.require_mean_error_max is not None
        and float(payload.get("mean_position_error_m", 0.0)) > args.require_mean_error_max
    ):
        failures.append(
            f"mean position error {float(payload.get('mean_position_error_m', 0.0)):.6f} m > "
            f"{args.require_mean_error_max:.6f} m"
        )
    if (
        args.require_p95_error_max is not None
        and float(payload.get("p95_position_error_m", 0.0)) > args.require_p95_error_max
    ):
        failures.append(
            f"p95 position error {float(payload.get('p95_position_error_m', 0.0)):.6f} m > "
            f"{args.require_p95_error_max:.6f} m"
        )
    if (
        args.require_max_error_max is not None
        and float(payload.get("max_position_error_m", 0.0)) > args.require_max_error_max
    ):
        failures.append(
            f"max position error {float(payload.get('max_position_error_m', 0.0)):.6f} m > "
            f"{args.require_max_error_max:.6f} m"
        )
    if (
        args.require_mean_sats_min is not None
        and float(payload.get("mean_satellites", 0.0)) < args.require_mean_sats_min
    ):
        failures.append(
            f"mean satellites {float(payload.get('mean_satellites', 0.0)):.6f} < {args.require_mean_sats_min:.6f}"
        )
    if failures:
        raise SystemExit("PPP products PPC checks failed:\n  - " + "\n  - ".join(failures))


def run_checked_json(command: list[str], summary_path: Path) -> dict[str, object]:
    run_checked(command)
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit("Expected JSON object summary payload")
    return payload


def resolve_ppc_run_dir(args: argparse.Namespace) -> Path:
    if args.run_dir is not None:
        return args.run_dir
    if args.dataset_root is None or args.city is None:
        raise SystemExit("--profile ppc requires --run-dir or both --dataset-root and --city")
    return args.dataset_root / args.city / args.run


def augment_ppc_malib_comparison(
    payload: dict[str, object],
    reference_csv: Path,
    malib_pos: Path,
    match_tolerance_s: float,
) -> None:
    reference = ppc_demo.read_flexible_reference_csv(reference_csv)
    comparison = ppc_demo.comparison
    malib_epochs = comparison.read_rtklib_pos(malib_pos)
    malib_metrics = ppc_demo.summarize_solution_epochs(
        reference,
        malib_epochs,
        ppc_demo.solver_fixed_status("ppp"),
        "MALIB",
        match_tolerance_s,
        solver_wall_time_s=None,
    )
    payload["malib_solution_pos"] = str(malib_pos)
    payload["comparison_target"] = "MALIB"
    payload["comparison_status"] = classify_comparison_status(
        float(payload.get("mean_position_error_m", 0.0)) - float(malib_metrics["mean_h_m"]),
        float(payload.get("p95_position_error_m", 0.0)) - float(malib_metrics["p95_h_m"]),
        float(payload.get("max_position_error_m", 0.0)) - float(malib_metrics["max_h_m"]),
    )
    payload["malib"] = malib_metrics
    payload["libgnss_minus_malib_mean_error_m"] = round(
        float(payload.get("mean_position_error_m", 0.0)) - float(malib_metrics["mean_h_m"]), 6
    )
    payload["libgnss_minus_malib_p95_error_m"] = round(
        float(payload.get("p95_position_error_m", 0.0)) - float(malib_metrics["p95_h_m"]), 6
    )
    payload["libgnss_minus_malib_max_error_m"] = round(
        float(payload.get("max_position_error_m", 0.0)) - float(malib_metrics["max_h_m"]), 6
    )


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

    if args.profile == "ppc":
        run_dir = resolve_ppc_run_dir(args)
        rover_obs = args.obs or (run_dir / "rover.obs")
        reference_csv = args.reference_csv or (run_dir / "reference.csv")
        ensure_input_exists(rover_obs, "PPC rover observation file", ROOT_DIR)
        ensure_input_exists(reference_csv, "PPC reference CSV", ROOT_DIR)
        if args.use_existing_solution:
            ensure_input_exists(out_path, "existing PPC PPP solution file", ROOT_DIR)
            if args.ppp_run_summary_json is None:
                raise SystemExit("--profile ppc with --use-existing-solution requires --ppp-run-summary-json")
            ensure_input_exists(args.ppp_run_summary_json, "existing PPP run summary JSON", ROOT_DIR)

        fetch_payload = run_fetch_products(
            ROOT_DIR,
            rover_obs,
            presets,
            list(args.product),
            product_date_text=args.product_date,
            cache_dir=args.product_cache_dir,
        )
        fetched_products = fetch_payload.get("products", {})
        if not isinstance(fetched_products, dict):
            raise SystemExit("gnss fetch-products did not return a product map")
        if "sp3" not in fetched_products or "clk" not in fetched_products:
            raise SystemExit("--profile ppc requires fetched products to include both sp3 and clk")

        temp_summary_dir: tempfile.TemporaryDirectory[str] | None = None
        if args.ppp_run_summary_json is not None:
            ppp_summary_path = args.ppp_run_summary_json
        else:
            temp_summary_dir = tempfile.TemporaryDirectory(prefix="gnss_ppp_products_ppc_")
            ppp_summary_path = Path(temp_summary_dir.name) / "ppp_summary.json"
        try:
            command = [*resolve_gnss_command(ROOT_DIR), "ppc-demo", "--solver", "ppp"]
            if args.run_dir is not None:
                command.extend(["--run-dir", str(args.run_dir)])
            else:
                command.extend(["--dataset-root", str(args.dataset_root), "--city", str(args.city), "--run", args.run])
            command.extend(
                [
                    "--out",
                    str(out_path),
                    "--summary-json",
                    str(summary_path),
                    "--reference-csv",
                    str(reference_csv),
                    "--match-tolerance-s",
                    str(args.match_tolerance_s),
                    "--ppp-summary-json",
                    str(ppp_summary_path),
                    "--sp3",
                    str(fetched_products["sp3"]),
                    "--clk",
                    str(fetched_products["clk"]),
                ]
            )
            if "ionex" in fetched_products:
                command.extend(["--ionex", str(fetched_products["ionex"])])
            if "dcb" in fetched_products:
                command.extend(["--dcb", str(fetched_products["dcb"])])
            if args.nav is not None:
                command.extend(["--nav", str(args.nav)])
            if args.max_epochs > 0:
                command.extend(["--max-epochs", str(args.max_epochs)])
            if args.solver_wall_time_s is not None:
                command.extend(["--solver-wall-time-s", str(args.solver_wall_time_s)])
            if args.use_existing_solution:
                command.append("--use-existing-solution")
            if args.require_valid_epochs_min is not None:
                command.extend(["--require-valid-epochs-min", str(args.require_valid_epochs_min)])
            if args.require_matched_epochs_min is not None:
                command.extend(["--require-matched-epochs-min", str(args.require_matched_epochs_min)])
            payload = run_checked_json(command, summary_path)
        finally:
            if temp_summary_dir is not None:
                temp_summary_dir.cleanup()

        payload["products_signoff_profile"] = "ppc"
        payload["product_presets"] = presets
        payload["product_specs"] = list(args.product)
        payload["fetch_products"] = True
        payload["fetched_products"] = sorted(fetched_products.keys())
        payload["fetched_product_date"] = fetch_payload.get("effective_date")
        payload["sp3"] = str(fetched_products["sp3"]) if "sp3" in fetched_products else None
        payload["clk"] = str(fetched_products["clk"]) if "clk" in fetched_products else None
        payload["ionex"] = str(fetched_products["ionex"]) if "ionex" in fetched_products else None
        payload["dcb"] = str(fetched_products["dcb"]) if "dcb" in fetched_products else None
        payload["reference_csv"] = str(reference_csv)
        payload["run_dir"] = str(run_dir)
        payload["mean_position_error_m"] = payload.get("mean_h_m")
        payload["p95_position_error_m"] = payload.get("p95_h_m")
        payload["max_position_error_m"] = payload.get("max_h_m")

        if args.malib_pos is not None:
            ensure_input_exists(args.malib_pos, "MALIB PPP solution file", ROOT_DIR)
            augment_ppc_malib_comparison(payload, reference_csv, args.malib_pos, args.match_tolerance_s)
        elif args.malib_bin is not None or args.use_existing_malib:
            raise SystemExit("--profile ppc currently supports --malib-pos comparison only")

        enforce_ppc_metric_requirements(payload, args)
        enforce_ppp_requirements(payload, args)
        enforce_comparison_requirements(payload, args)
        summary_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print("Finished PPP products sign-off.")
        print("  profile: ppc")
        print(f"  solution: {out_path}")
        print(f"  summary: {summary_path}")
        return 0

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
    enforce_ppp_requirements(payload, args)
    enforce_comparison_requirements(payload, args)
    summary_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print("Finished PPP products sign-off.")
    print(f"  profile: {args.profile}")
    print(f"  solution: {out_path}")
    print(f"  summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
