#!/usr/bin/env python3
"""Run baro OFF vs ON standalone SPP on Nantes/Mimir runs and score them.

Pipeline per run: Mimir adapter (rover.obs + baro.csv) -> native SPP with
barometer OFF -> native SPP with barometer ON -> score both (and the Android
Fix.csv chipset comparator) against the Awinda reference with a *fixed* lag.
The lag is part of the frozen configuration; it is never tuned per run here.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

import gnss_smartphone_baro_eval as ev

HERE = Path(__file__).resolve().parent
ADAPTER = HERE / "gnss_smartphone_mimir_adapter.py"
SOURCE_URL = "https://zenodo.org/records/12566912"
SOURCE_TERMS = "CC-BY-4.0 (Zenodo record 12566912; LICENSE.txt in archive)"


def run(command: list[str], log: Path) -> None:
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    log.write_text(
        "$ " + " ".join(command) + "\n" + result.stdout + result.stderr, encoding="utf-8"
    )
    if result.returncode != 0:
        raise SystemExit(f"command failed ({result.returncode}): {' '.join(command)}\n{result.stderr}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    p.add_argument("--dataset-root", type=Path, required=True,
                   help="extracted Android_GNSS_Dataset_Nantes/ tree (contains dataset/)")
    p.add_argument("--scenario", required=True, help="e.g. S3 or S4")
    p.add_argument("--runs", nargs="+", required=True, help="e.g. A2 A4 A6")
    p.add_argument("--broadcast-nav", type=Path, required=True)
    p.add_argument("--approx-llh", required=True)
    p.add_argument("--spp-binary", type=Path, required=True)
    p.add_argument("--output-root", type=Path, required=True)
    p.add_argument("--lag-s", type=float, default=0.0,
                   help="Awinda timestamp minus GPST (frozen to 0.0 in the protocol)")
    p.add_argument("--level-split-m", type=float, default=3.0)
    p.add_argument("--baro-arg", action="append", default=[],
                   help="extra argument for the ON run, repeatable (e.g. --baro-arg=--baro-sigma-m=1.0)")
    p.add_argument("--spp-arg", action="append", default=[],
                   help="extra argument for BOTH the OFF and ON runs (common QC), repeatable")
    p.add_argument("--reuse-off", action="store_true",
                   help="reuse an existing OFF solution (same common args) instead of re-running")
    p.add_argument("--tag", default="on", help="name of the ON variant (output sub-directory)")
    p.add_argument("--adapter-signal-set", choices=("legacy-l1-e1", "multi"), default="legacy-l1-e1",
                   help="Mimir adapter signal set (default: the frozen #564 L1/E1 behaviour)")
    p.add_argument("--adapter-arg", action="append", default=[],
                   help="extra adapter argument, repeatable (e.g. --adapter-arg=--hatch-window-s "
                   "--adapter-arg=30)")
    p.add_argument("--skip-adapter", action="store_true")
    p.add_argument("--no-reference", action="store_true",
                   help="run OFF/ON only (runs without an Awinda reference)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    out_root = args.output_root
    per_run: dict[str, dict] = {}
    matched_by_method: dict[str, list[list[dict]]] = {"off": [], "on": [], "fix": []}
    window_epochs: list[int] = []
    start_alts: list[float] = []
    common_offsets: list[float | None] = []
    sys.path.insert(0, str(HERE))
    for run_name in args.runs:
        d = args.dataset_root / "dataset" / args.scenario / run_name
        gp7 = d / "GP7"
        work = out_root / f"{args.scenario}_{run_name}"
        adapter_dir = work / "adapter"
        work.mkdir(parents=True, exist_ok=True)
        if not (args.skip_adapter and (adapter_dir / "summary.json").is_file()):
            run(
                [
                    sys.executable, str(ADAPTER),
                    "--raw", str(gp7 / "Raw.csv"), "--psr", str(gp7 / "PSR.csv"),
                    "--output-dir", str(adapter_dir),
                    "--dataset-id", f"nantes-{args.scenario}-{run_name}-GP7",
                    "--source-url", SOURCE_URL, "--source-terms", SOURCE_TERMS,
                    "--approx-llh", args.approx_llh,
                    *(["--signal-set", "multi"] if args.adapter_signal_set == "multi"
                      else ["--enable-galileo-e1"]),
                    "--broadcast-nav", str(args.broadcast_nav),
                    *args.adapter_arg,
                ],
                work / "adapter.log",
            )
        common = [
            str(args.spp_binary), "--quiet",
            "--obs", str(adapter_dir / "rover.obs"), "--nav", str(args.broadcast_nav),
        ]
        common += list(args.spp_arg)
        if not (args.reuse_off and (work / "off.pos").is_file()):
            run(common + ["--out", str(work / "off.pos"), "--summary-json", str(work / "off.json")],
                work / "off.log")
        on_dir = work / args.tag
        on_dir.mkdir(exist_ok=True)
        run(
            common
            + [
                "--out", str(on_dir / "on.pos"), "--summary-json", str(on_dir / "on.json"),
                "--baro-height", "--baro-csv", str(adapter_dir / "baro.csv"),
                "--baro-telemetry-csv", str(on_dir / "telemetry.csv"),
            ]
            + list(args.baro_arg),
            on_dir / "on.log",
        )
        info: dict = {"run": run_name}
        info["closure"] = {
            "off": ev.closure(ev.read_pos(work / "off.pos")),
            "on": ev.closure(ev.read_pos(on_dir / "on.pos")),
        }
        if not args.no_reference:
            awinda = d / "AWINDA" / "pos_awinda_60hz.csv"
            rows = ev.read_awinda(awinda)
            ref = ev.Reference(rows, args.lag_s)
            epochs = ev.read_epoch_times(adapter_dir / "rover.obs")
            off = ev.read_pos(work / "off.pos")
            on = ev.read_pos(on_dir / "on.pos")
            # Android Fix.csv: Location.getTime() is GNSS-derived UTC -> GPST
            # with the nominal 18 s leap offset (no phone-clock correction).
            fix = ev.read_fix(gp7 / "Fix.csv", -ev.GPS_UNIX_OFFSET_S * 1000.0 + 18000.0)
            m_off = ev.match_epochs(off, ref)
            m_on = ev.match_epochs(on, ref)
            m_fix = ev.match_epochs(fix, ref)
            common_off = ev.median([m["dv_raw"] for m in m_off]) if m_off else None
            lo, hi = ref.span
            win = sum(1 for t in epochs if lo <= t <= hi)
            window_epochs.append(win)
            start_alts.append(rows[0][3])
            common_offsets.append(common_off)
            matched_by_method["off"].append(m_off)
            matched_by_method["on"].append(m_on)
            matched_by_method["fix"].append(m_fix)
            kw = dict(
                window_epochs=[win], common_offsets=[common_off],
                start_alts=[rows[0][3]], level_split_m=args.level_split_m,
            )
            info["off"] = ev.summarize([m_off], **kw)
            info["on"] = ev.summarize([m_on], **kw)
            # Fix comparator availability is relative to its own epoch count.
            info["fix"] = ev.summarize(
                [m_fix], window_epochs=[sum(1 for f in fix if lo <= f[0] <= hi)],
                common_offsets=[common_off], start_alts=[rows[0][3]],
                level_split_m=args.level_split_m,
            )
            info["reference"] = {
                "lag_s": args.lag_s,
                "span_gpst_tow": [lo, hi],
                "start_alt_m": rows[0][3],
                "common_height_offset_from_off_m": common_off,
            }
        per_run[run_name] = info
    result: dict = {"scenario": args.scenario, "runs": per_run, "tag": args.tag,
                    "baro_args": args.baro_arg, "spp_args": args.spp_arg, "lag_s": args.lag_s,
                "spp_binary": str(args.spp_binary),
                "adapter_signal_set": args.adapter_signal_set, "adapter_args": args.adapter_arg}
    if not args.no_reference:
        pooled = {}
        for method, runs in matched_by_method.items():
            pooled[method] = ev.summarize(
                runs, window_epochs=window_epochs if method != "fix" else None,
                common_offsets=common_offsets, start_alts=start_alts,
                level_split_m=args.level_split_m,
            )
        result["pooled"] = pooled
    path = out_root / f"results_{args.scenario}_{args.tag}.json"
    path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
