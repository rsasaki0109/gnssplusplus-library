#!/usr/bin/env python3
"""Verify (and optionally export) the frozen inputs of the PPC goal matrix.

The README section "PPC 2024 goal matrix vs Kaiyodai and gici-open" is built
by a truth-free post-processing chain (``docs/ppc_reproduction.md``: tier-3
Tokyo 1 selection, Nagoya 2 wrong-basin escape, Nagoya 3 causal consensus,
kinematic status demotion, Tokyo 3 FGO consensus, Tokyo 1/2 multi-shadow
position consensus, staged residual policy) on top of solver outputs whose
own generation argv was never recorded.  Those solver outputs, the
gici-open trajectories and the Nagoya 1 FIX-target profile are therefore
frozen here by SHA-256; the ``ppc-goal`` reproduce lane replays everything
downstream of them and rescores.

``--inputs-root`` uses the historical ``output/`` layout (the relative paths
below), so the output directory of the checkout that produced the README can
be passed directly.  ``--out`` copies the 26 files into a fresh directory with
the same layout (about 33 MB) for archiving or transfer.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys


# Relative path -> (SHA-256, role).  Recorded 2026-09-29 from the checkout
# that produced README commit a978b227 (files dated 2026-07-18 .. 2026-07-22).
FROZEN_INPUTS: dict[str, tuple[str, str]] = {
    "tokyo1_selected_quality_rtkbaseline_tier2_truthfree.pos": (
        "89463ab75ad4931b93d45bef36f0c843c280a1cd13a8b04149f0af82449ed02d",
        "Tokyo 1 tier-2 selected KF trajectory (tier-3 baseline)",
    ),
    "tokyo2_selected_quality_rtkbaseline_tier2_truthfree.pos": (
        "0a929ac9aac94c951eb4b902b245868e33f2a856aa5a41a21fe3fa2c83cafaa4",
        "Tokyo 2 tier-2 selected KF trajectory",
    ),
    "tokyo3_selected_quality_rtkbaseline_tier2_truthfree.pos": (
        "853022d62807b65c4b29bd5ca16590817b452e2a2e36db651b0fc2a0e0dddfb5",
        "Tokyo 3 tier-2 selected KF trajectory",
    ),
    "tc_m3_full_t1_on/rtk.pos": (
        "c1dabd747000711781aae2cc08ba8195e240fd07dbfbde76fb00e084dd1b5ac0",
        "Tokyo 1 tightly-coupled RTK candidate (gnss_fuse, low-cost preset)",
    ),
    "nagoya2_selected_quality_rtkbaseline_tier2_truthfree.pos": (
        "773c5cb7e16634b5f069b446ab4a832114f6a6294fe4fbdd9ab3da67883c91de",
        "Nagoya 2 tier-2 selected KF trajectory",
    ),
    "probe_fuse_nagoya2_full_tc_m4.pos": (
        "1b9bb07468b05ca9eea94f4cf09bc966880363ba7e9654196fdfe10c973670c5",
        "Nagoya 2 tightly-coupled FGO FLOAT escape candidate",
    ),
    "hybrid_nagoya1_multistage_m4_fixedpos_bridge05_vertical025_veld_vertical10_truthfree.pos": (
        "512429e4979f7fbc6dcc64480f35395a8bc8335b0ae46649c75f974cc68a0989",
        "Nagoya 1 selected hybrid trajectory",
    ),
    "nagoya3_selected_quality_rtkbaseline_truthfree.pos": (
        "0dfdd4678168616c4d7a2433eea42b28e0c84b03a5f22660798a5dfdd49f7ede",
        "Nagoya 3 selected KF trajectory (consensus primary)",
    ),
    "fgo_partial_noreset_ddpranchor_nagoya3_first2100_ecef.csv": (
        "a798b36956f5cf44980e44428f196cadfa9f54ac27ba7bf45b0fbf03d4a012b5",
        "Nagoya 3 FGO shadow window 1",
    ),
    "fgo_partial_noreset_ddpranchor_nagoya3_start2000_3000_ecef.csv": (
        "eebaa97dc6ec4d027f78428f92b771f598bb72831ea0b9dc9634033452ed6636",
        "Nagoya 3 FGO shadow window 2",
    ),
    "fgo_shipping_tokyo3_start10950_400_ecef.csv": (
        "307adb33894a52cc6cd94a13203b55f1aac480b833df9f539b6ea677f6bc798e",
        "Tokyo 3 FGO shadow window (start 10950)",
    ),
    "fgo_shipping_tokyo3_start11150_400_ecef.csv": (
        "1539a4853c3a398f11e7a9f70bd583657bc07f858642d4578416165fbdef3ceb",
        "Tokyo 3 FGO shadow window (start 11150)",
    ),
    "fgo_shipping_tokyo1_full_ecef.csv": (
        "7e4f387eb26eda06b0d113bfe23e144d474d71687ece3fae580dd3abce15e0fa",
        "Tokyo 1 full-run FGO shadow",
    ),
    "fgo_shipping_nhc_tokyo1_start6500_3500_ecef.csv": (
        "b867469991e273c4bdafeaeffee59bff4d468c688f0c894b9a9eec1a4605fe9d",
        "Tokyo 1 restarted FGO shadow (start 6500)",
    ),
    "fgo_shipping_nhc_tokyo1_start7100_500_ecef.csv": (
        "e7efd12098a136c138b1e793014201eea38de5bc28113c1085402f95f84b0574",
        "Tokyo 1 restarted FGO shadow (start 7100)",
    ),
    "fgo_shipping_nhc_tokyo1_start8000_1000_ecef.csv": (
        "b8d6b17fb12e3ff3d640b2028a905bba2eb3d3ffc67e4300f98f0faa694a9a94",
        "Tokyo 1 restarted FGO shadow (start 8000)",
    ),
    "fgo_shipping_nhc_tokyo1_start9400_700_ecef.csv": (
        "f8a4d8d9881f08d122f910cc928b8b52279b6aee67dffe3525c6db724f31198b",
        "Tokyo 1 restarted FGO shadow (start 9400)",
    ),
    "fgo_shipping_tokyo2_full_ecef.csv": (
        "b046e66d3c651e6727d030155981f8f265de12baaa711df4f17ac9a9e751084e",
        "Tokyo 2 full-run FGO shadow",
    ),
    "fgo_shipping_nhc_tokyo2_start3000_1000_ecef.csv": (
        "9a81f6c9263c60f9ce3fc340c78abfa0282eb1a41234823d985ee50a8b44fc41",
        "Tokyo 2 restarted FGO shadow (start 3000)",
    ),
    "gici_common/tokyo1.pos": (
        "d90485687f6a29deb91920b81b19042653226befa9001b65e74ee7220a14c239",
        "gici-open e7666110 Tokyo 1 (NMEA converted with convert_gici_nmea_to_pos.py)",
    ),
    "gici_common/tokyo2.pos": (
        "d54276c4e98fb2d4780bdb731e735317d8be8d42f28172e55f2b2875416c0782",
        "gici-open e7666110 Tokyo 2",
    ),
    "gici_common/tokyo3.pos": (
        "258c71d40e60c53d8ae7fac9353dc1cf2055037f4bb06b5cb569a9587c8959f3",
        "gici-open e7666110 Tokyo 3",
    ),
    "gici_common/nagoya1.pos": (
        "ad454586694ffa5923a3fd6a863c1781d7247498043f2b60068e3b13e94cbd04",
        "gici-open e7666110 Nagoya 1",
    ),
    "gici_common/nagoya2.pos": (
        "960a1b592e994c1d95b0217d08c6fa1639025dfc58c07bca87c11f0dce21061a",
        "gici-open e7666110 Nagoya 2",
    ),
    "gici_common/nagoya3.pos": (
        "5a7dc8e939c4fb0a8d33bc0be3d4333a4d045cb0de649b46fb98eca048c3f00f",
        "gici-open e7666110 Nagoya 3",
    ),
    "goal_kf_current_r2_min8_rate20_rescue29_8/solution.pos": (
        "168adfaa200893410c1065a75dd9e1e503ffeb5276daab9d07729b64f8e03cd1",
        "Nagoya 1 FIX-target profile (gnss ppc-demo, separate from the matrix)",
    ),
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify(inputs_root: Path) -> tuple[list[dict[str, object]], list[str]]:
    rows: list[dict[str, object]] = []
    problems: list[str] = []
    for relative, (expected, role) in FROZEN_INPUTS.items():
        path = inputs_root / relative
        if not path.is_file():
            problems.append(f"missing: {relative}")
            rows.append({"path": relative, "role": role, "sha256": None, "ok": False})
            continue
        observed = sha256_file(path)
        ok = observed == expected
        if not ok:
            problems.append(f"SHA-256 mismatch: {relative} ({observed} != {expected})")
        rows.append(
            {
                "path": relative,
                "role": role,
                "bytes": path.stat().st_size,
                "sha256": observed,
                "ok": ok,
            }
        )
    return rows, problems


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--inputs-root", type=Path, required=True,
                        help="directory holding the frozen inputs in the historical output/ layout")
    parser.add_argument("--out", type=Path, default=None,
                        help="optional directory to copy the verified inputs into (same layout)")
    parser.add_argument("--summary-json", type=Path, default=None,
                        help="write the per-file verification table here")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    rows, problems = verify(args.inputs_root)
    total = sum(int(row.get("bytes", 0) or 0) for row in rows)
    print(f"{sum(bool(row['ok']) for row in rows)}/{len(rows)} frozen inputs verified "
          f"({total / 1e6:.1f} MB) under {args.inputs_root}")
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(
            json.dumps({"schema": "ppc_goal_inputs.v1", "passed": not problems, "files": rows}, indent=2) + "\n",
            encoding="utf-8",
        )
    if problems:
        for problem in problems:
            print(f"error: {problem}", file=sys.stderr)
        return 1
    if args.out is not None:
        for relative in FROZEN_INPUTS:
            target = args.out / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(args.inputs_root / relative, target)
        print(f"copied {len(FROZEN_INPUTS)} files to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
