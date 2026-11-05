#!/usr/bin/env python3
"""Assemble the browser GUI (web/) with the WebAssembly tools into a static site.

    python3 wasm/assemble_site.py --dist build-wasm/wasm/dist --out site/app

Layout of --out:

    index.html, css/, js/      the GUI (web/, without its tests)
    wasm/<tool>.js|.wasm       the Emscripten build (wasm/CMakeLists.txt)
    wasm/build-info.json       version / commit shown in the status bar
    samples/                   demo inputs copied from tracked fixtures

The result needs no server-side support: any static host (GitHub Pages,
``python3 -m http.server``) can serve it.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WEB_DIR = ROOT / "web"
TOOLS = ("gnss_convert", "gnss_rinex_info", "gnss_visibility", "gnss_spp", "gnss_solve", "gnss_fuse", "gnss_ppp")
# Sample inputs offered by the GUI ("Load files" / "Run demo pipeline").
SAMPLES = {
    "rover.rtcm": "tests/fixtures/realtime_stress/rover.rtcm",
    "base.rtcm": "tests/fixtures/realtime_stress/base.rtcm",
    "reference.csv": "tests/fixtures/realtime_stress/reference.csv",
    "imu.csv": "tests/fixtures/realtime_stress/imu.csv",
    "synthetic_ppp.obs": "demo/fixtures/synthetic_ppp.obs",
    "synthetic_ppp.sp3": "demo/fixtures/synthetic_ppp.sp3",
    "synthetic_ppp.clk": "demo/fixtures/synthetic_ppp.clk",
}
SAMPLE_NOTES = """Sample inputs for the libgnss++ browser GUI.

rover.rtcm / base.rtcm / reference.csv / imu.csv: a 40 s excerpt (GPS only,
5 Hz rover, 1 Hz base) of the Precise Positioning Challenge dataset, Tokyo
run1 (MIT licence, (c) 2025 Taro Suzuki), as RTCM3 MSM7 + 1005 + 1019, with
the matching reference trajectory and 100 Hz IMU samples. See
tests/fixtures/realtime_stress/README.md.

synthetic_ppp.*: the project-authored deterministic PPP fixture from
demo/fixtures/ (synthetic; not an accuracy benchmark).
"""


def project_version() -> str:
    text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    match = re.search(r"project\(\s*gnss_lib\s+VERSION\s+([0-9.]+)", text)
    return match.group(1) if match else "unknown"


def git_commit() -> str:
    sha = os.environ.get("GITHUB_SHA", "")
    if sha:
        return sha[:12]
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short=12", "HEAD"],
            cwd=ROOT, capture_output=True, text=True, check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--dist", type=Path, help="Emscripten output directory (build-wasm/wasm/dist)")
    parser.add_argument("--out", type=Path, required=True, help="destination directory (replaced)")
    parser.add_argument(
        "--allow-missing-wasm", action="store_true",
        help="assemble the GUI without wasm tools (Plot works; tools show a banner)",
    )
    args = parser.parse_args()

    missing = []
    if args.dist is None:
        missing = list(TOOLS)
    else:
        for tool in TOOLS:
            for suffix in (".js", ".wasm"):
                if not (args.dist / f"{tool}{suffix}").is_file():
                    missing.append(f"{tool}{suffix}")
    if missing and not args.allow_missing_wasm:
        where = f"in {args.dist}" if args.dist else "(no --dist given)"
        print(f"error: missing wasm outputs {where}: {', '.join(missing)}", file=sys.stderr)
        return 1

    out = args.out
    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(WEB_DIR, out, ignore=shutil.ignore_patterns("test", "README.md"))

    wasm_out = out / "wasm"
    wasm_out.mkdir()
    if not missing:
        for tool in TOOLS:
            for suffix in (".js", ".wasm"):
                shutil.copy2(args.dist / f"{tool}{suffix}", wasm_out / f"{tool}{suffix}")
        (wasm_out / "build-info.json").write_text(
            json.dumps({"version": project_version(), "commit": git_commit(), "tools": list(TOOLS)}, indent=2) + "\n",
            encoding="utf-8",
        )

    samples_out = out / "samples"
    samples_out.mkdir()
    for name, source in SAMPLES.items():
        shutil.copy2(ROOT / source, samples_out / name)
    (samples_out / "README.txt").write_text(SAMPLE_NOTES, encoding="utf-8")

    size = sum(p.stat().st_size for p in out.rglob("*") if p.is_file())
    print(f"assembled {out} ({size / 1e6:.1f} MB){' without wasm tools' if missing else ''}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
