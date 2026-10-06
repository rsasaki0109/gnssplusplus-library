#!/usr/bin/env python3
"""Run the tracked, project-authored PVA scoring witness without a receiver."""
import argparse
import json
import os
from pathlib import Path

import gnss_pva_evaluate


def main():
    parser = argparse.ArgumentParser(description=__doc__, prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--output-dir", type=Path, required=True, help="Must not exist")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    parents = Path(__file__).resolve().parents
    source = parents[3]/"demo/fixtures" if len(parents) > 3 else Path("__no_source_fixtures__")
    installed = Path(__file__).resolve().parent.parent/"share/libgnsspp/demo"
    fixtures = source if (source/"synthetic_pva.csv").is_file() else installed
    command = ["--estimate", str(fixtures/"synthetic_pva.csv"), "--reference", str(fixtures/"synthetic_reference.csv"),
               "--output-dir", str(args.output_dir)] + (["--plot"] if args.plot else [])
    result = gnss_pva_evaluate.main(command)
    if result: return result
    report = json.loads((args.output_dir/"score.json").read_text(encoding="utf-8"))
    if report["epochs"] != 12 or report["matched_epochs"] != 12 or report["heading_error_over_150_epochs"] != 1:
        raise RuntimeError("synthetic PVA witness failed")
    print("Synthetic scoring witness passed: 12 snapshots, one deliberate half-turn. This is not field accuracy evidence.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
