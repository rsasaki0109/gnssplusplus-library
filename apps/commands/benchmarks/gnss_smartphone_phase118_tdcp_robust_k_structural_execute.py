#!/usr/bin/env python3
"""Launch-free Phase118 structural runner gate.

The raw execution entry point is intentionally closed in this commit.  The
runner delegates only to the launch-free validator and refuses an execution
request until a later independent authorization pins a dedicated wrapper.
It never opens or stats raw phone/base members, solution rows, truth, MAT, or
coordinate payloads.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
VALIDATOR_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase118_tdcp_robust_k.py"


def load_validator():
    spec = importlib.util.spec_from_file_location("phase118_structural_validator", VALIDATOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load validator: {VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    parser.add_argument(
        "--execute-authorized",
        action="store_true",
        help="reserved for a later independent raw authorization; never enabled here",
    )
    args = parser.parse_args()
    if args.execute_authorized:
        print(
            "phase118 structural runner: fail-closed: independent raw authorization "
            "is not present; no raw or solver execution is available",
            file=sys.stderr,
        )
        return 2
    if not any((args.verify_freeze, args.verify_manifest, args.verify_pre_raw)):
        parser.error("one launch-free verification mode is required")
    try:
        validator = load_validator()
    except (RuntimeError, OSError) as exc:
        print(f"phase118 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2
    try:
        if args.verify_freeze:
            validator.verify_freeze()
        if args.verify_manifest:
            validator.verify_manifest()
        if args.verify_pre_raw:
            import json

            print(json.dumps(validator.verify_pre_raw(), indent=2, sort_keys=True))
        return 0
    except (validator.Phase118ContractError, OSError) as exc:
        print(f"phase118 structural runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
