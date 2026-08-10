# PPC experiments

This directory contains PPC benchmark drivers, offline selectors, parity
replays, and their report generators. Shared benchmark readers and metrics stay
in `scripts/` or `scripts/analysis/`; the scripts here add this directory to
their import path so they can also be run directly from the repository root.

Examples:

```bash
python3 scripts/experiments/ppc/run_ppc_realtime_guard_sweep.py --help
python3 scripts/experiments/ppc/generate_ppc_goal_scorecard.py --help
```
