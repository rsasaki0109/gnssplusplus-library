# Benchmark test cases

The benchmark regression suite is grouped by the behavior it exercises:

| Module | Scope |
| --- | --- |
| `core_metrics.py` | PPC scorecards, status transitions, and shared metrics |
| `clas_iers.py` | CLAS, IERS, VMF, and compact-correction helpers |
| `ppc_selectors.py` | PPC coverage, selector, and IMU bridge workflows |
| `position_bridges.py` | Position, velocity, and FGO bridge helpers |
| `signoffs.py` | Sign-off scripts, commercial-receiver helpers, and CI scope |
| `reports.py` | Comparison, scorecard, and segmented benchmark reports |
| `multi_candidate.py` | PPC demo and multi-candidate selector workflows |

Run the complete suite through the existing compatibility entry point:

```sh
python3 tests/test_benchmark_scripts.py
```

Standard `test*.py` discovery sees the compatibility runner. The domain
filenames intentionally omit the `test` prefix, and `__init__.py` re-exports
no `TestCase` classes; this keeps discovery from collecting the same classes
twice.
