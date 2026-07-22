# PPC KF/FGO goal completion audit

Overall: **PASS** (15/15).

| Requirement | Status | Observed | Evidence |
|---|---|---|---|
| PPC weighted official score >= 78.7% | PASS | `78.845491` | `docs/ppc_kf_fgo_goal_metrics.json` |
| Tokyo run1 FIX >= 80.8% | PASS | `80.860848` | `docs/ppc_kf_fgo_goal_metrics.json` |
| Correct FIX/reference >= 66.230% | PASS | `66.286505` | `docs/ppc_kf_fgo_goal_metrics.json` |
| Macro Wrong FIX/FIX < 1.5% | PASS | `1.463246` | `docs/ppc_kf_fgo_goal_metrics.json` |
| Total wrong FIX <= 700 | PASS | `574` | `docs/ppc_wrong_fix_event_ledger.json` |
| Wrong FIX above 10 m <= 10 | PASS | `5` | `docs/ppc_wrong_fix_event_ledger.json` |
| Wrong-FIX event ledger is runtime-truth-free | PASS | `{"events": 188, "events_above_10m": 4, "wrong_fix_epochs": 574}` | `docs/ppc_wrong_fix_event_ledger.json` |
| Nagoya 3 106 m root cause and non-inherited hold diagnosis documented | PASS | `"106.130 m; float-KF basin; no prior held integers"` | `docs/ppc_nagoya3_wrong_fix_root_cause.md` |
| KF wrong-basin prevention and FGO recovery design documented | PASS | `"detection/recovery authority separation"` | `docs/ppc_online_consensus_design.md` |
| Online KF/FGO consensus implementation and tests exist | PASS | `{"final_state": "NORMAL", "positions_replaced": 0, "runtime_truth_used": false}` | `include/libgnss++/algorithms/integrity_consensus.hpp; tests/test_integrity_consensus.cpp` |
| Six-fold LOO and extension LOO are complete | PASS | `{"extension_folds": 6, "folds": 6}` | `docs/ppc_kinematic_integrity_loo.json` |
| Active non-PPC receiver holdout catches wrong FIX without truth at runtime | PASS | `{"correct_harmed": 0, "selected": 22, "wrong_caught": 22}` | `docs/ppc_residual_integrity_external_audit.json` |
| CI regression gate covers staged policy | PASS | `"core CTest registrations"` | `tests/CMakeLists.txt` |
| README, tables, figures, and reproduction instructions are current | PASS | `"current staged metrics and external audit linked"` | `README.md; docs/ppc_libgnss_gici_comparison.png; docs/ppc_public_targets.png; docs/ppc_kf_fgo_fix_status_xy.png; docs/ppc_reproduction.md` |
| MIT/GPL boundary is explicit and libgnss++ license remains MIT | PASS | `"GPL-3.0 external executable; NMEA output only"` | `LICENSE; docs/ppc_kf_fgo_goal_metrics.json; docs/ppc_online_consensus_design.md` |
