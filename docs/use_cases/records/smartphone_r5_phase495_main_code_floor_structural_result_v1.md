# Phase495 — completed main-only uncertainty-floor structural verification

Single native H run (PID 3770993, exec session 82771) is terminal, return 0,
elapsed 403.111133749946 s. Current verify_phase495_main_floor.py passed.
All 101916 retained P sigmas changed. GNSS-first summary equals the Phase493
baseline exactly; graph aggregate fields except costs/iterations are unchanged.
Both stage solver/floor/finite-cost/complete-trace checks passed; main converged
in 42 iterations. Runtime is not a controlled performance benchmark.

Frozen output SHA256:
548a3dec72dea622f24770195b6b680ea24acf06f93c36f9bad7123a8065b95d

Output differs from baseline, as expected for a changed likelihood, but no
accuracy claim follows. No truth read or scoring yet. Next freeze a separate
one-shot exact-key development evaluation for this output. No parameter sweep,
no MAT or saved-position inference, no submission or default promotion.
Overall .782/leaderboard objective remains active and unmet.
