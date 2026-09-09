# Phase481 — H transfer result; U next

Phase479 completed once, exit 0, 179.5689669289859 seconds; session 48414
closed. verify_phase479_lambda_floor.py passed pins, both floors, preserved
stage-specific solvers, convergence and numerical checks. No failed-lambda
records; GNSS-first/main accepted iterations equal trials (59/43), main
failure count zero. Output SHA:
4a0c4ef8822c72aa9134368cd57fe769817895e63b5dbe925783a9c8d091fa8e.

Evaluation-only comparison to Phase438 matched all 3139 output keys:
mean separation 0.00006682917411953135 m, P50 0.00005750086580240919 m,
P95 0.000290634149149503 m, max 0.0003047567205208613 m. Major graph counts
match: 252384 factors, 15700 values, 3139 IMU intervals, 965 stop pose factors,
988 stop velocity factors and 988 stop epochs. No truth or accuracy scoring.
Small separation bounds same-key same-metric error changes, not unknown-route
performance. No controlled runtime benchmark or promotion.

H terminal result inspected before launching prepared Phase480 U, same fixed
1e-8 floor. Use its launcher completion record/session as live authority.
Launched 2026-09-08T20:48:11.315662Z, session 30191, PID 3737961;
manifest SHA c6014addbda1f401ec48beee412a842060f19677109a4dd1f518d260a0205e4d.
After U completes run verify_phase480_lambda_floor.py and
compare_lambda_floor_transfer.py 480. Preserve pinned files while running.
Default remains OFF; numerical efficiency is not the original accuracy goal.
