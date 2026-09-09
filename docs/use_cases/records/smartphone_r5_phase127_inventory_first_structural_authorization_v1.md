# Phase127 independent inventory-first raw authorization

- Execution label: `Luna Max`
- Scope: exactly one structural attempt for MTV-A, then exactly one for LAX-T.
- Authorization JSON: `smartphone_r5_phase127_inventory_first_structural_authorization_v1.json`
- Candidate: `phase127-inventory-first-raw-base-glonass-structural-v1`
- Implementation: `f41d082e5170fe5dcbebbb4526c7d513f9512b60`
- Contract freeze: `5c3e66fc4b62d86b52f8e9ee8aa1f26f4cb2a8a4`
- Runner/manifest contract: `93ee772e595aac01a5de4e7357becd88581f70ec`
- Pre-raw seal: `da8d50cb37e9adebcd6b140b2f67a482276323be`
- Target binary SHA-256: `653797970fbe65f199fc98dfe47ddd57ec26da66fadc4df40dff930a6d1c436a`

This is a new, independent one-shot authorization.  The two route input
paths, byte counts, and digests are inherited as metadata from the sealed
Phase95 raw-input result; base member paths and digests are inherited from
the sealed Phase65 base manifest.  No payload was opened while this
authorization was prepared.

After this commit, Stage 1 may read only each route's raw phone GNSS, raw
phone IMU, broadcast navigation, and sealed raw base RINEX once for compact
inventory.  It must validate the raw base header FCN ledger, exact selected
broadcast `geph.frq` at every existing rover/base query time, `|query-toe| <=
1800 s`, FCN `[-7,6]`, duplicate/tie/conflict/missing accounting, and complete
rover/base GLONASS coverage.  Header FCN is primary; selected time-valid
broadcast FCN is the only permitted corroboration/fallback.  Fixed channels,
carrier-frequency inference, external tables, nearest/extrapolated records,
and repair are forbidden.

Any Stage 1 failure must seal that route as inventory-fail-closed with zero
native solver invocations.  Only a complete inventory may invoke the pinned
Phase126/127/118 command once.  Phase117, Phase120, and additional-frequency
selectors remain off.  Phase126 A/B/C transaction, exactly-once correction,
GNSS-first/main progress and strict cost decrease, QR selection, finite
coverage, C7/D/CCDD handoff, and output metadata are structural gates.

Truth, MAT, PDC, precomputed coordinates/corrections, accuracy, Kaggle,
solution-row opening, fallback, repair, rerun, and publication are not
authorized.  The authorized runner writes only compact inventory/summary
telemetry and opaque output-path presence; it never opens solution rows.
