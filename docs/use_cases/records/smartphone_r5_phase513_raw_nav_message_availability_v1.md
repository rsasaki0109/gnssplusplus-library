# Phase513 — smartphone raw navigation-message availability

Read only three exact pixel5 supplemental/gnss_log.txt members in
data/gsdc2023/cache/dataset_2023.zip, using Python ZipFile streaming. No archive
extraction, MAT member access, truth access, positioning values interpreted,
or production changes. Classified non-comment lines by their first CSV token
and hashed uncompressed member bytes. A Nav header is not a Nav observation.

| Route | Raw records | Nav records | SHA256 of raw log |
|---|---:|---:|---|
| 2021-08-24-20-32-us-ca-mtv-h | 112833 | 0 | 3473c6cc8d7825973ac725fef21bd70c04edc7c3627fce0e1e72f5ab15771b9b |
| 2023-03-08-21-34-us-ca-mtv-u | 35810 | 0 | fd96db81c209b5a04842e0db8321813863c9530ab58de06d089aaacfca448796 |
| 2022-04-01-18-22-us-ca-lax-t | 51243 | 0 | 2d15859f4e96cc06d3fdbd0d0476170a608956f284f0293014ebc61c7a6f83a6 |

Exact member pattern: dataset_2023/train/ROUTE/pixel5/supplemental/gnss_log.txt.
All three have a commented Nav schema but only Raw non-comment records.
The archive also contains each route's brdc.nav, already audited as RINEX 3.04.
This evidence is limited to these inputs; it does not prove historical CNAV
messages unavailable from all external archives.

Decision: no recoverable phone CNAV payload for the Phase512 ISC hypothesis
in these logs. Do not create a bias table from fitted trajectories, treat
missing ISC as measured zero, or implement an ungrounded TGD multiplier.
Pause that candidate. The larger goal is not blocked: existing native
observable/ionosphere modelling remains inspectable and testable from raw data.
The ordinary FGO builder already scales Klobuchar delay by (fL1/f)^2, so
adding that scale again would double-apply it. Next review prior residual
ionosphere experiments before choosing another candidate. No accuracy claim.
