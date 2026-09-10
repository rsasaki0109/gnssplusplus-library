# Phase354: centering reduces GPS correction regression, not baseline error

Frozen Phase353 completed once in 262.0578312940197 seconds. Both solver
stages converged; full-correction retained identities checked; 81591 P
factors and 3139 exact output rows. One frozen evaluation, one truth read.

H development score 1.163115125953733 m (P50 0.982927816219824,
P95 1.343302435687642). Compare uncentered GPS-only Phase351
1.7792513752546433 m; same-support zero-values Phase330
1.1370801747544546 m; operational baseline 1.0769392017393964 m.
Centered GPS remains worse than zero values by 0.0260349511992784 m and
worse than operational baseline by 0.08617592421433651 m. Not promoted.

Removing per-stream finite full-base-grid medians largely removes the
uncentered GPS regression in this development experiment. This implicates
persistent components under this intervention but does not identify their
physical source: reference geometry, receiver/satellite code offsets and
atmospheric/multipath components remain confounded. It is not a surveyed
base-position estimate or permission to fit offsets from H truth. The
remaining temporal correction is not demonstrated to help either.

Stop treating centering as a performance fix; retain diagnostics default-off.
No claim of heldout/LB performance, no production change, no MAT or saved
positioning inputs. Active 0.782-class objective remains unachieved.
