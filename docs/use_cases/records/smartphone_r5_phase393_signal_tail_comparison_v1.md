# Phase392/393 — signal-separated TDCP evidence

Both one-shot raw runs completed (LAX-T 259.3263403410092 s; H
325.30725492804777 s). Both verification scripts passed: original baseline
candidate SHA identities unchanged, all residuals finite, and grouped counts,
tail counts, Huber costs and squared residuals reconcile with whole totals.
No truth reads, scoring or inference from persisted positions.

| Signal | LAX-T count / tail | H count / tail | LAX-T Huber cost | H Huber cost |
| --- | ---: | ---: | ---: | ---: |
| GPS L1 C/A | 7623 / 953 | 16047 / 780 | 48698.6625 | 49583.1639 |
| GPS L5 | 3507 / 646 | 15989 / 3221 | 21492.7637 | 91328.2208 |
| GLONASS L1 C/A | 2230 / 132 | 3988 / 45 | 6662.9417 | 2995.5544 |
| Galileo E1 | 4159 / 878 | 8758 / 839 | 23262.7292 | 26105.2969 |
| Galileo E5a | 7445 / 1650 | 12049 / 2677 | 53329.6781 | 75603.3376 |
| BeiDou B1I | absent | 12439 / 711 | absent | 47259.3471 |

Enum labels verified against `include/libgnss++/core/types.hpp`. These are
aggregate postfit reconstructions, not measurement-error or truth statistics.

Largest LAX-T absolute residual is GLONASS L1 (2.2056 m), but its Huber cost
is only about 4.34% of the total. Thus selecting GLONASS removal solely from
the maximum would ignore robust influence. Galileo E5a has nearly the same
tail fraction on both routes (~22.2%); GPS L1 and Galileo E1 show substantially
larger tail fractions on LAX-T. No single uniquely failing signal is proven.

Checked raw GNSS `MessageType=Raw` constellation counts without reading any
coordinate columns: LAX-T Android IDs 1=18372, 3=8873, 6=23998; H IDs 1=45810,
3=12367, 4=3051, 5=19667, 6=31938. LAX-T contains no raw BeiDou (ID 5) rows;
its missing BeiDou TDCP is therefore not a new graph admission loss.

The larger global tail fraction partly compares different signal mixtures.
Next inspect the GPS L1 / Galileo E1 measurement equations and source noise
assumptions, accounting for raw signal availability. Do not introduce a
route-specific constellation switch or exclude GLONASS from this maximum.
No accuracy gain has been demonstrated; the full objective remains unmet.
