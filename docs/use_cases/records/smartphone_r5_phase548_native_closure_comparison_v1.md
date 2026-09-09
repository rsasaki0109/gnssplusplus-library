# Phase548 — native raw code/carrier closure does not isolate U failure

Added scripts/native_dual_code_carrier_closure.cpp, compiled standalone and
linked against current native libraries as /tmp/phase548_closure. The loader
uses pixel5, verify_enriched_pseudorange=false and strict frequency-pair timing
(zero offsets/common cross-band raw clock). No saved positions, navigation
positions, truth or MATLAB inputs. Loader quality selection is not final FGO
factor admission. Adjacent dual-band valid code/ADR pairs require no lock loss,
positive <=1.5 s interval and unchanged hardware clock discontinuity count.

Aggregate absolute delta[(P1-P5)+(L1-L5)] in metres:

| Route | Pairs | P50 | P95 | Max |
| --- | ---: | ---: | ---: | ---: |
| H | 20673 | 1.79888 | 7.49013 | 31.4982 |
| U | 4749 | 1.50073 | 6.46321 | 26.3757 |
| LAX | 5732 | 1.50635 | 6.31101 | 23.9794 |

U is not uniquely elevated in this aggregate: these observations do not
support attributing its 80.5 cm positioning regression to larger short-term
closure outliers. Constant code bias remains invisible, so the result neither
confirms nor excludes that alternative. Do not fit a closure threshold from
these route scores or equate closure with ionosphere.

This completes the proposed aggregate closure check. Further repetitions of
these same summaries would not resolve constant-mode ambiguity. Next useful
work should examine a richer constant-mode identifiability control with free
geometry/C7 clocks, or an independent absolute atmospheric/bias constraint,
before another shared-ionosphere accuracy experiment. Existing rejected
selector remains off by default; no runtime source change or promotion here.
