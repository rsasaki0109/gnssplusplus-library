# Phase545 — fixed joint model paired development results

LAX evaluated once after numeric verification and magnitude review. Both
outputs matched all 1466 raw UTC keys; removed exactly the first row from
each for evaluation only, retaining other bytes unchanged. Original hashes,
raw pin and projection policy frozen before projected artifacts; projected
hashes frozen before scoring. No projection was supplied to inference.

LAX matched 1465/1465 truth rows, all finite, no over-70-m/s events. Baseline
3.1026030892587917 m; candidate 2.9065180213176047 m; delta
-0.19608506794118696 m. Candidate P50 2.710369129505412 m and P95
3.1026669131297977 m. Metric is (P50+P95)/2, evaluation-only paired reads.

Same frozen 3 m / 0.02 m/sqrt(s) / 1.5 s model across development routes:
- H: 1.076918 -> 1.059536 m, -0.017382 m.
- U: 1.305960 -> 2.110805 m, +0.804845 m.
- LAX: 3.102603 -> 2.906518 m, -0.196085 m.

No promotion: U degradation exceeds gains elsewhere. These reused training
routes do not prove generalization or leaderboard performance. Do not choose
the model by route or retune priors against observed truth. Goal remains unmet.
All three native jobs terminal; no submissions or native source changes here.
Next investigate raw-observable separation of constant code/systematic errors
from residual ionosphere, informed by Phase542's counterexample, before any
new accuracy candidate. Preserve this complete negative transfer experiment.
