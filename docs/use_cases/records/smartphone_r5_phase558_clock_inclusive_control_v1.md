# Phase558 — clock-inclusive synthetic rate control

Extended the implicit travel-time test with 9 clock-drift combinations per
geometry (81 total): receiver drift -1e-6/0/1e-6 s/s, satellite drift
-1e-10/0/1e-10 s/s. Synthetic code P(t)=rho(t)+c*br(t)-c*bs(t-rho(t)/c).
Its derivative with respect to receiver clock tag is
[rho_dot+c*br_dot-c*bs_dot*(1-rho_dot/c)]/(1+br_dot).
Central differences agree within 2e-6 m/s. Standalone
/tmp/phase558_clock_derivative passed both tests.

This tests a specifically defined synthetic observable, not whether phone
firmware reports its rate against that exact clock tag. Android API's
uncorrected-clock statement alone does not settle that distinction. Therefore
no receiver-tag denominator or transmit-time correction has been added to
native inference. Existing sign/unit handling remains unchanged.

Further work needs independent physical/reference-model evidence or a
controlled raw-observation simulator comparison, not more truth-selected
drift parameters. No new accuracy evaluation or active job remains here.
