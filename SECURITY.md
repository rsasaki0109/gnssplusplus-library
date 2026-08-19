# Security policy

## Supported versions

The `v0.2.x` line is supported on a best-effort basis. Older versions may be
useful for reproducing history but do not receive a security response promise.

## Reporting a vulnerability

Please do not put vulnerability details in a public issue, pull request, or
support form. Use GitHub's
[private vulnerability reporting](https://github.com/rsasaki0109/gnssplusplus-library/security/advisories/new)
to send the report to the maintainers. The repository owner will enable this
feature after this policy merges; if the private-report link is unavailable,
send a private report to `rsasaki0109@gmail.com` instead.

Useful reports include the affected version or commit, a minimal
reproduction, impact, and any input or parser boundary involved. Remove
credentials, private datasets, and other sensitive material before sending.

## Scope

Reports are in scope for vulnerabilities in GNSS input and parser handling
(including RINEX, RTCM, UBX, SBF, NMEA, and BINEX), network streams or product
fetching, the CLI and Python bindings, installed tooling, and release or
container packaging. Ordinary field-accuracy differences, expected errors on
malformed inputs, and local environment problems are not security reports by
themselves.

## Response expectations

Security handling is best-effort and has no SLA. Maintainers will acknowledge
reports when practical, investigate reproducible impact, and coordinate a
fix or mitigation when warranted. Do not assume a report is accepted until a
maintainer confirms it privately.
