# MALIB Operations Gap Analysis

This page compares the current `libgnss++` operations and validation tooling
against the areas where `MALIB` is the most relevant primary reference.

Primary upstream:

- [JAXA-SNU/MALIB](https://github.com/JAXA-SNU/MALIB/tree/feature/1.2.0)
- [MALIB readme.md](https://github.com/JAXA-SNU/MALIB/blob/feature/1.2.0/readme.md)

## Scope

`MALIB` matters here because it is the strongest public reference for:

- `rtkrcv` / `rnx2rtkp` operational flow around `MADOCA-PPP`
- replay-friendly sample-data workflow
- config-file-driven execution
- packaged test data and unit tests

For libgnss++, this is the main upstream reference for **operations,
validation, and replay ergonomics**, not for native solver architecture.

## Current libgnss++ baseline

Relevant code entrypoints today:

- `apps/gnss_live_signoff.py`
- `apps/gnss_ppp_static_signoff.py`
- `apps/gnss_ppp_kinematic_signoff.py`
- `apps/gnss_rtk_kinematic_signoff.py`
- `apps/gnss_ppc_demo.py`
- `apps/gnss_ppc_rtk_signoff.py`
- `apps/gnss_web.py`
- `tests/test_packaging.py`
- `tests/test_web_ui.py`
- `.github/workflows/ci.yml`

## Already aligned enough to build on

| Area | libgnss++ status |
|---|---|
| Replay-oriented validation | `live-signoff`, PPP sign-offs, RTK sign-offs, Odaiba benchmark, and PPC sign-offs already exist |
| Installed-prefix dogfooding | Packaging tests and installed command smoke tests are present |
| Browser-level operational visibility | `gnss web` exposes live sign-offs, benchmark summaries, PPC summaries, and receiver status |
| Optional external dataset CI hooks | PPC RTK sign-off can run in CI when datasets are present |
| Machine-readable outputs | sign-off commands emit summary JSON rather than only console text |

## Main gaps versus a MALIB-style operations stack

| Gap | Current libgnss++ state | Why it matters |
|---|---|---|
| A single packaged replay dataset with one documented `real-time` and one `post-process` golden path | Current repo has several bundled samples and optional datasets, but not one MALIB-style named replay package with a single operational narrative | A very clear sample workflow reduces onboarding cost |
| Config-driven solver profiles for operations | Current CLI favors explicit flags and small sign-off wrappers over reusable named config files | Config files are useful when users want reproducible operational profiles without copying long commands |
| `rtkrcv`-style interactive live console | `gnss rcv` is operationally useful, but it is not an interactive RTKLIB-style console | Some operators expect that style of workflow |
| Larger in-tree unit-test corpus around low-level PPP support functions | libgnss++ has broad unit coverage, but MALIB’s `test/utest` style shows the value of many small component tests around ionex, precise ephemeris, filter math, and misc helpers | This can accelerate solver-side refactors safely |
| Built-in replay data packaging for docs/demo | Current external demos like PPC remain optional and user-supplied | A small official replay bundle would make docs and CI stories stronger |

## Important design difference

`MALIB` is built around RTKLIB-style executables and config-driven operation.

`libgnss++` is built around:

- one `gnss` dispatcher,
- explicit sign-off wrappers,
- JSON-first summaries,
- browser-visible local status pages,
- installable non-GUI tooling.

So the goal is not to become a config-file clone of `MALIB`.

The goal is to import the strong parts of MALIB’s operational style:

- easy replay,
- clear sample data,
- obvious runbooks,
- low-level regression breadth.

## What should *not* be copied directly

Do not grow a parallel RTKLIB-style runtime surface only because MALIB has one.

Keep these libgnss++ constraints:

- `gnss` as the main entrypoint,
- sign-off commands as quality gates,
- browser-visible artifacts and JSON summaries,
- native module boundaries instead of config-only behavior.

## Immediate work items derived from this gap

1. Decide whether a small official replay bundle should be checked into the repo or published separately.
2. Decide whether named solver profile files are worth adding on top of the current CLI.
3. Expand low-level unit coverage in the areas that still gate PPP/CLAS refactors.
4. Strengthen docs around one “official replay story” for `live` and one for `ppp`.

## Exit condition for this analysis

This page is considered "done enough" when the operations roadmap is clear
about:

- whether replay/config ergonomics need new user-visible commands,
- whether sample data should be packaged more explicitly,
- and which low-level regression families are still worth expanding.
