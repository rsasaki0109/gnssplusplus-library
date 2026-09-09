# Smartphone R5 Phase136 wrapper forbidden-token boundary audit

- Execution label: `Luna Max`
- Audit date: `2026-09-04`
- Repository: `/home/sasaki/workspace/rtklib_v2_ws/gnssplusplus-library`
- Starting commit: `49b9db9197f395c6cc302d9ae202c17616afa33e`
- Scope: read-only audit of the sealed Phase135 structural result, the
  authorized Phase135 wrapper, and the native CLI option ownership.  This
  audit does not open any raw phone GNSS/IMU, navigation, or base payload,
  solution row, truth row, MAT/PDC/precomputed-coordinate artifact, or Kaggle
  resource.  It does not launch the native process or rerun a route.

## Disposition

Exactly one wrapper-boundary correction is selected:

`phase136-typed-argv-path-forbidden-token-boundary-v1`

The correction changes only the authorized wrapper's command preflight.  CLI
option tokens are classified by exact option ownership; input path/value
tokens are classified by the existing forbidden path terms.  The required
native algorithm option
`--native-pdc-imu-tdcp-no-bridge` is accepted exactly once because it belongs
to the pinned Phase135 recipe.  A PDC file/path/value, a forbidden artifact,
or an unknown selector remains fail-closed.  No native algorithm source,
graph, factor, equation, unit, recipe, solver, LM setting, or output policy
is changed.

The previous independent authorization is not reusable after this wrapper
source changes.  A new launch-free qualification and a new independent raw
authorization are required before any payload is materialized or any native
process is launched.

## Authoritative evidence

| item | value |
|---|---|
| sealed Phase135 result commit | `49b9db9197f395c6cc302d9ae202c17616afa33e` |
| sealed result JSON | `docs/use_cases/records/smartphone_r5_phase135_official_affine_structural_raw_result_v1.json` |
| sealed result JSON SHA-256 | `3d56dcab6472f463041ddeb7d0c8c7b893594f807ef9c891e9693c2baf0f51db` |
| prior authorization commit | `15ea056c9604b4e53a58d85e23d435e97936d8fa` |
| prior authorized wrapper SHA-256 | `fd322c928be37d0a1d168a48d02d61f9fee2363046ad7ee8a3db0a649d484ce9` |
| wrapper under audit | `apps/commands/benchmarks/gnss_smartphone_phase135_official_affine_structural_authorized_execute.py` |
| native CLI source | `apps/native/gnss_fgo_imu_no_base.cpp` |
| native CLI source SHA-256 | `4ce95d3ef322c83fca2aa5327e9150ee2d2890fe012d6cc4d3b80d439939656e` |

The sealed result records the exact failure: MTV-A read and hash-checked its
four authorized raw inputs, then the wrapper rejected the required token
`--native-pdc-imu-tdcp-no-bridge`; native solver invocation and structural
summary production were zero.  Route sequencing then fail-closed before any
LAX-T payload read.  This is evidence of the wrapper predicate, not evidence
about route data or solver behavior.

## Ownership proof

The native source advertises the option in `usage()` at approximately
`apps/native/gnss_fgo_imu_no_base.cpp:321-323`, parses it at approximately
`:524-525` into `Options::native_pdc_imu_tdcp_no_bridge`, and treats it as an
in-process recipe switch at approximately `:654-663`.  The same source
separately owns raw input paths (`--android-gnss`, `--android-imu`, `--nav`,
and `--native-base-rinex`) and rejects MATLAB paths.  Thus the spelling
contains the historical `pdc` substring but is not a PDC input/artifact
path.  Phase135's static runner also lists it once in
`REQUIRED_RECIPE_FLAGS`; removing or renaming it would change the pinned
native recipe.

The wrapper currently defines one `FORBIDDEN` substring tuple containing
`.mat`, `truth`, `ground_truth`, `pdc`, `precomputed`, `kaggle`, and `token`,
then applies it to every command token in
`verify_command_raw_only()` (`:277-281`).  This mixes two namespaces:

1. path/access values, where those terms must remain forbidden; and
2. exact argv options, where the required native algorithm selector is
   legitimate and must be checked by ownership/count instead.

The sealed result's failure predicate is therefore precisely:

`any(forbidden_path_term in lowered_argv_token)`

with no distinction between an option token and a path/value token.

## Frozen boundary policy

The implementation must enforce all of the following before launch:

- Every option token must be an exact option in the static Phase135 command
  template; unknown options fail closed.
- Every required recipe option, including
  `--native-pdc-imu-tdcp-no-bridge`, appears exactly once.
- Every on-selector appears exactly once and every off-selector is absent.
- Values following path-bearing options are checked as input/artifact paths;
  any forbidden term, including `pdc`, remains rejected.
- A standalone/non-option value containing a forbidden term remains rejected;
  this covers PDC path, PDC artifact, truth/MAT/precomputed/Kaggle/token
  values without relying on substring matching against legitimate options.
- Output metadata may retain only the opaque solution filename exception
  already frozen by Phase135; solution coordinate content remains unread.
- The native command is not launched by qualification or audit.  Any changed
  wrapper hash invalidates the prior raw authorization.

This is a single namespace/boundary correction.  It does not authorize raw
reads, solver execution, truth evaluation, accuracy, rerun, fallback, repair,
sweep, or publication.

## Required qualification evidence

Launch-free synthetic tests must prove that the exact required PDC-named
algorithm option passes, while a PDC path, PDC value, forbidden artifact
value, unknown selector, duplicate required option, and forbidden/off
selector fail closed.  They must also prove exact argv counts and preserve
the static command template.  Source checks must continue to verify native
parser/usage ownership.  Raw phone/nav/base reads, solver invocations, truth
reads, solution-coordinate reads, and MAT/PDC/precomputed/Kaggle reads remain
zero.

