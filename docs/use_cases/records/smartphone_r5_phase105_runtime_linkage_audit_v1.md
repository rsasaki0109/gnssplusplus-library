# Phase105 runtime-linkage audit

## Scope and boundary

This is a read-only audit of the Phase104 loader failure. No Android GNSS,
Android IMU, broadcast navigation, truth, MAT, base, PDC, precomputed
coordinate, Kaggle, or solver input was read. No native rerun is authorized by
this audit.

The Phase104 sealed failure (`b7abeb0`) recorded exactly one attempted child
for MTV-A and one for LAX-T. Each child returned `127` before application
startup with the same loader message:

`libmetis-gtsam.so: cannot open shared object file: No such file or directory`

The corresponding native logs are 148 bytes each and have SHA256
`b7459f3d85a3e29e3a5e62c3e2c42f57ff06cb1310e185f932e179ac0ef5951e`.

## Binary and trusted directory evidence

The pinned Phase104 binary is
`build/apps/gnss_fgo_imu_no_base` (SHA256
`6c2fbf9ed119e84383beb3b7db7f11cf1bce8db7a7e64919a38378373784c5c4`). Its
dynamic section has `RUNPATH=/home/sasaki/.local/lib` and direct NEEDED entries
for `libgtsam.so.4`, `libm.so.6`, `libcholmod.so.5`, the C++/Boost runtime, and
the standard C/C++ libraries. The binary itself has no NEEDED entry for
`libmetis-gtsam.so` or `libcephes-gtsam.so.1`; those are transitive needs of
the trusted GTSAM library.

The trusted directory exists and was inspected read-only:

| path | bytes | SHA256 |
|---|---:|---|
| `/home/sasaki/.local/lib/libgtsam.so.4` (target `libgtsam.so.4.3a2`) | 12240736 | `0bb5858e396250a1afecc990c766bde5255fecc38b22b22bd9964775dc9e47a3` |
| `/home/sasaki/.local/lib/libmetis-gtsam.so` | 511632 | `27d1eafe1948d539734637a685b0459411b56b6b575d443a328ed06742810571` |
| `/home/sasaki/.local/lib/libcephes-gtsam.so.1` (target `libcephes-gtsam.so.1.0.0`) | 38928 | `e28cf8a3f92a13773bad55f1a7c0e319bdd3da32bbd37ed95175bbf1b3800ceb` |

With no `LD_LIBRARY_PATH`, `ldd` resolves `libgtsam.so.4` through the binary
RUNPATH but reports `libmetis-gtsam.so => not found` and
`libcephes-gtsam.so.1 => not found`. With the exact environment
`LD_LIBRARY_PATH=/home/sasaki/.local/lib` prepended, both resolve from that
directory. The complete resolved closure at audit time is pinned in the
Phase105 freeze JSON.

## Historical wrapper comparison

The Phase104 wrapper copied the parent environment, removed truth/MAT/Kaggle
convenience variables, and set locale variables, but omitted
`LD_LIBRARY_PATH`. Its child therefore inherited neither the trusted directory
nor a loader search path for the transitive GTSAM libraries.

The sealed successful-path wrappers use the same exact trusted directory. The
Phase95 wrapper sets
`environment["LD_LIBRARY_PATH"] = "/home/sasaki/.local/lib" + ":" +
existing` when an existing value is present. Phase100 and Phase92 use the same
prepend convention. This is the only observed difference capable of explaining
the Phase104 pre-application loader failure; no solver or graph code is needed
to reach that failure.

## Candidate decision

Exactly one candidate is frozen for Phase105: in the Phase104 execution
wrapper, set the child-only `LD_LIBRARY_PATH` to
`/home/sasaki/.local/lib` followed by the inherited value when nonempty. Do not
modify the parent shell, binary RUNPATH, binary contents, graph, factors,
initialization, filtering, LM, or output contract. The wrapper must retain the
truth/MAT/base/PDC/precomputed/Kaggle exclusions and the exact two-route,
one-shot, no-fallback/no-rerun policy.

This candidate is a runtime-linkage repair only. It does not imply that either
route will pass structural or accuracy gates; those gates remain fail-closed.
