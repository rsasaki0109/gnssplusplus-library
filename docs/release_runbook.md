# Maintainer release runbook

This runbook covers the tag-driven v0.2.0 release. Release automation is
intentionally started only by an annotated `vMAJOR.MINOR.PATCH` tag pushed to
the repository.

For human-reviewed post-release outreach and reproducible, owner-only growth
measurement, use the [growth launch runbook](growth_launch_runbook.md). It does
not automate posting or change repository publication state.

## Before tagging

1. Merge the validated release change into `develop` and confirm the merge
   commit is the intended release source.
2. Run the targeted release tests, packaging smoke test, and strict MkDocs
   build from a clean checkout.
3. Confirm `CMakeLists.txt` contains the version represented by the tag:

   ```bash
   python3 scripts/release/validate_version.py \
     --tag v0.2.0 --cmake-file CMakeLists.txt
   ```

## Start the release

From the validated `develop` merge commit, create and push the annotated tag:

```bash
git tag -a v0.2.0 -m "libgnss++ v0.2.0"
git push origin v0.2.0
```

The release workflow checks out the event commit, then refuses to proceed
unless the local `v0.2.0` ref is an annotated tag that peels exactly to that
commit and matches CMake. It builds repeatable tag-driven packages for Ubuntu
24.04 amd64 and verifies the tag with `gh release create --verify-tag`; the
workflow never creates a Git tag.

## Verify publication

- Confirm the workflow artifact contains exactly one TGZ, one DEB, and
  `SHA256SUMS`; verify the checksums locally with `sha256sum -c SHA256SUMS`.
- The GitHub Release upload is limited to those three validated regular files;
  CPack staging directories under `dist/` are not release assets.
- The DEB declares Eigen development headers and the Python runtime,
  NumPy, and Matplotlib dependencies, and the workflow installs it in a
  fresh Ubuntu 24.04 amd64 container before running the offline demo.
- Treat the binaries and packages as Ubuntu 24.04 amd64 release artifacts;
  the TGZ is not a universal portability promise.
- Confirm the GitHub Release for `v0.2.0` contains those assets and generated
  notes plus `docs/releases/v0.2.0.md` when that file is present.
- Confirm GHCR has `v0.2.0`, `0.2.0`, and `0.2` aliases, together with the
  normal branch/ref, short-SHA, and `latest` tags.

If a rerun finds an existing release, it uploads the freshly checked assets
with `--clobber`. A version mistake requires a new corrective tag; do not
move or delete a published release tag as part of a routine rerun.
