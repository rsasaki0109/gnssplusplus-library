# Community onboarding

Welcome. The fastest path for a first contribution is one small, reviewable
change with one clear user value. This **15-minute first contribution** path
helps you choose a lane, find an appropriately sized issue, and run a focused
check before opening a pull request.

## Route the conversation

- Search the [quick start](quickstart.md), [interfaces](interfaces.md), and
  [validation](validation.md) pages first.
- Ask for usage or documentation help with the
  [question form](https://github.com/rsasaki0109/gnssplusplus-library/issues/new?template=question.yml).
- Report a reproducible regression with the
  [bug report form](https://github.com/rsasaki0109/gnssplusplus-library/issues/new?template=bug_report.yml).
- Propose one focused capability with the
  [feature form](https://github.com/rsasaki0109/gnssplusplus-library/issues/new?template=feature_request.yml).
- Report suspected vulnerabilities privately using
  [SECURITY.md](https://github.com/rsasaki0109/gnssplusplus-library/blob/develop/SECURITY.md);
  do not put them in a public issue.

Good starting searches are [good first issue](https://github.com/rsasaki0109/gnssplusplus-library/issues?q=is%3Aopen+is%3Aissue+label%3A%22good+first+issue%22)
and [help wanted](https://github.com/rsasaki0109/gnssplusplus-library/issues?q=is%3Aopen+is%3Aissue+label%3A%22help+wanted%22).

## Three contribution lanes

Pick one lane and keep the first change narrow. In every lane, finish with
`git diff --check` and describe the exact check in the pull request.

### Docs-only

Edit the smallest page or example, then build the site strictly:

```bash
python3 -m mkdocs build --strict --site-dir /tmp/libgnsspp-site
```

Install the pinned docs requirements from `requirements-docs.txt` first if
MkDocs is not available locally.

### Python tool or test

Keep the change focused on one CLI surface, helper, or pure-Python regression
and run the small CLI UX suite:

```bash
python3 tests/test_cli_ux.py
```

For a new pure-Python contract, add a stdlib-only test under `tests/` and run
that file directly as well.

### C++

For a library, parser, or native CLI change, configure a Release test build
and run the consolidated test target:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DGNSSPP_BUILD_PYTHON_BINDINGS=OFF
cmake --build build --target gnss_run_tests --parallel 2
ctest --test-dir build -R '^run_tests$' --output-on-failure
```

Use a focused existing test or fixture when the full target is not needed,
and state any dataset or credential requirement explicitly.

## Before opening the pull request

Keep the branch based on current `develop`, explain the user-visible value,
name non-goals, and include the smallest passing command. The longer
[contributor guide](contributing.md) covers PR boundaries and the broader CI
lanes.
