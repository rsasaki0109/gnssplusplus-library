#!/usr/bin/env python3
"""Classify changed paths to decide whether heavy CI lanes should run."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Sequence


DOCS_ONLY_EXACT_PATHS = {
    ".github/workflows/docs.yml",
    "README.md",
    "CONTRIBUTING.md",
    "mkdocs.yml",
    "requirements-docs.txt",
    "scripts/generate_architecture_diagram.py",
}
DOCS_ONLY_PREFIXES = (
    "docs/",
    "notes/",
)


def normalize_paths(paths: Sequence[str]) -> list[str]:
    cleaned = {path.strip().lstrip("./") for path in paths if path.strip()}
    return sorted(cleaned)


def is_docs_only_path(path: str) -> bool:
    if path in DOCS_ONLY_EXACT_PATHS:
        return True
    return any(path.startswith(prefix) for prefix in DOCS_ONLY_PREFIXES)


def classify_changed_paths(paths: Sequence[str]) -> dict[str, object]:
    normalized = normalize_paths(paths)
    docs_only = bool(normalized) and all(is_docs_only_path(path) for path in normalized)
    return {
        "changed_paths": normalized,
        "docs_only": docs_only,
        "run_heavy": not docs_only,
    }


def render_markdown_summary(payload: dict[str, object]) -> str:
    docs_only = bool(payload["docs_only"])
    run_heavy = bool(payload["run_heavy"])
    changed_paths = list(payload["changed_paths"])
    lines = [
        "## CI Scope",
        "",
        f"- `docs_only`: `{str(docs_only).lower()}`",
        f"- `run_heavy`: `{str(run_heavy).lower()}`",
        f"- `changed_paths`: `{len(changed_paths)}`",
        "",
    ]
    if changed_paths:
        lines.append("Changed paths:")
        lines.append("")
        for path in changed_paths[:20]:
            lines.append(f"- `{path}`")
        if len(changed_paths) > 20:
            lines.append(f"- `...` ({len(changed_paths) - 20} more)")
    else:
        lines.append("Changed paths: `(none)`")
    lines.append("")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--paths-file", type=Path, required=True)
    parser.add_argument("--github-output", type=Path, default=None)
    parser.add_argument("--github-step-summary", type=Path, default=None)
    return parser.parse_args()


def append_github_outputs(path: Path, payload: dict[str, object]) -> None:
    lines = [
        f"docs_only={'true' if payload['docs_only'] else 'false'}",
        f"run_heavy={'true' if payload['run_heavy'] else 'false'}",
        f"changed_paths_json={json.dumps(payload['changed_paths'])}",
    ]
    with path.open("a", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def append_github_step_summary(path: Path, payload: dict[str, object]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(render_markdown_summary(payload))
        handle.write("\n")


def main() -> int:
    args = parse_args()
    payload = classify_changed_paths(args.paths_file.read_text(encoding="utf-8").splitlines())
    print(json.dumps(payload, indent=2))
    if args.github_output is not None:
        append_github_outputs(args.github_output, payload)
    summary_path = args.github_step_summary
    if summary_path is None:
        summary_env = os.environ.get("GITHUB_STEP_SUMMARY", "").strip()
        summary_path = Path(summary_env) if summary_env else None
    if summary_path is not None:
        append_github_step_summary(summary_path, payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
