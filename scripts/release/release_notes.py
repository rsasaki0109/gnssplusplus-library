#!/usr/bin/env python3
"""Turn docs/releases/vX.Y.Z.md into the body of the GitHub Release.

The notes link the rest of the docs relatively (``../web_app.md``), which
works in the MkDocs site but not on the GitHub Release page: there such a
link resolves under ``/releases/`` and gives 404. This rewrites every
relative link to its page on the published docs site (``site_url`` in
mkdocs.yml): ``../web_app.md#maps`` becomes ``<site_url>web_app/#maps``, an
``index.md`` becomes its directory and other files keep their name. Absolute
links, in-page anchors and fenced code blocks are left as they are. A link to
a file that is not under the docs directory is an error, so a broken link
fails the release instead of reaching the release page.
"""

from __future__ import annotations

import argparse
import posixpath
import re
import sys
from pathlib import Path


_LINK = re.compile(r"(\]\()([^)\s]+)(\))")
_ABSOLUTE = re.compile(r"^(?:[a-zA-Z][a-zA-Z0-9+.-]*:|//|#)")
_FENCE = re.compile(r"^\s*(```|~~~)")


def mkdocs_site(mkdocs_file: Path) -> tuple[str, Path]:
    """Return (site_url ending in '/', docs directory) from mkdocs.yml."""

    text = mkdocs_file.read_text(encoding="utf-8")
    site = re.search(r"(?m)^site_url:\s*['\"]?([^'\"\s#]+)", text)
    if site is None:
        raise ValueError(f"no site_url in {mkdocs_file}")
    docs = re.search(r"(?m)^docs_dir:\s*['\"]?([^'\"\s#]+)", text)
    url = site.group(1) if site.group(1).endswith("/") else site.group(1) + "/"
    return url, mkdocs_file.parent / (docs.group(1) if docs else "docs")


def page_url(target: str, notes_path: Path, docs_dir: Path, site_url: str) -> str:
    """Map one relative link target in notes_path to its URL on the docs site."""

    path, _, anchor = target.partition("#")
    page = posixpath.normpath(
        posixpath.join(notes_path.resolve().parent.relative_to(docs_dir.resolve()).as_posix(), path)
    )
    if page == ".." or page.startswith("../"):
        raise ValueError(f"{notes_path}: link {target!r} points outside {docs_dir}")
    if not (docs_dir / page).is_file():
        raise ValueError(f"{notes_path}: link {target!r} has no file {docs_dir / page}")
    if page.endswith(".md"):
        stem = page[: -len(".md")]
        if stem == "index":
            page = ""
        elif stem.endswith("/index"):
            page = stem[: -len("index")]
        else:
            page = stem + "/"
    return site_url + page + (f"#{anchor}" if anchor else "")


def rewrite_links(text: str, notes_path: Path, docs_dir: Path, site_url: str) -> str:
    """Rewrite the relative Markdown links of text (the notes at notes_path)."""

    def replace(match: re.Match[str]) -> str:
        target = match.group(2)
        if _ABSOLUTE.match(target):
            return match.group(0)
        return match.group(1) + page_url(target, notes_path, docs_dir, site_url) + match.group(3)

    lines = []
    fenced = False
    for line in text.splitlines(keepends=True):
        if _FENCE.match(line):
            fenced = not fenced
        lines.append(line if fenced or _FENCE.match(line) else _LINK.sub(replace, line))
    return "".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("notes", type=Path, help="release notes, e.g. docs/releases/v0.5.0.md")
    parser.add_argument("--mkdocs", type=Path, default=Path("mkdocs.yml"),
                        help="MkDocs config with site_url (default: mkdocs.yml)")
    parser.add_argument("--output", type=Path, help="write here instead of stdout")
    args = parser.parse_args(argv)
    try:
        site_url, docs_dir = mkdocs_site(args.mkdocs)
        body = rewrite_links(args.notes.read_text(encoding="utf-8"), args.notes, docs_dir, site_url)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    if args.output:
        args.output.write_text(body, encoding="utf-8")
    else:
        sys.stdout.write(body)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
