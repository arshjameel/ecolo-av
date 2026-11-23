#!/usr/bin/env python3
"""
Tell CI whether a pull request (or a merge queue entry) changes the docs only.

CI's job `changes` asks this, and a docs-only change skips the build and the Rust and
ROS tests: `linux` runs only the docs' checks, `arm` is skipped, `car-kit` runs no steps.

Docs: every changed file is a Markdown file (anywhere), or in docs/html/ or docs/img/.
One other file (code, a YAML, pixi.lock, the workflow itself) and it's the full run.

  python3 tools/ci_docs_only.py <base sha>     prints docs_only=true or docs_only=false

The base is main as the pull request was tested on, or for the merge queue main without
the entries ahead: so a code change ahead in the queue gives the entries behind it the
full run too. Anything that goes wrong (no base, git failing) prints docs_only=false:
a fault makes a run longer, never lets code through untested. CI runs main's copy of
this script, not the pull request's: a change to it is code, and gets the full run.
"""

import subprocess
import sys

DOCS_FOLDERS = ('docs/html/', 'docs/img/')


def is_doc(path):
    """Say if a path is a doc: Markdown anywhere, or in docs/html/ or docs/img/."""
    return path.endswith('.md') or path.startswith(DOCS_FOLDERS)


def docs_only(paths):
    """Say if there are changed files and every one is a doc."""
    paths = [p for p in paths if p]
    return bool(paths) and all(is_doc(p) for p in paths)


def changed_files(base):
    """
    List the files changed between base and HEAD.

    A rename counts as both names: a .py renamed to .md still changed code.
    """
    subprocess.run(['git', 'fetch', '-q', '--depth=1', 'origin', base], check=True)
    out = subprocess.run(['git', 'diff', '--no-renames', '--name-only', '-z', base, 'HEAD'],
                         check=True, capture_output=True, text=True).stdout
    return out.split('\0')


def main(argv):
    base = argv[1] if len(argv) > 1 else ''
    try:
        if not base:
            raise ValueError('no base commit')
        paths = [p for p in changed_files(base) if p]
    except (subprocess.CalledProcessError, OSError, ValueError) as e:
        print(f'the full run: could not list the changed files ({e})', file=sys.stderr)
        paths = []
    for p in paths:
        print(f"  {'doc ' if is_doc(p) else 'code'}  {p}", file=sys.stderr)
    print(f'docs_only={str(docs_only(paths)).lower()}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
