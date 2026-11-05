#!/usr/bin/env python3
"""
Download the upstream source that vendor/*.repos lists, to look things up.

Each vendor/<name>.repos (vcstool's format) lands in vendor/<name>/, every repository at
its pinned commit. Nothing in vendor/ is ours: vendor/COLCON_IGNORE keeps colcon out, and
.gitignore keeps git out. See vendor/README.md.

usage: tools/vendor.py [--check] [name ...]   download what's missing (or just report)
       tools/vendor.py --freeze name          pin what vendor/<name>/ has checked out now
"""

import argparse
import pathlib
import subprocess
import sys

import yaml

VENDOR = pathlib.Path(__file__).resolve().parent.parent / 'vendor'


def git(*args, cwd=None):
    """Run git; return its stdout, stripped, or None if it failed."""
    done = subprocess.run(['git', *args], cwd=cwd, capture_output=True, text=True)
    return done.stdout.strip() if done.returncode == 0 else None


def checkouts(root):
    """Yield every git checkout under root, as a path relative to root."""
    for dot_git in sorted(root.rglob('.git')):
        yield dot_git.parent.relative_to(root)


def freeze(name):
    """Write vendor/<name>.repos from the checkouts in vendor/<name>/."""
    root = VENDOR / name
    repos = {}
    for rel in checkouts(root):
        url = git('config', '--get', 'remote.origin.url', cwd=root / rel)
        commit = git('rev-parse', 'HEAD', cwd=root / rel)
        if url and commit:
            repos[rel.as_posix()] = {'type': 'git', 'url': url, 'version': commit}
        else:
            print(f'skipped {rel}: no origin remote, or no commit', file=sys.stderr)
    header = (f'# vendor/{name}/: {len(repos)} repositories, each at an exact commit.\n'
              f'# Made by tools/vendor.py --freeze {name}; download with pixi run get vendor.\n')
    text = yaml.safe_dump({'repositories': repos}, sort_keys=True, default_flow_style=False)
    (VENDOR / f'{name}.repos').write_text(header + text)
    print(f'vendor/{name}.repos: {len(repos)} repositories')


def fetch(name, check_only):
    """Clone what's missing from vendor/<name>.repos; return the number of problems."""
    root = VENDOR / name
    repos = yaml.safe_load((VENDOR / f'{name}.repos').read_text())['repositories']
    counts = {'pinned': 0, 'cloned': 0, 'missing': 0, 'moved': 0, 'failed': 0}
    for rel, spec in repos.items():
        dest = root / rel
        want = spec['version']
        have = git('rev-parse', 'HEAD', cwd=dest) if (dest / '.git').exists() else None
        if have == want:
            counts['pinned'] += 1
        elif have:
            counts['moved'] += 1
            print(f'  {rel}: at {have[:10]}, the list says {want[:10]} (left alone)')
        elif check_only:
            counts['missing'] += 1
            print(f'  {rel}: missing')
        elif (git('clone', '--quiet', '--filter=blob:none', spec['url'], str(dest)) is not None
              and git('checkout', '--quiet', want, cwd=dest) is not None):
            counts['cloned'] += 1
        else:
            counts['failed'] += 1
            url = spec['url']
            print(f'  {rel}: could not clone {url} at {want[:10]}')
    print(f'vendor/{name}/: ' + ', '.join(f'{n} {k}' for k, n in counts.items() if n))
    return counts['missing'] + counts['moved'] + counts['failed']


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('names', nargs='*', help='lists in vendor/ (default: all of them)')
    parser.add_argument('--check', action='store_true', help='only report; clone nothing')
    parser.add_argument('--freeze', action='store_true', help='pin what is checked out now')
    args = parser.parse_args()
    names = args.names or sorted(p.stem for p in VENDOR.glob('*.repos'))
    for name in names:
        if not args.freeze and not (VENDOR / f'{name}.repos').exists():
            parser.error(f'there is no vendor/{name}.repos')
    if args.freeze:
        if not args.names:
            parser.error('--freeze needs a name, such as ros2-src')
        for name in names:
            freeze(name)
        return 0
    problems = sum(fetch(name, args.check) for name in names)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
