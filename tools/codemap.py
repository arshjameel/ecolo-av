#!/usr/bin/env python3
"""
Write docs/codemap.md: one line per package, grouped by area.

So you find code without searching the tree. No ROS needed; the standard library only.

For each package under ros/<area>/: what it is (package.xml's description, its first
sentence), its language, and where its logic, node, tests and config live. Then the
firmware's crates (Cargo.toml's description). Run it after adding, moving or removing a
package; tools/test.sh checks the page is current.

usage: tools/codemap.py > docs/codemap.md
       tools/codemap.py --check    (exit 1 if docs/codemap.md is out of date)
"""

from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ElementTree

ROOT = Path(__file__).resolve().parents[1]
PAGE = ROOT / 'docs' / 'codemap.md'
WIDTH = 110   # the most of a description kept

# The areas in the order a command flows (ros/README.md's picture), then the rest.
AREAS = ['sources', 'localization', 'perception', 'drivers', 'command_mux', 'ego', 'io_mux',
         'can_bridge', 'safety', 'sim', 'hud', 'bringup', 'ecolo_msgs']


def first_sentence(text):
    """Return text's first sentence, one line, cut to WIDTH."""
    text = ' '.join(text.split())
    m = re.match(r'(.+?[.:;])(\s|$)', text)
    text = m.group(1).rstrip('.:;') if m else text
    return text if len(text) <= WIDTH else text[:WIDTH - 3].rstrip() + '...'


def cargo_description(path):
    """Return a Cargo.toml's description, or ''."""
    m = re.search(r'^description\s*=\s*"(.*?)"', path.read_text(), re.M)
    return first_sentence(m.group(1)) if m else ''


def language(pkg):
    """Return the package's language: what builds it."""
    if (pkg / 'Cargo.toml').exists():
        return 'Rust'
    if (pkg / 'setup.py').exists():
        return 'Python'
    if (pkg / 'msg').exists() or (pkg / 'action').exists() or (pkg / 'srv').exists():
        return 'messages'
    return 'launch, config'


def code(paths):
    """Return paths as short code spans, joined; '-' for none."""
    return ', '.join(f'`{p}`' for p in paths) or '-'


def where(pkg):
    """Return (logic, node, tests, config): the folders and files that hold each."""
    name = pkg.name
    logic, node, tests, config = [], [], [], []
    crates = sorted(p.name for p in (pkg / 'crates').glob('*') if (p / 'Cargo.toml').exists())
    if crates:
        nodes = [c for c in crates if c.endswith('-node')]
        node = [f'crates/{c}' for c in nodes]
        logic = [f'crates/{c}' for c in crates if c not in nodes]
    elif (pkg / 'src').exists() and (pkg / 'Cargo.toml').exists():
        logic = ['src']
    module = pkg / name
    if module.is_dir():
        files = sorted(p.name for p in module.glob('*.py') if p.name != '__init__.py')
        nodes = [f for f in files
                 if re.search(r'^\s*def main\(', (module / f).read_text(), re.M)]
        node += [f'{name}/{f}' for f in nodes]
        rest = [f for f in files if f not in nodes]
        if rest:
            logic.append(f'{name}/' + (rest[0] if len(rest) == 1 else f'({len(rest)} modules)'))
    for t in ('test', 'tests'):
        if (pkg / t).is_dir():
            tests.append(f'{t}/')
    for c in ('config', 'launch'):
        if (pkg / c).is_dir():
            config.append(f'{c}/')
    return logic, node, tests, config


def ros_packages():
    """Yield (area, package dir) for every colcon package and Cargo workspace under ros/."""
    for area_dir in sorted((ROOT / 'ros').iterdir()):
        if not area_dir.is_dir():
            continue
        for pkg in sorted(area_dir.iterdir()):
            if (pkg / 'package.xml').exists() or (pkg / 'Cargo.toml').exists():
                yield area_dir.name, pkg


def describe(pkg):
    """Return a package's one-line description."""
    xml = pkg / 'package.xml'
    if xml.exists():
        found = ElementTree.parse(xml).getroot().find('description')
        if found is not None and found.text:
            return first_sentence(found.text)
    if (pkg / 'Cargo.toml').exists():
        return cargo_description(pkg / 'Cargo.toml')
    return ''


def render():
    """Return the page."""
    out = ['# The code map', '',
           "Where the code is: **read this to find code; don't search the tree.** One line",
           'per package, by area. Written by `tools/codemap.py` (`tools/test.sh` checks it is',
           'current): after adding, moving or removing a package, run',
           '`python3 tools/codemap.py > docs/codemap.md`.', '',
           '| Column | Means |', '|---|---|',
           '| Logic | the maths, no ROS: a Rust crate, or the Python modules |',
           '| Node | the ROS process: a `*-node` crate, or a Python file with `main()` |',
           '| Tests, Config | the folders, in the package |', '']
    by_area = {}
    for area, pkg in ros_packages():
        by_area.setdefault(area, []).append(pkg)
    order = [a for a in AREAS if a in by_area] + sorted(set(by_area) - set(AREAS))
    for area in order:
        out += [f'## ros/{area}', '',
                '| Package | Language | What | Logic | Node | Tests | Config |',
                '|---|---|---|---|---|---|---|']
        for pkg in by_area[area]:
            logic, node, tests, config = where(pkg)
            what = describe(pkg).replace('|', '\\|')
            out.append(f'| [{pkg.name}](../ros/{area}/{pkg.name}/README.md) | {language(pkg)} '
                       f'| {what} | {code(logic)} | {code(node)} | {code(tests)} '
                       f'| {code(config)} |')
        out.append('')
    crates = sorted(p for p in (ROOT / 'firmware' / 'spine' / 'crates').glob('*')
                    if (p / 'Cargo.toml').exists())
    out += ['## firmware/spine/crates (Rust, `no_std` where it runs on a board)', '',
            '| Crate | What |', '|---|---|']
    for c in crates:
        what = cargo_description(c / 'Cargo.toml').replace('|', '\\|')
        out.append(f'| `{c.name}` | {what} |')
    out += ['', '## Elsewhere', '', '| Folder | Holds |', '|---|---|',
            '| `firmware/can/` | `aav.dbc`, the CAN contract, and its shared test vectors |',
            '| `firmware/legacy/` | the 2025-26 ESP32 firmware, kept until the Rust one '
            'is flashed |',
            "| `firmware/status/`, `firmware/udp/` | the boards' status lines; the wheel "
            "encoder's UDP vectors |",
            '| `tools/` | scripts and checks (`pixi run ...`); their tests in `tools/tests/` |',
            '| `docs/` | the guide; `interfaces.yaml` is the topic contract |',
            '| `maps/`, `models/` | maps and trained models (most made, not kept in git) |',
            '']
    return '\n'.join(out)


def main():
    page = render()
    if '--check' in sys.argv:
        if not PAGE.exists() or PAGE.read_text() != page:
            print('docs/codemap.md is out of date: python3 tools/codemap.py > docs/codemap.md',
                  file=sys.stderr)
            return 1
        return 0
    sys.stdout.write(page)
    return 0


if __name__ == '__main__':
    sys.exit(main())
