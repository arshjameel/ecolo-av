"""
docs/features.md keeps its shape: every feature has a type, a why, a who, a date, a status.

A commit that adds or changes a feature updates its row (CLAUDE.md, Ben 25 Oct). This
can't see whether a row was forgotten, only that each row is complete.
"""

from datetime import date
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
PAGE = ROOT / 'docs' / 'features.md'
TYPES = ('Sensors', 'Localization', 'Perception', 'Drivers', 'Navigation', 'Behaviour',
         'Control', 'Safety', 'Backends', 'Firmware', 'Simulation', 'HUD', 'Tools',
         'Infrastructure')
STATUSES = ('planned', 'built', 'sim', 'car')
HEADER = ['Feature', 'Why', 'Who', 'Date', 'Status']


def sections():
    """Map each `## heading` to the table rows under it (lists of cells)."""
    out, heading = {}, None
    for line in PAGE.read_text(encoding='utf-8').splitlines():
        if line.startswith('## '):
            heading = line[3:].strip()
            out[heading] = []
        elif heading and line.startswith('|') and not re.fullmatch(r'\|[\s:|-]+\|', line):
            out[heading].append([c.strip() for c in line.strip().strip('|').split('|')])
    return out


def test_every_section_is_a_known_type():
    unknown = [h for h in sections() if h not in TYPES and h != 'Status values']
    assert not unknown, f'sections that are not a feature type {TYPES}: {unknown}'


def test_every_row_is_complete():
    for heading, rows in sections().items():
        if heading not in TYPES:
            continue
        assert rows and rows[0] == HEADER, f'{heading}: the header must be {HEADER}'
        for row in rows[1:]:
            assert len(row) == len(HEADER), f'{heading}: {len(row)} cells (a | in a cell?): {row}'
            assert all(row), f'{heading}: an empty cell: {row}'
            date.fromisoformat(row[3])
            assert row[4] in STATUSES, f'{heading}: status {row[4]!r} is not one of {STATUSES}'


def test_claude_is_never_the_who():
    # A school project: credit the person (CLAUDE.md, Git).
    who = [r[2] for t in TYPES for r in sections().get(t, [])[1:]]
    assert not [w for w in who if 'claude' in w.lower()]
