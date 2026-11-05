"""
How much each topic sends: the numbers `pixi run measure bandwidth` prints (tools/bandwidth.py).

No ROS here, so the tests run anywhere (tools/tests/test_topic_load.py).

A Meter per topic is fed each message's arrival time (s, any clock) and size (bytes, the
serialized message). After the window, summary() turns the meters into rows:

  hz         messages per second over the whole window (like ros2 topic hz, but a
             topic that sends once counts as once per window, not as nothing)
  avg, largest  message size, bytes
  mb_s       megabytes per second (1 MB = 1 000 000 bytes), = hz x avg
  mbit_s     megabits per second, = 8 x mb_s: the unit links are sold in
  worst_gap  the longest wait between two messages, s: late or bursty data shows here

table() prints the rows biggest first, with a total, as plain text or Markdown.
"""

from dataclasses import dataclass, field
import math

MB = 1e6


@dataclass
class Meter:
    """One topic's messages, as they arrive."""

    count: int = 0
    total: int = 0
    largest: int = 0
    first: float = math.nan
    last: float = math.nan
    worst_gap: float = 0.0

    def add(self, t, size):
        """Count one message of `size` bytes that arrived at time `t` (s)."""
        if self.count:
            self.worst_gap = max(self.worst_gap, t - self.last)
        else:
            self.first = t
        self.count += 1
        self.total += size
        self.largest = max(self.largest, size)
        self.last = t


@dataclass
class Row:
    """One topic's load over the window."""

    topic: str
    kind: str = ''
    hz: float = 0.0
    avg: float = 0.0
    largest: int = 0
    mb_s: float = 0.0
    worst_gap: float = math.nan
    publishers: list = field(default_factory=list)

    @property
    def mbit_s(self):
        """Return the load in megabits per second."""
        return 8.0 * self.mb_s


def summary(meters, window, types=None, publishers=None):
    """
    Return one Row per topic, biggest load first.

    meters: {topic: Meter}; window: the measuring time (s); types: {topic: type name};
    publishers: {topic: [node names]}. A topic that sent nothing gets a row of zeros.
    """
    if window <= 0:
        raise ValueError(f'the window must be longer than 0 s, not {window}')
    types = types or {}
    publishers = publishers or {}
    rows = []
    for topic, meter in meters.items():
        rows.append(Row(
            topic=topic, kind=types.get(topic, ''),
            hz=meter.count / window,
            avg=meter.total / meter.count if meter.count else 0.0,
            largest=meter.largest,
            mb_s=meter.total / window / MB,
            worst_gap=meter.worst_gap if meter.count > 1 else math.nan,
            publishers=sorted(publishers.get(topic, []))))
    rows.sort(key=lambda r: (-r.mb_s, -r.hz, r.topic))
    return rows


def size_text(n):
    """Return a byte count for people: 512 B, 3.2 kB, 691 kB, 2.7 MB."""
    for unit, scale in (('MB', 1e6), ('kB', 1e3)):
        if n >= scale:
            value = n / scale
            return f'{value:.1f} {unit}' if value < 10 else f'{value:.0f} {unit}'
    return f'{n:.0f} B'


def rate_text(x):
    """Return a rate or a load with sensible digits: 0, 0.003, 0.25, 9.8, 120."""
    if x == 0:
        return '0'
    if x < 0.001:
        return '<0.001'
    if x < 0.01:
        return f'{x:.3f}'
    if x < 10:
        return f'{x:.2f}'
    return f'{x:.0f}'


COLUMNS = ('topic', 'Hz', 'size', 'max', 'MB/s', 'Mbit/s', 'worst gap s', 'type', 'from')
NUMBERS = ('Hz', 'size', 'max', 'MB/s', 'Mbit/s', 'worst gap s')


def cells(row):
    """Return a row's printable cells, in COLUMNS order."""
    short_type = row.kind.replace('/msg/', '/')
    who = ', '.join(row.publishers[:2]) + (' ...' if len(row.publishers) > 2 else '')
    gap = '' if math.isnan(row.worst_gap) else f'{row.worst_gap:.2f}'
    return [row.topic, rate_text(row.hz), size_text(row.avg), size_text(row.largest),
            rate_text(row.mb_s), rate_text(row.mbit_s), gap, short_type, who]


def total(rows):
    """Return a Row adding up every topic's messages and load."""
    return Row(topic='TOTAL', hz=sum(r.hz for r in rows), mb_s=sum(r.mb_s for r in rows),
               largest=max((r.largest for r in rows), default=0),
               avg=(sum(r.mb_s for r in rows) * MB / sum(r.hz for r in rows)
                    if any(r.hz for r in rows) else 0.0))


def table(rows, markdown=False, quiet=False):
    """
    Return the rows as a table, with a total line.

    markdown: a Markdown table for docs/bandwidth.md; else lined-up plain text.
    quiet: leave out topics that sent nothing in the window.
    """
    shown = [r for r in rows if r.hz > 0] if quiet else list(rows)
    lines = [cells(r) for r in shown]
    end = cells(total(rows))
    end[-3:] = ['', '', '']
    end[2] = ''
    if markdown:
        out = ['| ' + ' | '.join(COLUMNS) + ' |',
               '|' + '|'.join('---:' if c in NUMBERS else '---' for c in COLUMNS) + '|']
        out += ['| ' + ' | '.join(f'`{c}`' if i == 0 else c for i, c in enumerate(line)) + ' |'
                for line in lines]
        out.append('| ' + ' | '.join(f'**{c}**' if c else '' for c in end) + ' |')
        return '\n'.join(out)
    every = [list(COLUMNS)] + lines + [end]
    widths = [max(len(line[i]) for line in every) for i in range(len(COLUMNS))]
    right = {1, 2, 3, 4, 5, 6}

    def lay(line):
        return '  '.join(c.rjust(w) if i in right else c.ljust(w)
                         for i, (c, w) in enumerate(zip(line, widths))).rstrip()
    rule = '  '.join('-' * w for w in widths)
    return '\n'.join([lay(every[0]), rule] + [lay(line) for line in lines] + [rule, lay(end)])
