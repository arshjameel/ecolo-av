#!/usr/bin/env python3
"""
How busy the car's CAN bus is: every frame in firmware/can/aav.dbc, its size and rate.

  python3 tools/can_load.py [firmware/can/aav.dbc] [--bitrate 500000] [--markdown]
  pixi run python3 tools/can_load.py

No ROS; the tests are tools/tests/test_can_load.py. The rates come from each frame's
comment in the DBC ("at 50 Hz"), the bitrate from the file's own comment ("Classic CAN at
500 kbit/s"), so a change to the contract changes the answer.

A classic CAN frame with an 11-bit ID and n data bytes is 47 + 8n bits on the wire
(start, ID, control, CRC, ACK, end of frame, the 3-bit gap), before bit stuffing. The
transmitter adds a stuff bit after 5 equal bits, so the worst case adds
floor((34 + 8n - 1) / 4) bits (Davis et al., "Controller Area Network schedulability
analysis: refuted, revisited and revised", 2007): 135 bits for 8 bytes, not 111. We count
the worst case: the bus is never busier than that.
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
import re

DBC = Path(__file__).resolve().parents[1] / 'firmware' / 'can' / 'aav.dbc'


@dataclass
class Frame:
    """One CAN frame from the DBC."""

    can_id: int
    name: str
    size: int          # data bytes, 0-8
    sender: str
    hz: float = 0.0


def frame_bits(size, stuffing=True):
    """Return the bits on the wire for a classic CAN frame with an 11-bit ID, `size` data bytes."""
    if not 0 <= size <= 8:
        raise ValueError(f'a classic CAN frame carries 0-8 bytes, not {size}')
    bits = 47 + 8 * size
    if stuffing:
        bits += (34 + 8 * size - 1) // 4
    return bits


def parse_dbc(text):
    """Return (frames, bitrate in bit/s or None) from a DBC file's text."""
    frames = {}
    for can_id, name, size, sender in re.findall(r'^BO_ (\d+) (\w+): (\d+) (\w+)', text, re.M):
        frames[int(can_id)] = Frame(int(can_id), name, int(size), sender)
    for can_id, comment in re.findall(r'^CM_ BO_ (\d+) "([^"]*)"', text, re.M):
        rate = re.search(r'at ([\d.]+) Hz', comment)
        if rate and int(can_id) in frames:
            frames[int(can_id)].hz = float(rate.group(1))
    bitrate = re.search(r'at (\d+) kbit/s', text)
    return (sorted(frames.values(), key=lambda f: f.can_id),
            int(bitrate.group(1)) * 1000 if bitrate else None)


def load(frames, bitrate):
    """Return [(frame, bits per second, share of the bus)] and the bus's total share (0-1)."""
    rows = [(f, f.hz * frame_bits(f.size), f.hz * frame_bits(f.size) / bitrate) for f in frames]
    return rows, sum(share for _, _, share in rows)


def verdict(share):
    """Return a few words on a bus load (0-1), by the usual rule of thumb."""
    if share < 0.3:
        return 'plenty of room (under 30%)'
    if share < 0.5:
        return 'fine, but watch it (30-50%)'
    if share < 0.7:
        return 'busy: low-priority frames will wait (50-70%)'
    return 'too busy: frames will be late or lost (over 70%)'


def report(frames, bitrate, markdown=False):
    """Return the load table as text."""
    rows, share = load(frames, bitrate)
    head = ('ID', 'frame', 'from', 'bytes', 'Hz', 'bits each', 'kbit/s', 'bus %')
    lines = [(f'0x{f.can_id:03X}', f.name, f.sender, str(f.size), f'{f.hz:g}',
              str(frame_bits(f.size)), f'{bps / 1000:.1f}', f'{100 * part:.1f}')
             for f, bps, part in rows]
    end = ('', 'TOTAL', '', '', f'{sum(f.hz for f in frames):g}', '',
           f'{sum(b for _, b, _ in rows) / 1000:.1f}', f'{100 * share:.1f}')
    if markdown:
        out = ['| ' + ' | '.join(head) + ' |', '|---|---|---|---:|---:|---:|---:|---:|']
        out += ['| ' + ' | '.join(line) + ' |' for line in lines]
        out.append('| ' + ' | '.join(f'**{c}**' if c else '' for c in end) + ' |')
    else:
        every = [head] + lines + [end]
        widths = [max(len(line[i]) for line in every) for i in range(len(head))]
        out = ['  '.join(c.rjust(w) for c, w in zip(line, widths)) for line in every]
        out.insert(1, '  '.join('-' * w for w in widths))
        out.insert(-1, out[1])
    out.append('')
    out.append(f'{bitrate // 1000} kbit/s bus, worst-case bit stuffing: '
               f'{100 * share:.1f}% busy - {verdict(share)}.')
    return '\n'.join(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('dbc', nargs='?', default=str(DBC))
    parser.add_argument('--bitrate', type=int, help="bit/s; default: the DBC's comment")
    parser.add_argument('--markdown', action='store_true', help='a Markdown table')
    args = parser.parse_args()
    frames, bitrate = parse_dbc(Path(args.dbc).read_text())
    bitrate = args.bitrate or bitrate
    if not bitrate:
        parser.error('no "at N kbit/s" in the DBC: give --bitrate')
    missing = [f.name for f in frames if not f.hz]
    if missing:
        print(f'No "at N Hz" in the comment of: {", ".join(missing)} (counted as 0 Hz)')
    print(report(frames, bitrate, args.markdown))


if __name__ == '__main__':
    main()
