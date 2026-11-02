"""
The pipe between kiss_icp_node and worker.py: the frames, both ways. No ROS.

The node runs in the default environment; KISS-ICP runs in the kiss environment's
Python (worker.py), so they talk through the worker's stdin and stdout:

  start    one line of JSON: KISS-ICP's settings (worker.settings())
  node ->  'SCAN', n (uint32), flags (uint8: 1 = times follow), then n x 3 float64
           (x, y, z in base_link), then n float64 times (0-1 across the turn) if flagged
  <- node  'POSE', 16 float64 (the 4x4 pose, row by row), 1 float64 (ms, wall clock)

Little-endian. The worker imports this file by itself (python -I worker.py), so it needs
only NumPy and the standard library.
"""

import json
import struct

import numpy as np

SCAN = struct.Struct('<4sIB')
POSE = struct.Struct('<4s17d')
HAS_TIMES = 1


def read_exact(stream, size):
    """Return exactly `size` bytes from a stream; EOFError if it ends first."""
    chunks, left = [], size
    while left:
        chunk = stream.read(left)
        if not chunk:
            raise EOFError('the pipe closed')
        chunks.append(chunk)
        left -= len(chunk)
    return b''.join(chunks)


def encode_settings(settings):
    """Return the first line the worker reads: its settings as JSON."""
    return (json.dumps(settings) + '\n').encode()


def decode_settings(line):
    """Return the settings from the worker's first line."""
    return json.loads(line.decode())


def encode_scan(points, times=None):
    """Return one scan's bytes: points N x 3 (m), times N (0-1) or None."""
    points = np.ascontiguousarray(points, dtype='<f8').reshape(-1, 3)
    out = [SCAN.pack(b'SCAN', len(points), HAS_TIMES if times is not None else 0),
           points.tobytes()]
    if times is not None:
        out.append(np.ascontiguousarray(times, dtype='<f8').reshape(-1).tobytes())
    return b''.join(out)


def read_scan(stream):
    """Return (points N x 3, times N or None) read from a stream."""
    magic, count, flags = SCAN.unpack(read_exact(stream, SCAN.size))
    if magic != b'SCAN':
        raise ValueError(f'not a scan: {magic!r}')
    # Copies: KISS-ICP's bindings refuse NumPy's read-only views of the bytes.
    points = np.frombuffer(read_exact(stream, 24 * count), dtype='<f8').reshape(count, 3).copy()
    times = (np.frombuffer(read_exact(stream, 8 * count), dtype='<f8').copy()
             if flags & HAS_TIMES else None)
    return points, times


def encode_pose(pose, ms):
    """Return the worker's answer: the 4x4 pose and the match's time (ms)."""
    return POSE.pack(b'POSE', *np.asarray(pose, dtype=np.float64).reshape(16), float(ms))


def read_pose(stream):
    """Return (the 4x4 pose, ms) read from a stream."""
    magic, *values = POSE.unpack(read_exact(stream, POSE.size))
    if magic != b'POSE':
        raise ValueError(f'not a pose: {magic!r}')
    return np.array(values[:16]).reshape(4, 4), values[16]
