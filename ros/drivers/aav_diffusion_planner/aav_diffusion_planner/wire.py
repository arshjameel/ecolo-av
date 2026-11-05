"""
The pipe between the planner node and its model worker. No ROS; NumPy only.

The node (the default environment's Python) and the worker (the learn environment's,
with ONNX Runtime) are two programs, as kiss_icp's are: one JSON line of settings, then
for each plan a request and a reply.

  request  header '<IIf': seq, k (samples), 0.0; crop (SIZE x SIZE float32); cond (COND
           float32); noise (k x POINTS x CHANNELS float32)
  reply    header '<IIf': seq, k (0: the model failed), ms the model took; paths (k x
           POINTS x CHANNELS float32)

seq numbers each request, so a reply that came too late for its plan is known and dropped.
"""

import json
import struct

import numpy as np

from .window import CHANNELS, COND, POINTS, SIZE

HEADER = struct.Struct('<IIf')


def encode_settings(settings):
    return (json.dumps(settings) + '\n').encode()


def decode_settings(line):
    return json.loads(line.decode())


def read_exact(stream, n):
    """Read n bytes, or raise EOFError if the stream ends first."""
    out = bytearray()
    while len(out) < n:
        chunk = stream.read(n - len(out))
        if not chunk:
            raise EOFError('the pipe closed')
        out += chunk
    return bytes(out)


def encode_request(seq, crop, cond, noise):
    noise = np.ascontiguousarray(noise, dtype=np.float32)
    return (HEADER.pack(seq, noise.shape[0], 0.0) +
            np.ascontiguousarray(crop, dtype=np.float32).reshape(SIZE * SIZE).tobytes() +
            np.ascontiguousarray(cond, dtype=np.float32).reshape(COND).tobytes() +
            noise.tobytes())


def read_request(stream):
    """Return (seq, crop (1, 1, SIZE, SIZE), cond (1, COND), noise (k, POINTS, CHANNELS))."""
    seq, k, _ = HEADER.unpack(read_exact(stream, HEADER.size))
    crop = np.frombuffer(read_exact(stream, 4 * SIZE * SIZE), np.float32)
    cond = np.frombuffer(read_exact(stream, 4 * COND), np.float32)
    noise = np.frombuffer(read_exact(stream, 4 * k * POINTS * CHANNELS), np.float32)
    return (seq, crop.reshape(1, 1, SIZE, SIZE), cond.reshape(1, COND),
            noise.reshape(k, POINTS, CHANNELS))


def encode_reply(seq, paths, ms):
    if paths is None:
        return HEADER.pack(seq, 0, ms)
    paths = np.ascontiguousarray(paths, dtype=np.float32)
    return HEADER.pack(seq, paths.shape[0], ms) + paths.tobytes()


def read_reply(stream):
    """Return (seq, paths (k, POINTS, CHANNELS) or None if the model failed, ms)."""
    seq, k, ms = HEADER.unpack(read_exact(stream, HEADER.size))
    if k == 0:
        return seq, None, ms
    paths = np.frombuffer(read_exact(stream, 4 * k * POINTS * CHANNELS), np.float32)
    return seq, paths.reshape(k, POINTS, CHANNELS), ms
