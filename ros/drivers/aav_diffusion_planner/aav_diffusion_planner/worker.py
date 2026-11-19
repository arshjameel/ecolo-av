"""
The diffusion model itself, in the learn environment's Python: crops in, paths out.

    .pixi/envs/learn/bin/python -I worker.py      (diffusion_planner starts it)

Reads its settings (one JSON line: model, provider: engines.py's, which falls back to
the CPU if the engine can't load the model), then requests, from stdin; writes a
reply for each to stdout (wire.py). Exits when stdin closes. Messages go to stderr. No
ROS here: the learn environment has Python, NumPy, PyTorch and ONNX Runtime (pixi.toml).
"""

import functools
import os
import signal
import sys
import time

# -I leaves this file's own folder off the path: put its package's parent back, so the
# package's own modules (wire, window) import as they do in the node.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from aav_diffusion_planner import engines, wire  # noqa: E402


def session(model, provider, cache_dir=None):
    """Return an ONNX Runtime session on `provider`, falling back down engines.CHAINS."""
    import onnxruntime as ort
    chain = engines.usable(engines.engines(provider, cache_dir),
                           ort.get_available_providers())
    if not chain:
        chain = engines.engines(engines.CPU)
    opts = ort.SessionOptions()
    opts.log_severity_level = 3
    return ort.InferenceSession(model, sess_options=opts,
                                providers=[name for name, _ in chain],
                                provider_options=[options for _, options in chain])


def run(sess, crop, cond, noise):
    """Return the model's paths for one crop, one cond and a batch of noise."""
    return sess.run(['paths'], {'crop': crop, 'cond': cond, 'noise': noise})[0]


def main():
    # Ctrl-C reaches the whole process group: the node closes the pipe, and that ends us.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    stdin, stdout = sys.stdin.buffer, sys.stdout.buffer
    line = stdin.readline()
    if not line:
        return
    settings = wire.decode_settings(line)
    # TensorRT's engines, built on the first start (minutes on the Orin), kept beside the
    # model: `pixi run measure diffusion` builds them first, in the same folder.
    cache = os.path.join(os.path.dirname(os.path.abspath(settings['model'])), 'engines')
    os.makedirs(cache, exist_ok=True)
    sess = session(settings['model'], settings.get('provider', engines.CPU), cache)
    print(f'diffusion worker: {settings["model"]} on {sess.get_providers()[0]}',
          file=sys.stderr, flush=True)
    while True:
        try:
            seq, crop, cond, noise = wire.read_request(stdin)
        except EOFError:
            return
        t0 = time.perf_counter()
        try:
            # A fixed-size export (19 Nov on) runs a node's other sample counts in batches.
            k = next(i.shape[0] for i in sess.get_inputs() if i.name == 'noise')
            paths = engines.batches(functools.partial(run, sess, crop, cond), noise,
                                    k if isinstance(k, int) else None)
        except Exception as e:  # noqa: B902 - any model failure means: no paths this time
            print(f'diffusion worker: the model failed: {e}', file=sys.stderr, flush=True)
            paths = None
            if sess.get_providers()[0] != engines.CPU:
                # An engine that loaded the model but can't run it (8 Nov: Core ML on the
                # 0.3 export): the CPU from now on. This plan is the lattice's.
                sess = session(settings['model'], engines.CPU)
                print('diffusion worker: on the CPU from now on', file=sys.stderr, flush=True)
        stdout.write(wire.encode_reply(seq, paths, 1000.0 * (time.perf_counter() - t0)))
        stdout.flush()


if __name__ == '__main__':
    main()
