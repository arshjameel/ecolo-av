"""
The diffusion model itself, in the learn environment's Python: crops in, paths out.

    .pixi/envs/learn/bin/python -I worker.py      (diffusion_planner starts it)

Reads its settings (one JSON line: model, provider), then requests, from stdin; writes a
reply for each to stdout (wire.py). Exits when stdin closes. Messages go to stderr. No
ROS here: the learn environment has Python, NumPy, PyTorch and ONNX Runtime (pixi.toml).
"""

import os
import signal
import sys
import time

# -I leaves this file's own folder off the path: put its package's parent back, so the
# package's own modules (wire, window) import as they do in the node.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from aav_diffusion_planner import wire  # noqa: E402


def session(model, provider):
    import onnxruntime as ort
    providers = [provider] if provider == 'CPUExecutionProvider' else [
        provider, 'CPUExecutionProvider']
    opts = ort.SessionOptions()
    opts.log_severity_level = 3
    return ort.InferenceSession(model, sess_options=opts, providers=providers)


def main():
    # Ctrl-C reaches the whole process group: the node closes the pipe, and that ends us.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    stdin, stdout = sys.stdin.buffer, sys.stdout.buffer
    line = stdin.readline()
    if not line:
        return
    settings = wire.decode_settings(line)
    sess = session(settings['model'], settings.get('provider', 'CPUExecutionProvider'))
    print(f'diffusion worker: {settings["model"]} on {sess.get_providers()[0]}',
          file=sys.stderr, flush=True)
    while True:
        try:
            seq, crop, cond, noise = wire.read_request(stdin)
        except EOFError:
            return
        t0 = time.perf_counter()
        try:
            paths = sess.run(['paths'], {'crop': crop, 'cond': cond, 'noise': noise})[0]
        except Exception as e:  # noqa: B902 - any model failure means: no paths this time
            print(f'diffusion worker: the model failed: {e}', file=sys.stderr, flush=True)
            paths = None
        stdout.write(wire.encode_reply(seq, paths, 1000.0 * (time.perf_counter() - t0)))
        stdout.flush()


if __name__ == '__main__':
    main()
