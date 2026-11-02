"""
KISS-ICP itself, in the kiss environment's Python: scans in, poses out, through a pipe.

    .pixi/envs/kiss/bin/python -I worker.py      (kiss_icp_node starts it)

Reads its settings (one JSON line), then scans, from stdin; writes a pose for each to
stdout (wire.py). Exits when stdin closes. Messages go to stderr. No ROS here: the kiss
environment has only Python, NumPy and kiss-icp (MIT, PRBonn; pixi.toml).
"""

import os
import signal
import sys
import time

# -I leaves this file's own folder off the path: put it back, for wire.py beside it.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import wire  # noqa: E402


def settings_from(given):
    """Return KISS-ICP's KISSConfig from our settings (see kiss_icp_node's parameters)."""
    from kiss_icp.config import KISSConfig
    config = KISSConfig()
    config.data.max_range = float(given['max_range'])
    config.data.min_range = float(given['min_range'])
    config.data.deskew = bool(given['deskew'])
    # KISS-ICP's own rule when none is given: a hundredth of the range.
    config.mapping.voxel_size = float(given.get('voxel_size') or config.data.max_range / 100.0)
    config.mapping.max_points_per_voxel = int(given['max_points_per_voxel'])
    config.registration.max_num_threads = int(given['threads'])
    return config


def main():
    from kiss_icp.kiss_icp import KissICP
    from kiss_icp.preprocess import get_preprocessor
    # Ctrl-C reaches the whole process group: the node closes the pipe, and that ends us.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    stdin, stdout = sys.stdin.buffer, sys.stdout.buffer
    line = stdin.readline()
    if not line:
        return 0
    config = settings_from(wire.decode_settings(line))
    odometry = KissICP(config)
    # A scan without times (Gazebo's lidar) must skip the deskew: KISS-ICP 1.3 scales the
    # times itself, and equal times give it NaN (checked 30 Oct). So two preprocessors.
    deskew = odometry.preprocessor
    config.data.deskew = False
    straight = get_preprocessor(config)
    while True:
        try:
            points, times = wire.read_scan(stdin)
        except EOFError:
            return 0
        start = time.perf_counter()
        odometry.preprocessor = deskew if times is not None else straight
        odometry.register_frame(points, times if times is not None
                                else np.zeros(len(points)))
        ms = (time.perf_counter() - start) * 1000.0
        stdout.write(wire.encode_pose(odometry.last_pose, ms))
        stdout.flush()


if __name__ == '__main__':
    sys.exit(main())
