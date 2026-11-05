"""
stream_check: the camera stream end to end on one computer, without the car.

  Gazebo's front camera ──ROS──> this check: scaled to 1280x720, time code drawn on
      ──raw──> ffmpeg (H.264, the Pi's settings) ──RTSP──> a local MediaMTX
      ──RTSP──> camera_stream_front_left (decode, undistort, RF-DETR) ──> detections
      ──> the sensor watchdog (aav_safety), watching the camera as on the car

  pixi run -e ml measure stream            # RF-DETR Nano, as on the car (needs PyTorch)
  pixi run measure stream --detector none  # the stream alone, no PyTorch

Measures, over the last --seconds (60): the delay from the time code being drawn to the
picture leaving the decoder, and to its detections being published (median, 95th
percentile, worst); the bitrate on the wire (MediaMTX's own count); the size of the
detections and the previews; the frame rates and the frames dropped; each program's CPU.
And the sensor watchdog's front_left_stream input, as car.launch.py starts it: OK all
through the measurement, then STALE soon after the encoder stops (the stream lost).
Prints a table and exits 1 if a check fails.

Options:
  --seconds N        how long to measure (60)
  --encoder E        openh264 (the default: what MediaMTX's rpiCamera runs on a Pi 5) or
                     x264 (ultrafast, zerolatency: what ffmpeg runs for a USB camera)
  --detector D       rfdetr (the default), yolo or none
  --decoder D        the node's decoder: auto (the default, as the node's: GStreamer from
                     the gst environment, with VideoToolbox, vtdec, on a Mac; this check
                     installs it once, pixi install --locked -e gst), ffmpeg (to compare:
                     it holds back about 2 frames, 135 ms), gst-mac, gst-soft...;
                     --hwaccel H, ffmpeg's (videotoolbox); --program P, the decoder program
  --gst              gst-soft: GStreamer's software decoder (avdec_h264), to compare
  --pattern          no Gazebo: aav_detector's stop-sign photo, moving, with noise (the
                     noise makes the encoder work as on a real camera)
  --sign-at X        put Gazebo's stop sign X m ahead of the car (25)
  --drive V          roll the car towards it at V m/s while measuring (0.3), so the
                     picture changes as on the road and the sign comes into range

It runs on this computer only: a private ROS domain and Gazebo partition, MediaMTX and
RTSP on 127.0.0.1 with ports picked free, every program in its own process group
(stopped by group, never by name). Needs vendor/mediamtx: pixi run get mediamtx.
What it can't measure: the camera's own capture and the Pi 5's encoder (its CPU is not
this computer's), the wired network between the Pi and the Orin, and the Orin's decoder.
"""

import json
import os
import platform
import random
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOMAIN = os.environ.get('AAV_STREAM_DOMAIN', str(random.randint(150, 230)))
os.environ['ROS_DOMAIN_ID'] = DOMAIN
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['GZ_IP'] = '127.0.0.1'
os.environ['GZ_PARTITION'] = os.environ.get('AAV_STREAM_PARTITION', f'stream_check_{DOMAIN}')

from aav_camera_stream import timecode  # noqa: E402
from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
import cv2  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import numpy  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from sensor_msgs.msg import CompressedImage, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vision_msgs.msg import Detection2DArray  # noqa: E402
import yaml  # noqa: E402

MEDIAMTX = os.path.join(ROOT, 'vendor', 'mediamtx', 'mediamtx')
PI_CONFIG = os.path.join(ROOT, 'ros', 'sources', 'aav_camera_stream', 'pi', 'mediamtx.yml')
PHOTO = os.path.join(ROOT, 'ros', 'perception', 'aav_detector', 'test', 'stop_sign.jpg')
WIDTH, HEIGHT, FPS = 1280, 720, 15              # the Pi's stream (pi/mediamtx.yml)
BITRATE = 2_000_000                             # bit/s
GZ_FOCAL = 343.0                                # Gazebo's camera at 640x360 (sensors xacro)
CAMERA = 'front_left'
LATEST = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.RELIABLE)
RESULTS = []
LOG = os.environ.get('AAV_STREAM_LOG', '/tmp/aav-stream-check')
WATCHED = f'{CAMERA}_stream'                    # the sensor watchdog's input for it


def result(name, ok, detail):
    RESULTS.append((name, ok, detail))
    print(f'{"ok  " if ok else "info" if ok is None else "FAIL"} {name}: {detail}', flush=True)


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def encoder_args(encoder):
    """Return the ffmpeg encoder's options, as the camera Pi sets it (pi/mediamtx.yml)."""
    if encoder == 'openh264':
        # MediaMTX's rpiCamera software encoder is OpenH264: real-time, bitrate mode,
        # max 1.2 x the target, baseline (no B-frames), an IDR every rpiCameraIDRPeriod.
        return ['-c:v', 'libopenh264', '-b:v', str(BITRATE), '-maxrate', str(int(BITRATE * 1.2)),
                '-g', str(FPS), '-rc_mode', 'bitrate', '-allow_skip_frames', '0']
    if encoder == 'x264':
        return ['-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency', '-bf', '0',
                '-g', str(FPS), '-b:v', str(BITRATE), '-maxrate', str(BITRATE),
                '-bufsize', str(BITRATE // 2), '-pix_fmt', 'yuv420p']
    raise SystemExit(f'--encoder {encoder}: use openh264 or x264')


def mediamtx_config(rtsp_port, api_port):
    """Return the Pi's settings, the cameras swapped for a publisher, on 127.0.0.1."""
    with open(PI_CONFIG) as f:
        conf = yaml.safe_load(f)
    conf.pop('pathDefaults', None)            # the rpiCamera settings: no camera here
    conf.update(rtspAddress=f'127.0.0.1:{rtsp_port}', rtspTransports=['tcp'],
                apiAddress=f'127.0.0.1:{api_port}', webrtc=False, logLevel='warn',
                paths={CAMERA: {'source': 'publisher'}})
    return conf


def ps_seconds(text):
    """Return ps's cputime, [dd-][hh:]mm:ss.ss, in s."""
    days, _, clock = text.rpartition('-')
    parts = [float(p) for p in clock.split(':')]
    return sum(p * 60 ** i for i, p in enumerate(reversed(parts))) + 86400 * int(days or 0)


def cpu_by_group(groups):
    """Return the CPU time so far (user + system, s) of each process group."""
    out = subprocess.run(['ps', '-axo', 'pgid=,time='], capture_output=True,
                         text=True).stdout
    totals = dict.fromkeys(groups, 0.0)
    for line in out.splitlines():
        fields = line.split()
        if len(fields) == 2 and int(fields[0]) in totals:
            totals[int(fields[0])] += ps_seconds(fields[1])
    return totals


def load_average():
    return ', '.join(f'{x:.1f}' for x in os.getloadavg())


class Programs:
    """The programs this check starts, each in its own process group."""

    def __init__(self):
        self.procs = {}

    def start(self, name, argv, log, **kwargs):
        out = open(f'{LOG}-{name}.log', 'w') if log else subprocess.DEVNULL
        self.procs[name] = subprocess.Popen(argv, stdout=out, stderr=subprocess.STDOUT,
                                            start_new_session=True, cwd=ROOT, **kwargs)
        return self.procs[name]

    def stop_all(self):
        for name, proc in reversed(list(self.procs.items())):
            if proc.poll() is not None:
                continue
            for sig, wait in ((signal.SIGINT, 15), (signal.SIGKILL, 5)):
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    proc.wait(timeout=wait)
                    break
                except subprocess.TimeoutExpired:
                    continue


class Check(Node):
    """Feeds the encoder 15 pictures a second, and listens to what comes out."""

    def __init__(self, encoder_stdin, pattern):
        super().__init__('stream_check')
        self.stdin = encoder_stdin
        self.pattern = pattern
        self.newest = None
        self.gz_frames = 0
        self.fed = 0
        self.lock = threading.Lock()
        self.detections = []        # (size in bytes, labels)
        self.previews = []          # (time, size in bytes)
        self.status = None
        self.x = None               # the car's x in Gazebo's world (its truth), m
        self.sign_seen_at = []      # the car's x at each picture where the sign was found
        self.speed = 0.0
        if not pattern:
            self.create_subscription(Image, f'/sensors/camera/{CAMERA}/image_raw',
                                     self.on_image, LATEST)
            self.create_subscription(Odometry, '/vehicle/gazebo/truth', self.on_truth, 10)
            # Straight to Gazebo's car: no ego or mux here (a private domain, a stand-in).
            self.drive = self.create_publisher(AckermannDriveStamped,
                                               '/control/gazebo/actuator_cmd', 10)
            self.create_timer(0.1, self.send_drive)
        self.create_subscription(Detection2DArray, '/perception/detections', self.on_det, 10)
        self.create_subscription(CompressedImage,
                                 f'/perception/camera/{CAMERA}/preview/compressed',
                                 self.on_preview, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 10)
        self.watchdog = []          # (time, the sensor watchdog's state line)
        self.create_subscription(String, '/safety/sensor_watchdog/state',
                                 lambda m: self.watchdog.append((time.monotonic(), m.data)), 10)
        self.photo = None
        if pattern:
            photo = cv2.imread(PHOTO)
            self.photo = cv2.resize(photo, (WIDTH + 200, HEIGHT), interpolation=cv2.INTER_AREA)
        self.stopping = threading.Event()
        self.feeder = threading.Thread(target=self.feed, daemon=True)

    def on_image(self, msg):
        frame = numpy.frombuffer(msg.data, numpy.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.newest = frame
            self.gz_frames += 1

    def picture(self, i):
        if self.pattern:
            # The photo sliding left and right, with sensor-like noise.
            x = int(100 + 100 * numpy.sin(i / 15.0))
            frame = self.photo[:, x:x + WIDTH].copy()
            noise = numpy.random.default_rng(i).integers(-12, 13, frame.shape, dtype=numpy.int16)
            return numpy.clip(frame.astype(numpy.int16) + noise, 0, 255).astype(numpy.uint8)
        with self.lock:
            newest = self.newest
        if newest is None:
            return None
        return cv2.resize(newest, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)

    def feed(self):
        """15 Hz, like the camera: the newest Gazebo picture, its time code drawn last."""
        period = 1.0 / FPS
        next_t = time.monotonic()
        i = 0
        while not self.stopping.is_set():
            frame = self.picture(i)
            if frame is not None:
                timecode.stamp(frame, timecode.now_ms(time.time()))
                try:
                    self.stdin.write(frame.tobytes())
                    self.stdin.flush()
                except (BrokenPipeError, ValueError):
                    return
                self.fed += 1
            i += 1
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))

    def on_truth(self, msg):
        self.x = msg.pose.pose.position.x

    def send_drive(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(self.speed)
        self.drive.publish(msg)

    def on_det(self, msg):
        labels = [d.results[0].hypothesis.class_id for d in msg.detections if d.results]
        self.detections.append((len(serialize_message(msg)), labels))
        if 'stop sign' in labels and self.x is not None:
            self.sign_seen_at.append(self.x)

    def on_preview(self, msg):
        self.previews.append((time.monotonic(), len(serialize_message(msg))))

    def on_diag(self, msg):
        for status in msg.status:
            if status.name == f'camera_stream: {CAMERA}':
                self.status = {kv.key: kv.value for kv in status.values}
                self.status['level'] = status.level

    def spin_for(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if until is not None and until():
                return True
        return until is None


def api_bytes(api_port):
    with urllib.request.urlopen(f'http://127.0.0.1:{api_port}/v3/paths/get/{CAMERA}',
                                timeout=5) as r:
        info = json.load(r)
    return info.get('bytesReceived', 0), info.get('bytesSent', 0), info.get('tracks2', [])


def move_sign(ahead):
    """Put the road world's stop sign `ahead` m in front of the car, just right of it."""
    req = (f'name: "stop_sign", position: {{x: {ahead}, y: -1.5, z: 0}}, '
           'orientation: {x: 0, y: 0, z: 1, w: 0}')
    done = subprocess.run(['gz', 'service', '-s', '/world/aav/set_pose', '--reqtype',
                           'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
                           '--req', req], capture_output=True, text=True, timeout=30)
    return done.returncode == 0 and 'data: true' in done.stdout


def gst_environment():
    """Return the gst environment's gst-launch-1.0 (pixi.toml), installing it once."""
    from aav_camera_stream import decoders
    if not decoders.pixi_gst(ROOT):
        print('GStreamer, once: pixi install --locked -e gst', flush=True)
        subprocess.run(['pixi', 'install', '--locked', '-e', 'gst'], cwd=ROOT, timeout=900)
    path = decoders.pixi_gst(ROOT)
    if not path:
        raise SystemExit('No GStreamer: pixi install --locked -e gst failed (above)')
    return path


def option(args, name, default):
    return args[args.index(name) + 1] if name in args else default


def main():
    args = sys.argv[1:]
    if '-h' in args or '--help' in args:
        print(__doc__)
        return 0
    seconds = float(option(args, '--seconds', '60'))
    encoder = option(args, '--encoder', 'openh264')
    detector = option(args, '--detector', 'rfdetr')
    decoder = option(args, '--decoder', 'auto')
    hwaccel = option(args, '--hwaccel', '')
    program = option(args, '--program', '')
    if '--gst' in args:
        decoder = 'gst-soft'
    if decoder in ('auto', 'gst-mac', 'gst-soft') and not program and sys.platform in (
            'darwin', 'linux') and platform.machine() in ('arm64', 'x86_64'):
        program = gst_environment()     # the node finds it there too; checked here first
    sign_at = float(option(args, '--sign-at', '25'))
    speed = float(option(args, '--drive', '0.3'))
    pattern = '--pattern' in args
    if not os.access(MEDIAMTX, os.X_OK):
        print('No MediaMTX here: pixi run get mediamtx (once)', file=sys.stderr)
        return 2
    if shutil.which('ffmpeg') is None:
        print('No ffmpeg: run this through pixi (pixi run measure stream)', file=sys.stderr)
        return 2
    if detector != 'none':
        try:
            import torch  # noqa: F401
        except ImportError:
            print(f'--detector {detector} needs PyTorch: pixi run -e ml measure stream, or '
                  '--detector none', file=sys.stderr)
            return 2

    rtsp_port, api_port = free_port(), free_port()
    url = f'rtsp://127.0.0.1:{rtsp_port}/{CAMERA}'
    tmp = tempfile.mkdtemp(prefix='aav-stream-check-')
    conf = os.path.join(tmp, 'mediamtx.yml')
    with open(conf, 'w') as f:
        yaml.safe_dump(mediamtx_config(rtsp_port, api_port), f)
    # Gazebo's lens, scaled to the stream: no distortion, so the undistortion runs (its
    # cost is measured) and changes nothing.
    calibration = os.path.join(tmp, 'gazebo_front_left.yaml')
    with open(calibration, 'w') as f:
        yaml.safe_dump({'image_width': 640, 'image_height': 360,
                        'camera_matrix': {'rows': 3, 'cols': 3,
                                          'data': [GZ_FOCAL, 0, 319.5, 0, GZ_FOCAL, 179.5,
                                                   0, 0, 1]},
                        'distortion_model': 'plumb_bob',
                        'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': [0] * 5}}, f)
    print(f'ROS domain {DOMAIN}, Gazebo partition {os.environ["GZ_PARTITION"]}, RTSP {url}; '
          f'this computer only. Load average: {load_average()}', flush=True)
    load_before = os.getloadavg()

    programs = Programs()
    rclpy.init()
    check = None
    try:
        programs.start('mediamtx', [MEDIAMTX, conf], log=True)
        if not pattern:
            programs.start('gazebo', ['ros2', 'launch', 'aav_gazebo', 'gazebo.launch.py',
                                      'world:=road', 'gui:=false', 'lidar:=false',
                                      'radar:=false'], log=True)
        time.sleep(1.0)
        enc = programs.start(
            'encoder', ['ffmpeg', '-hide_banner', '-loglevel', 'warning', '-f', 'rawvideo',
                        '-pix_fmt', 'bgr24', '-s', f'{WIDTH}x{HEIGHT}', '-framerate', str(FPS),
                        '-i', '-', *encoder_args(encoder), '-an', '-f', 'rtsp',
                        '-rtsp_transport', 'tcp', url], log=True, stdin=subprocess.PIPE)
        check = Check(enc.stdin, pattern)
        if not pattern:
            got = check.spin_for(120.0, lambda: check.gz_frames > 0)
            result('Gazebo camera', got, 'its pictures are coming' if got else
                   'no picture from Gazebo in 120 s (see ' + LOG + '-gazebo.log)')
            if not got:
                return 1
            result('stop sign moved', move_sign(sign_at) or None,
                   f'{sign_at:g} m ahead of the car, 1.5 m to its right')
        check.feeder.start()
        programs.start(
            'camera_stream',
            [sys.executable, '-m', 'aav_camera_stream.camera_stream', '--ros-args',
             '-r', f'__node:=camera_stream_{CAMERA}', '-p', f'camera:={CAMERA}',
             '-p', f'url:={url}', '-p', f'decoder:={decoder}', '-p', f'hwaccel:="{hwaccel}"',
             '-p', f'program:="{program}"',
             '-p', f'detector:={detector}', '-p', f'calibration:={calibration}',
             '-p', 'publish_camera_info:=false', '-p', 'time_code:=true',
             '-p', 'source:=sim'], log=True)
        # The sensor watchdog, watching this camera through its detections (car.launch.py).
        programs.start('watchdog', ['ros2', 'launch', 'aav_safety', 'watchdog.launch.py',
                                    f'inputs:={WATCHED}'], log=True)
        # Up: pictures through and, with a model, its first detections (loading takes time).
        up = check.spin_for(180.0, lambda: check.status is not None
                            and check.status.get('connected') == 'true'
                            and int(check.status.get('time_codes', '0')) > 0)
        result('stream up', up, 'camera_stream decodes the stream and reads the time code'
               if up else f'nothing through in 180 s (see {LOG}-camera_stream.log)')
        if not up:
            return 1
        # auto on a Mac must find VideoToolbox; elsewhere, what it picked is only reported.
        chosen = check.status.get('decoder', '?')
        mac_auto = decoder == 'auto' and sys.platform == 'darwin'
        result('decoder', chosen == 'gst-mac' if mac_auto else None,
               f'{chosen} (asked for {decoder})' + (
                   f'; a Mac should get gst-mac: see {LOG}-camera_stream.log'
                   if mac_auto and chosen != 'gst-mac' else ''))
        check.spin_for(5.0)                       # settle: the first frames are slow
        rx0, tx0, tracks = api_bytes(api_port)
        groups = {name: p.pid for name, p in programs.procs.items()}
        cpu0 = cpu_by_group(groups.values())
        det0, prev0, fed0, gz0, dog0 = (len(check.detections), len(check.previews),
                                        check.fed, check.gz_frames, len(check.watchdog))
        x0 = check.x
        t0 = time.monotonic()
        if not pattern and speed > 0:
            check.speed = speed     # the car rolls towards the sign: the picture changes
        print(f'Measuring for {seconds:g} s...', flush=True)
        check.spin_for(seconds)
        span = time.monotonic() - t0
        check.speed = 0.0
        check.spin_for(0.3)
        rx1, tx1, tracks = api_bytes(api_port)
        cpu1 = cpu_by_group(groups.values())
        status = check.status or {}
        dets = check.detections[det0:]
        previews = check.previews[prev0:]
        states = [text for _, text in check.watchdog[dog0:]]
        wrong = [text for text in states if WATCHED in text]
        result('watchdog', bool(states) and not wrong,
               f'{WATCHED} OK in all {len(states)} of its states' if states and not wrong
               else f'{len(wrong)} of {len(states)} states name {WATCHED}: '
                    f'{wrong[0] if wrong else "none heard"} (see {LOG}-watchdog.log)')
        # The stream lost: the encoder (the Pi, here) stops. The detections stop with it.
        lost = time.monotonic()
        os.killpg(programs.procs['encoder'].pid, signal.SIGKILL)

        def stale_at():
            return next((t for t, text in check.watchdog
                         if t > lost and f'{WATCHED} STALE' in text), None)

        stale = check.spin_for(10.0, lambda: stale_at() is not None)
        result('watchdog, stream lost', stale,
               f'{WATCHED} STALE {stale_at() - lost:.1f} s after the encoder stopped (its '
               'rule: silent 1 s)' if stale else 'not STALE 10 s after the encoder stopped')

        mbit = 8 * (rx1 - rx0) / span / 1e6
        props = tracks[0].get('codecProps', {}) if tracks else {}
        result('on the wire', mbit < 1.2 * BITRATE / 1e6 * 1.25,
               f'{mbit:.2f} Mbit/s ({(rx1 - rx0) / span / 1e3:.0f} kB/s) into MediaMTX, '
               f'{8 * (tx1 - tx0) / span / 1e6:.2f} Mbit/s out to the node; '
               f'{props.get("width")}x{props.get("height")} H.264 {props.get("profile")}, '
               f'{encoder}')
        fed = (check.fed - fed0) / span
        if not pattern:
            moved = (check.x - x0) if check.x is not None and x0 is not None else float('nan')
            result('Gazebo', None, f'{(check.gz_frames - gz0) / span:.1f} pictures a second '
                   f'from its camera (640x360; scaled up to {WIDTH}x{HEIGHT}); the car rolled '
                   f'{moved:.1f} m at {speed:g} m/s')
        fps_out = float(status.get('fps_out', 0))
        result('frame rate', fps_out >= (0.5 * FPS if detector == 'none' else 3),
               f'{fed:.1f} fps sent, {status.get("fps_in")} decoded, {fps_out:.1f} worked '
               f'on; {status.get("dropped")} dropped in all (only the newest is worked on); '
               f'detector {status.get("detect_ms", "?")} ms, all the work '
               f'{status.get("work_ms", "?")} ms a picture (medians)')
        for name, label in (('arrival', 'time code -> out of the decoder'),
                            ('detection', 'time code -> detections published')):
            med = status.get(f'{name}_median_ms')
            ok = med is not None and float(status.get(f'{name}_p95_ms', 1e9)) < 1000.0
            result(f'delay, {name}', ok,
                   f'{label}: median {med} ms, p95 {status.get(f"{name}_p95_ms")} ms, worst '
                   f'{status.get(f"{name}_max_ms")} ms (last minute, '
                   f'{status.get("time_codes")} codes read)')
        if dets:
            sizes = [s for s, _ in dets]
            labels = sorted({label for _, ls in dets for label in ls})
            seen = sum(1 for _, ls in dets if 'stop sign' in ls)
            result('detections', len(dets) > 0,
                   f'{len(dets) / span:.1f} messages/s, {min(sizes)}-{max(sizes)} bytes '
                   f'(mean {sum(sizes) / len(sizes):.0f}); labels: {", ".join(labels) or "none"}')
            if detector != 'none':
                where = ''
                if check.sign_seen_at and not pattern:
                    far = sign_at - min(check.sign_seen_at)
                    near = sign_at - max(check.sign_seen_at)
                    where = f'; from {far:.1f} m to {near:.1f} m (the car to the sign)'
                result('stop sign seen', seen > 0,
                       f'in {seen} of {len(dets)} pictures ({100 * seen / len(dets):.0f}%)'
                       f'{where}')
        else:
            result('detections', False, 'no /perception/detections in the measurement')
        if previews:
            sizes = [s for _, s in previews]
            result('previews', True, f'{len(previews) / span:.2f} a second, '
                   f'{sum(sizes) / len(sizes) / 1e3:.1f} kB each (JPEG, 320 px wide)')
        else:
            result('previews', False, 'none in the measurement')
        cpu = [f'{name} {100 * (cpu1[pid] - cpu0[pid]) / span:.0f}%'
               for name, pid in groups.items()]
        result('CPU', None, ', '.join(cpu) + ' (each with its children: camera_stream with '
               'its decoder, gazebo with gz sim; 100% = one core of this computer); load '
               f'average {", ".join(f"{x:.1f}" for x in load_before)} before, '
               f'{load_average()} at the end')
    finally:
        if check is not None:
            check.stopping.set()
            check.destroy_node()
        rclpy.try_shutdown()
        programs.stop_all()
        shutil.rmtree(tmp, ignore_errors=True)

    print('\n| Check | Result | Detail |\n|---|---|---|')
    for name, ok, detail in RESULTS:
        print(f'| {name} | {"ℹ️" if ok is None else "✅" if ok else "❌"} | {detail} |')
    print(f'\nLogs: {LOG}-*.log')
    return 0 if all(ok is not False for _, ok, _ in RESULTS) else 1


if __name__ == '__main__':
    sys.exit(main())
