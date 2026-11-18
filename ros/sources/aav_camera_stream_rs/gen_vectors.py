#!/usr/bin/env python3
"""
Write stream-core's agreement vectors from aav_camera_stream's Python modules (no ROS).

The Rust (crates/stream-core, tests/vectors.rs) must give the same answers:

  - decoders.py: check, every decoder's command line over the settings, pipelines with
    quotes and braces, shlex.split, choose (with scripted GStreamer answers), gst_works
    (with a scripted runner: the calls made and the verdict), child_env;
  - models.py: the detector's name, or the error;
  - latest.py: Latest over put/take sequences, Rate and Delays over timed events,
    stamp_ns;
  - timecode.py: crc8, bits_of, read on stamped, noisy, damaged and plain pictures,
    now_ms, age_s;
  - lens.py: the INI files in aav_sensors, camera_info YAML, scaled, identity,
    projection, the Undistorter's new K (OpenCV's getOptimalNewCameraMatrix) and its
    undistorted pictures (cv2.remap).

Pictures are small (base64 in the JSON). Floats are compared to 1e-12 (relative), not
bit for bit; undistorted pictures grey level by grey level.

  pixi run python ros/sources/aav_camera_stream_rs/gen_vectors.py
"""

import base64
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
PYTHON = HERE.parent / 'aav_camera_stream'
sys.path[:0] = [str(PYTHON)]
from aav_camera_stream import decoders, lens as lenses, models, timecode  # noqa: E402, I100
from aav_camera_stream.latest import Delays, Latest, Rate, stamp_ns  # noqa: E402

OUT = HERE / 'crates' / 'stream-core' / 'tests' / 'vectors.json'
CAMERAS = HERE.parent / 'aav_sensors' / 'config' / 'cameras'


def b64(array):
    return base64.b64encode(np.ascontiguousarray(array, dtype=np.uint8).tobytes()).decode()


def outcome(fn):
    """{'ok': value} or {'error': message}."""
    try:
        return {'ok': fn()}
    except (ValueError, KeyError) as e:
        return {'error': str(e) if isinstance(e, ValueError) else f'KeyError {e}'}


def lcg_noise(shape, seed, amplitude):
    """
    Noise both languages make: a 31-bit LCG (glibc's constants), one step per byte.

    Each byte gets (state >> 16) % (2 * amplitude + 1) - amplitude.
    """
    state = seed
    out = np.empty(int(np.prod(shape)), dtype=np.int32)
    for i in range(out.size):
        state = (1103515245 * state + 12345) & 0x7FFFFFFF
        out[i] = (state >> 16) % (2 * amplitude + 1) - amplitude
    return out.reshape(shape)


def pattern(width, height, channels=3):
    """Return a test picture both languages make: smooth ramps plus a checkerboard."""
    y, x, c = np.mgrid[0:height, 0:width, 0:channels]
    v = (x * 7 + y * 5 + c * 60) % 200 + 40 * (((x // 6) + (y // 6)) % 2)
    return v.astype(np.uint8)


def decoder_vectors():
    out = {'check': [], 'commands': [], 'shlex': [], 'choose': [], 'gst_works': [],
           'child_env': [], 'detectors': []}
    for decoder, transport in [('ffmpeg', 'tcp'), ('gst-mac', 'udp'), ('nope', 'tcp'),
                               ('opencv', 'quic'), ('auto', 'tcp'), ('ffmpeg', 'UDP')]:
        out['check'].append({'decoder': decoder, 'transport': transport,
                             **outcome(lambda: decoders.check(decoder, transport))})
    pipelines = ['', 'videotestsrc ! video/x-raw,width={width},height={height} ! fdsink',
                 "rtspsrc location=\"{url}\" protocols={protocols} ! fakesink name='a b'",
                 '{{literal}} {width}x{height} \\! fdsink', 'x {nope}', 'say "unclosed']
    for decoder in ('ffmpeg', 'gst-nvidia', 'gst-soft', 'gst-mac', 'opencv', 'auto'):
        for url, width, height in [('rtsp://192.168.1.108:8554/front_left', 1280, 720),
                                   ('rtsp://127.0.0.1:8554/rear', 640, 360)]:
            for transport in ('tcp', 'udp'):
                for program in ('', '/opt/gst/bin/gst-launch-1.0'):
                    for pipeline in pipelines:
                        for hwaccel in ('', 'videotoolbox'):
                            if hwaccel and decoder != 'ffmpeg':
                                continue
                            case = {'decoder': decoder, 'url': url, 'width': width,
                                    'height': height, 'transport': transport,
                                    'program': program, 'pipeline': pipeline,
                                    'hwaccel': hwaccel}
                            out['commands'].append({**case, **outcome(
                                lambda: decoders.command(decoder, url, width, height,
                                                         transport, program, pipeline,
                                                         hwaccel))})
    import shlex
    for text in ['a b  c', "'one two' three", 'a\\ b', '"x \\" y" z', '"\\n"', "''",
                 'end\\', "open 'quote", 'tab\there', decoders.VTDEC_TEST,
                 decoders.GST['gst-nvidia']]:
        out['shlex'].append({'text': text, **outcome(lambda: shlex.split(text))})

    for decoder in ('auto', 'gst-mac', 'gst-soft', 'ffmpeg', 'gst-nvidia'):
        for program in ('', '/x/gst-launch-1.0'):
            for platform in ('darwin', 'linux'):
                for mac_why in ('', 'vtdec could not decode a test picture'):
                    for soft_why in ('', 'no avdec_h264 in /x'):
                        asked = []

                        def works(gst, preset, mac_why=mac_why, soft_why=soft_why, asked=asked):
                            asked.append([gst, preset])
                            return mac_why if preset == 'gst-mac' else soft_why
                        c = decoders.choose(decoder, program, '', platform, works)
                        out['choose'].append({
                            'decoder': decoder, 'program': program, 'platform': platform,
                            'mac_why': mac_why, 'soft_why': soft_why, 'asked': asked,
                            'chosen': c.decoder, 'chosen_program': c.program,
                            'warning': c.warning})

    for decoder in ('gst-soft', 'gst-mac'):
        for missing in ('', 'h264parse', 'vtdec'):
            for vtdec_runs in ([True], [False, True], [False, False]):
                calls = []
                tries = list(vtdec_runs)

                def run(argv, env=None, capture_output=True, timeout=20, calls=calls,
                        missing=missing, tries=tries):
                    calls.append(list(argv))
                    if argv[1] == '--exists':
                        return SimpleNamespace(returncode=1 if argv[2] == missing else 0)
                    return SimpleNamespace(returncode=0 if tries.pop(0) else 1)
                why = decoders.gst_works('/x/bin/gst-launch-1.0', decoder, {}, run)
                out['gst_works'].append({'decoder': decoder, 'missing': missing,
                                         'vtdec_runs': vtdec_runs, 'calls': calls,
                                         'why': why})

    environ = dict.fromkeys(decoders.PIXI_ONLY, 'pixi')
    environ.update(PATH='/usr/bin', HOME='/home/x')
    for program, prefix in [('/usr/bin/gst-launch-1.0', '/ws/.pixi/envs/default'),
                            ('/ws/.pixi/envs/default/bin/ffmpeg', '/ws/.pixi/envs/default'),
                            ('/ws/.pixi/envs/default/../gst/bin/gst-launch-1.0',
                             '/ws/.pixi/envs/default'),
                            ('ffmpeg', '/ws/.pixi/envs/default'),
                            ('/usr/bin/gst-launch-1.0', ''),
                            ('/ws/.pixi/envs/defaultx/bin/a', '/ws/.pixi/envs/default/')]:
        out['child_env'].append({'environ': environ, 'program': program, 'prefix': prefix,
                                 'env': decoders.child_env(environ, program, prefix)})

    for name in ('rfdetr', ' YOLO ', 'none', 'None', 'detr'):
        kind = name.strip().lower()
        if kind in models.DETECTORS:
            out['detectors'].append({'name': name, 'ok': kind})
        else:
            out['detectors'].append({'name': name,
                                     **outcome(lambda: models.make(name, '', '', 0, print, ''))})
    return out


def latest_vectors():
    out = {'latest': [], 'rate': [], 'delays': [], 'stamp': []}
    for ops in (['put 1', 'take', 'take'], ['put 1', 'put 2', 'put 3', 'take', 'put 4', 'take'],
                ['take', 'put 9', 'put 8', 'take', 'take', 'put 7', 'take']):
        slot, results = Latest(), []
        for op in ops:
            if op == 'take':
                results.append(slot.take(0.0))
            else:
                slot.put(int(op.split()[1]))
                results.append(None)
        out['latest'].append({'ops': ops, 'results': results, 'dropped': slot.dropped,
                              'put_count': slot.put_count})
    now = [0.0]
    for window, events in [
            (5.0, [('tick', t / 15.0) for t in range(100)] + [('hz', 100 / 15.0)]),
            (5.0, [('tick', 0.0), ('hz', 0.0), ('tick', 0.0), ('hz', 0.1), ('tick', 1.0),
                   ('hz', 1.0), ('hz', 5.5), ('hz', 6.5), ('tick', 7.0), ('hz', 7.0)]),
            (2.0, [('tick', 0.1 * i * i) for i in range(20)] + [('hz', 40.0)]),
            (1.0, [(('tick' if i % 3 else 'hz'), 0.37 * i) for i in range(30)])]:
        rate, results = Rate(window, clock=lambda: now[0]), []
        for kind, t in events:
            now[0] = t
            if kind == 'tick':
                rate.tick()
                results.append(None)
            else:
                results.append(rate.hz())
        out['rate'].append({'window': window, 'events': events, 'results': results})
    for keep, values in [(5, [0.3, 0.1, 0.2]), (4, [0.4, 0.1, 0.3, 0.2]),
                         (300, [((i * 37) % 101) / 1000.0 for i in range(450)]),
                         (10, [0.05 * ((i * 7) % 13) for i in range(23)]), (3, [])]:
        d, summaries = Delays(keep=keep), []
        for v in values:
            d.add(v)
            summaries.append(d.summary())
        out['delays'].append({'keep': keep, 'values': values, 'summaries': summaries,
                              'empty': Delays(keep).summary()})
    for arrival, clock, wall, delay in [(1_700_000_000_123_456_789, 1_700_000_000_200_000_000,
                                         1_700_000_000_200_000_000, 0.06),
                                        (1_700_000_000_000_000_000, 12_000_000_000,
                                         1_700_000_000_500_000_000, 0.0335),
                                        (1_000, 500, 1_000, 0.0000025),
                                        (5, 0, 10, 0.5), (10**18, 10**18, 10**18, 1.5e-9)]:
        out['stamp'].append({'arrival': arrival, 'clock': clock, 'wall': wall,
                             'delay': delay, 'stamp': stamp_ns(arrival, clock, wall, delay)})
    return out


def timecode_vectors():
    out = {'crc8': [], 'bits': [], 'read': [], 'now_ms': [], 'age': []}
    for v in (0, 1, 255, 0x12345678, 0xFFFFFFFF, 1_234_567):
        out['crc8'].append({'value': v, 'crc': timecode.crc8(v)})
    for ms in (0, 1, 1_700_000_000_123, 2**32 + 5):
        out['bits'].append({'ms': ms, 'bits': timecode.bits_of(ms)})
    cases = []
    for width, height, channels, ms, noise, damage in [
            (320, 8, 3, 1_234_567, 0, None), (320, 8, 3, 4_000_000_000, 30, None),
            (336, 9, 1, 77, 20, None), (320, 8, 3, 99, 0, 10), (320, 8, 3, None, 0, None),
            (160, 8, 3, None, 0, None), (320, 3, 3, None, 0, None), (320, 8, 3, 5, 60, None)]:
        frame = pattern(width, height, channels)
        if channels == 1:
            frame = frame[:, :, 0].copy()
        if ms is not None:
            timecode.stamp(frame, ms)
        if noise:
            frame = np.clip(frame.astype(np.int32) + lcg_noise(frame.shape, ms or 1, noise),
                            0, 255).astype(np.uint8)
        if damage is not None:          # one data square turned over: the CRC fails
            size = timecode.square_size(width)
            x = damage * size
            frame[0:size, x:x + size] = 255 - frame[0:size, x:x + size]
        cases.append({'width': width, 'height': height, 'channels': channels, 'ms': ms,
                      'picture': b64(frame), 'read': timecode.read(frame)})
    out['read'] = cases
    for wall in (0.0, 1_700_000_000.1235, 1_700_000_000.0005, 4294967.2965, 1e9 + 0.0015):
        out['now_ms'].append({'wall': wall, 'ms': timecode.now_ms(wall)})
    for sent, wall in [(1000, 1.05), (4294967000, 0.5), (500, 0.1), (0, 2147483.648),
                       (123, 2147483.7), (4294967295, 4294967.2955)]:
        out['age'].append({'sent': sent, 'wall': wall, 'age': timecode.age_s(sent, wall)})
    return out


YAMLS = [
    """
image_width: 640
image_height: 360
camera_name: front_left
camera_matrix: {rows: 3, cols: 3, data: [343.0, 0, 319.5, 0, 343.0, 179.5, 0, 0, 1]}
distortion_model: plumb_bob
distortion_coefficients: {rows: 1, cols: 5, data: [-0.3, 0.07, 0, 0, 0]}
""",
    """
image_width: 1280
image_height: 720
camera_matrix:
  rows: 3
  cols: 3
  data: [900.5, 0.0, 641.25, 0.0, 905.0, 355.75, 0.0, 0.0, 1.0]
""",
    """
image_width: 1280
image_height: 720
camera_matrix: {data: [800, 0, 640, 0, 800, 360, 0, 0, 1]}
distortion_model: rational_polynomial
distortion_coefficients: {data: [0.1, -0.05, 0.001, -0.002, 0.01, 0.12, -0.04, 0.008]}
""",
    'image_width: 640\n',
    'camera_matrix: {data: [1, 0, 0, 0, 1, 0, 0, 0, 1]}\n',
]


def lens_vectors():
    out = {'ini': [], 'yaml': [], 'scaled': [], 'identity': [], 'new_k': [], 'undistort': []}
    for name in ('left.ini', 'right.ini', 'pinhole.ini', 'fullcalib.ini'):
        text = (CAMERAS / name).read_text()
        out['ini'].append({'name': name, 'text': text,
                           **outcome(lambda: vars(lenses.parse_ini(text)))})
    out['ini'].append({'name': 'empty', 'text': '', **outcome(lambda: vars(lenses.parse_ini('')))})
    for text in YAMLS:
        try:
            result = {'ok': vars(lenses.parse_yaml(text))}
        except ValueError as e:
            result = {'error': str(e)}
        out['yaml'].append({'text': text, **result})
    left = lenses.parse_ini((CAMERAS / 'left.ini').read_text())
    rational = lenses.parse_yaml(YAMLS[2])
    for lens, (w, h) in [(left, (1280, 720)), (left, (640, 360)), (left, (96, 54)),
                         (left, (640, 480)), (rational, (64, 36)), (left, (1920, 1080))]:
        out['scaled'].append({'lens': vars(lens), 'width': w, 'height': h,
                              **outcome(lambda: vars(lenses.scaled(lens, w, h)))})
    for w, h, f in [(640, 360, 343.0), (1280, 720, 1000.5), (7, 5, 3.0)]:
        lens = lenses.identity(w, h, f)
        out['identity'].append({'width': w, 'height': h, 'focal': f, 'lens': vars(lens),
                                'projection': lenses.projection(lens)})
    small = [lenses.scaled(left, 96, 54), lenses.scaled(rational, 64, 36),
             lenses.scaled(lenses.parse_yaml(YAMLS[0]), 80, 45)]
    big = [lenses.scaled(left, 1280, 720), lenses.scaled(rational, 1280, 720)]
    for lens in small + big:
        for alpha in (0.0, 0.5, 1.0):
            u = lenses.Undistorter(lens, alpha)
            out['new_k'].append({'lens': vars(lens), 'alpha': alpha, 'k': u.lens().k})
    for lens in small:
        for alpha in (0.0, 1.0):
            picture = pattern(lens.width, lens.height)
            undistorted = lenses.Undistorter(lens, alpha).apply(picture)
            out['undistort'].append({'lens': vars(lens), 'alpha': alpha,
                                     'picture': b64(undistorted)})
    return out


def main():
    vectors = {'decoders': decoder_vectors(), 'latest': latest_vectors(),
               'timecode': timecode_vectors(), 'lens': lens_vectors(),
               'opencv': cv2.__version__}
    OUT.write_text(json.dumps(vectors, indent=0, sort_keys=True) + '\n')
    counts = {k: sum(len(v) for v in vectors[k].values()) for k in vectors if k != 'opencv'}
    print(f'{OUT}: {counts} (OpenCV {cv2.__version__})')


if __name__ == '__main__':
    main()
