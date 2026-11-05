#!/usr/bin/env python3
"""
Check our Foxglove/Lichtblick layouts against a live foxglove_bridge: each topic, served or not.

    python3 tools/layout_check.py                          # ws://localhost:8765 (the sim)
    python3 tools/layout_check.py ws://192.168.1.140:8765  # the Jetson
    python3 tools/layout_check.py --data                   # and what each topic sends

It connects as Lichtblick does (a WebSocket, subprotocol foxglove.sdk.v1, or the older
foxglove.websocket.v1), collects the topics the bridge advertises for a few seconds, and
lists each of the layouts' topics (tools/foxglove/*.json) as served or missing. Some come
only from parts that are off by default (OPTIONAL): those are missing without being a
fault.

--data subscribes to every served topic the layouts show, as a laptop that opens them all
would, and counts what arrives: messages a second and kB a second for each, and the
total, the load that laptop puts on the car's WiFi (docs/bandwidth.md). A topic that
sends nothing in that time is listed as silent: a fault if it isn't optional.

No ROS and no extra Python packages: it runs on any Mac. Without --data it only reads the
topic list: the bridge sends it nothing else. Exit status: 0 if every topic that isn't
optional is served (and, with --data, sends something).
"""

import argparse
import base64
import json
import os
from pathlib import Path
import re
import socket
import struct
import sys
import time
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]
LAYOUTS = sorted((ROOT / 'tools' / 'foxglove').glob('*.json'))
PROTOCOLS = ('foxglove.sdk.v1', 'foxglove.websocket.v1')   # newest first
# The layouts' topics that come from parts that are off unless asked for, or that only
# publish once something happens.
OPTIONAL = {
    '/safety/ghost/marker': 'ghost:=true (aav_ghost)',
    '/drivers/nav/route/path': 'the street router, once it has a route',
    '/drivers/nav/route/status': 'the street router, once it has a route',
    '/goal_pose': 'a goal, sent from RViz, the map window or Lichtblick',
    '/perception/camera_obstacles': 'camera_obstacles:=true',
    '/perception/radar/cluster_boxes': 'a radar, and something it sees',
    '/sensors/radar/points': 'a radar (the road and test_lot worlds, the car later)',
    '/sensors/lidar/scan': 'the lidar part (the sim: worlds with sensors)',
    '/perception/camera/front_left/preview/compressed':
        "a camera: camera_stream on the car, foxglove.launch.py's preview elsewhere",
    '/safety/collision_monitor/stop_zone': 'a radar or lidar (collision_monitor)',
    '/perception/voxels/cubes': 'voxels:=true (the voxels trial, with the lidar)',
    '/vehicle/gazebo/chase_camera/compressed':
        "Gazebo's chase camera (a world that renders sensors; sim only)",
    '/localization/ndt/pose': 'ndt:=true (the NDT trial: world:=test_lot, its map made)',
    '/localization/ndt/status': 'ndt:=true (the NDT trial)',
    '/localization/amcl/pose': 'amcl:=true (the AMCL trial), once the car has moved',
    '/localization/kiss_icp/odom': 'kiss_icp:=true (the KISS-ICP trial, with the lidar)',
    '/localization/kiss_icp/status': 'kiss_icp:=true (the KISS-ICP trial)',
}
# The services the layouts call, and the parts that serve them only when asked for.
OPTIONAL_SERVICES = {
    '/track/set_parameters': 'a trial tracker at launch (tracker:=lqr...), sim only',
}
# Topics a layout's own user scripts make, inside the app: never asked of the bridge.
SCRIPT_PREFIX = '/studio_script/'
# Nav2's own topics the layouts show, outside docs/interfaces.yaml (Nav2 names them).
NAV2_OWN = {'/local_costmap/costmap'}
# What a 3D panel reads besides its topics: the frames, to place each one.
FRAMES = {'/tf', '/tf_static'}
# Topics that send only while the car drives to a goal: silent when parked is no fault.
WHILE_DRIVING = {'/control/cmd_vel', '/plan'}


def script_inputs(layout):
    """Return the topics a layout's user scripts read (each script's `inputs`)."""
    inputs = set()
    for script in layout.get('userNodes', {}).values():
        found = re.search(r'export const inputs = \[([^\]]*)\]', script['sourceCode'])
        inputs |= set(re.findall(r'"([^"]+)"', found.group(1))) if found else set()
    return inputs


def shown_services(layout):
    """Return the set of services a layout's Call Service panels call."""
    return {config['serviceName'] for panel, config in layout['configById'].items()
            if panel.split('!')[0] == 'CallService'}


def shown_topics(layout):
    """
    Return the set of topics a layout's panels read, from each panel's own setting.

    A user script's output (/studio_script/...) is the app's own: its inputs count instead.
    """
    def topic(path):
        return path.split('.')[0]

    shown = script_inputs(layout)
    for panel, config in layout['configById'].items():
        kind = panel.split('!')[0]
        if kind == '3D':
            shown |= set(config['topics']) | FRAMES
        elif kind == 'Image':
            shown.add(config['imageMode']['imageTopic'])
        elif kind == 'Indicator':
            shown.add(topic(config['path']))
        elif kind == 'CallService':
            continue                           # a service: shown_services
        elif kind == 'Tab':
            continue                           # its panels are in configById too
        elif kind == 'Table':
            shown.add(topic(config['topicPath']))
        elif kind in ('Plot', 'StateTransitions'):
            shown |= {topic(p['value']) for p in config['paths']}
        elif kind == 'RawMessages':
            shown.add(topic(config['topicPath']))
        elif kind in ('DiagnosticSummary', 'DiagnosticStatusPanel'):
            shown.add(config['topicToRender'])
        elif kind == 'map':
            shown |= set(config.get('topicColors', {}))
            if config.get('followTopic'):
                shown.add(config['followTopic'])
        else:
            raise ValueError(f'{panel}: a panel this check does not know')
    return {t for t in shown if not t.startswith(SCRIPT_PREFIX)}


def advertised(payloads, services=None):
    """
    Return {topic: channel id} from the bridge's text messages (its 'advertise' ops).

    Given a set as services, it also collects the services the bridge advertises.
    """
    channels = {}
    for payload in payloads:
        message = json.loads(payload)
        if message.get('op') == 'advertise':
            channels.update({c['topic']: c['id'] for c in message['channels']})
        elif message.get('op') == 'unadvertise':
            gone = set(message['channelIds'])
            channels = {t: i for t, i in channels.items() if i not in gone}
        elif message.get('op') == 'advertiseServices' and services is not None:
            services.update(s['name'] for s in message['services'])
    return channels


def message_data(payload):
    """Return (subscription id, bytes of message) from a binary frame, or None if not data."""
    if not payload or payload[0] != 0x01:      # 0x01: messageData
        return None
    (subscription,) = struct.unpack_from('<I', payload, 1)
    return subscription, len(payload) - 13     # opcode, id (4), receive time (8)


class Connection:
    """A bare WebSocket client: enough to talk to a foxglove_bridge."""

    def __init__(self, url, protocol, timeout=5.0):
        parts = urlparse(url)
        self.host, self.port = parts.hostname, parts.port or 8765
        self.sock = socket.create_connection((self.host, self.port), timeout=timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(
            (f'GET {parts.path or "/"} HTTP/1.1\r\nHost: {self.host}:{self.port}\r\n'
             'Upgrade: websocket\r\nConnection: Upgrade\r\n'
             f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n'
             f'Sec-WebSocket-Protocol: {protocol}\r\n\r\n').encode())
        self.buf = b''
        while b'\r\n\r\n' not in self.buf:
            self.buf += self._recv()
        head, self.buf = self.buf.split(b'\r\n\r\n', 1)
        self.status = head.split(b'\r\n')[0].decode(errors='replace')
        self.ok = ' 101 ' in self.status

    def _recv(self):
        chunk = self.sock.recv(1 << 20)
        if not chunk:
            raise EOFError('the bridge closed the connection')
        return chunk

    def _take(self, n):
        while len(self.buf) < n:
            self.buf += self._recv()
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def send_text(self, text):
        """Send one text frame. A client's frames are masked (RFC 6455)."""
        data = text.encode()
        mask = os.urandom(4)
        n = len(data)
        head = bytes([0x81])
        if n < 126:
            head += bytes([0x80 | n])
        elif n < 1 << 16:
            head += bytes([0x80 | 126]) + struct.pack('>H', n)
        else:
            head += bytes([0x80 | 127]) + struct.pack('>Q', n)
        self.sock.sendall(head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def frames(self, seconds):
        """Yield (opcode, payload) for `seconds`: 1 text, 2 binary (the server doesn't mask)."""
        end = time.monotonic() + seconds
        parts, kind = [], None
        while time.monotonic() < end:
            try:
                first, second = self._take(2)
                n = second & 0x7f
                if n == 126:
                    (n,) = struct.unpack('>H', self._take(2))
                elif n == 127:
                    (n,) = struct.unpack('>Q', self._take(8))
                payload = self._take(n)
            except (socket.timeout, EOFError):
                return
            opcode = first & 0x0f
            if opcode in (1, 2):
                kind, parts = opcode, [payload]
            elif opcode == 0:                  # a continuation of a fragmented message
                parts.append(payload)
            else:
                continue                       # ping, pong, close
            if first & 0x80:                   # the last fragment
                yield kind, b''.join(parts)

    def text_frames(self, seconds):
        for opcode, payload in self.frames(seconds):
            if opcode == 1:
                yield payload

    def close(self):
        self.sock.close()


def connect(url):
    """Return (an open Connection, its protocol), or print why not and return None."""
    for protocol in PROTOCOLS:
        try:
            conn = Connection(url, protocol)
        except OSError as e:
            print(f'no bridge at {url}: {e}. Is foxglove:=true on?')
            return None
        if conn.ok:
            return conn, protocol
        conn.close()
    print(f'{url} refused both {" and ".join(PROTOCOLS)}: {conn.status}')
    return None


def measure(conn, channels, seconds):
    """Subscribe to each {topic: channel}; return {topic: (messages, bytes)} for seconds."""
    ids = {}
    subscriptions = []
    for n, (topic, channel) in enumerate(sorted(channels.items()), start=1):
        ids[n] = topic
        subscriptions.append({'id': n, 'channelId': channel})
    conn.send_text(json.dumps({'op': 'subscribe', 'subscriptions': subscriptions}))
    counts = {topic: [0, 0] for topic in channels}
    for opcode, payload in conn.frames(seconds):
        found = message_data(payload) if opcode == 2 else None
        if found and found[0] in ids:
            counts[ids[found[0]]][0] += 1
            counts[ids[found[0]]][1] += found[1]
    return {t: tuple(c) for t, c in counts.items()}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('url', nargs='?', default='ws://localhost:8765')
    ap.add_argument('--layout', action='append',
                    help='a layout file (default: every one in tools/foxglove/)')
    ap.add_argument('--seconds', type=float, default=5.0, help='how long to listen')
    ap.add_argument('--data', action='store_true',
                    help='subscribe, and count what each topic sends')
    args = ap.parse_args()

    layouts = [Path(p) for p in args.layout] if args.layout else LAYOUTS
    want, calls = {}, {}
    for path in layouts:
        layout = json.loads(path.read_text())
        for topic in shown_topics(layout):
            want.setdefault(topic, []).append(path.stem)
        for service in shown_services(layout):
            calls.setdefault(service, []).append(path.stem)
    found = connect(args.url)
    if found is None:
        return 2
    conn, protocol = found
    services = set()
    channels = advertised(conn.text_frames(args.seconds), services)
    print(f'{args.url} ({protocol}) serves {len(channels)} topics; '
          f'the layouts ({", ".join(p.stem for p in layouts)}) show {len(want)}:')
    faults = 0
    for topic in sorted(want):
        if topic in channels:
            print(f'  served    {topic}')
        elif topic in OPTIONAL:
            print(f'  off       {topic}   (only with {OPTIONAL[topic]})')
        else:
            print(f'  MISSING   {topic}   ({", ".join(want[topic])})')
            faults += 1
    for service in sorted(calls):
        # A watch-only bridge (the car's) advertises no services: none is a fault there.
        if service in services:
            print(f'  served    {service}   (a service)')
        else:
            why = OPTIONAL_SERVICES.get(service, 'a bridge that allows services')
            print(f'  off       {service}   (a service: only with {why})')
    if args.data:
        served = {t: c for t, c in channels.items() if t in want}
        counts = measure(conn, served, args.seconds)
        total = sum(b for _, b in counts.values())
        print(f'\nWhat each sends, over {args.seconds:g} s (a laptop showing every layout):')
        for topic, (messages, size) in sorted(counts.items()):
            line = (f'  {messages / args.seconds:6.1f} msg/s  '
                    f'{size / args.seconds / 1000:8.1f} kB/s  {topic}')
            if messages == 0:
                excused = topic in OPTIONAL or topic in WHILE_DRIVING
                line += '   silent' + (' (parked?)' if topic in WHILE_DRIVING else
                                       '' if excused else '  <- FAULT')
                faults += not excused
            print(line)
        print(f'  total {total / args.seconds / 1000:.1f} kB/s '
              f'({total * 8 / args.seconds / 1e6:.2f} Mbit/s)')
    conn.close()
    return 1 if faults else 0


if __name__ == '__main__':
    sys.exit(main())
