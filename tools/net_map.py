#!/usr/bin/env python3
"""
Map the network this computer is on, and the ROS domains talking on it.

Every machine, what it probably is, and which ROS domains are talking. Made for joining
the car's network from a laptop (Friday 16 Oct).

usage: pixi run bash tools/net_map.sh                     (or bash tools/net_map.sh, no pixi)
       pixi run bash tools/net_map.sh --listen 20         (listen longer for ROS)
       pixi run bash tools/net_map.sh --interface en7     (this network: an Ethernet dongle, say)
       pixi run bash tools/net_map.sh --subnet 192.168.1.0/24 --out net-map.md

Run it through tools/net_map.sh: on a Mac, a program started by Python sees an empty ARP
table (macOS's Local Network privacy), so the wrapper reads it with `arp` itself, after
this script's ping, and hands it back with --arp.

Read-only. It pings each address once, opens and closes a few TCP ports (SSH, web,
Foxglove), and listens to ROS's discovery messages. It never logs in, and never sends
anything to ROS. No ROS needed: the ROS part reads the raw discovery packets, so it hears
Humble and Jazzy alike.

What it prints, as a Markdown table to paste into the audit:
  IP, name, MAC, a guess at the maker (Raspberry Pi, NVIDIA, Espressif...), open ports,
  and the ROS domains each machine announces itself in.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import datetime
import ipaddress
import re
import select
import socket
import struct
import subprocess
import sys
import time

# The first half of a MAC says who made the network chip. A guess: a USB adapter or a
# changed MAC fools it. Sources: the IEEE register.
MAKERS = {
    'b8:27:eb': 'Raspberry Pi', 'dc:a6:32': 'Raspberry Pi', 'e4:5f:01': 'Raspberry Pi',
    '28:cd:c1': 'Raspberry Pi', 'd8:3a:dd': 'Raspberry Pi', '2c:cf:67': 'Raspberry Pi',
    '48:b0:2d': 'NVIDIA (Jetson?)', '00:04:4b': 'NVIDIA (Jetson?)',
    '00:08:dc': 'WIZnet (the wheel-encoder Pico?)',
    '24:0a:c4': 'Espressif (ESP32)', '24:6f:28': 'Espressif (ESP32)',
    '30:ae:a4': 'Espressif (ESP32)', 'a4:cf:12': 'Espressif (ESP32)',
    '7c:9e:bd': 'Espressif (ESP32)', '84:cc:a8': 'Espressif (ESP32)',
    '8c:aa:b5': 'Espressif (ESP32)', 'ac:67:b2': 'Espressif (ESP32)',
    '3c:61:05': 'Espressif (ESP32)', '94:b9:7e': 'Espressif (ESP32)',
    'c8:c9:a3': 'Espressif (ESP32)', 'e8:db:84': 'Espressif (ESP32)',
    '08:3a:f2': 'Espressif (ESP32)', '10:52:1c': 'Espressif (ESP32)',
    '40:22:d8': 'Espressif (ESP32)', '78:21:84': 'Espressif (ESP32)',
    'b4:e6:2d': 'Espressif (ESP32)', 'cc:50:e3': 'Espressif (ESP32)',
    'ec:62:60': 'Espressif (ESP32)', 'f4:cf:a2': 'Espressif (ESP32)',
    '60:76:88': 'Velodyne (lidar)',
}

# TCP ports worth knowing about on the car. Opening one and closing it changes nothing.
PORTS = {
    22: 'ssh',
    80: 'web',          # the ESP32's update page (S4), or a router
    443: 'https',
    5000: 'flask/airplay',  # the 2025-26 HUD's back end; on a Mac, AirPlay
    5900: 'vnc',
    8080: 'web-alt',
    8765: 'foxglove',   # foxglove_bridge: watch with Lichtblick
    8554: 'rtsp',       # MediaMTX: the 2025-26 camera video streams, on the camera Pis
    9090: 'websocket',  # the 2025-26 camera dashboard's live stream
}

# ROS 2 (DDS) discovery: each domain announces on its own multicast port.
DDS_GROUP = '239.255.0.1'
DDS_BASE, DDS_DOMAIN_GAIN = 7400, 250
DDS_VENDORS = {
    b'\x01\x01': 'RTI Connext', b'\x01\x0f': 'Fast DDS', b'\x01\x10': 'Cyclone DDS',
    b'\x01\x12': 'Zenoh?',
}


# ---- the parts with no network, tested in tools/tests/test_net_map.py ----

def normal_mac(mac):
    """'0:8:dc:1:2:3' (macOS drops leading zeros) -> '00:08:dc:01:02:03'."""
    parts = re.split('[:-]', mac.strip().lower())
    if len(parts) != 6 or not all(re.fullmatch('[0-9a-f]{1,2}', p) for p in parts):
        return None
    return ':'.join(p.zfill(2) for p in parts)


def maker(mac):
    """Guess who made the network chip, from the MAC."""
    if not mac:
        return ''
    if mac[:8] in MAKERS:
        return MAKERS[mac[:8]]
    if int(mac[:2], 16) & 0x02:
        return 'private (random) MAC: a phone or laptop?'
    return ''


def parse_arp(text):
    """`arp -an` (macOS and Linux) or `ip neigh` -> {ip: mac}."""
    found = {}
    for line in text.splitlines():
        ip = re.search(r'\((\d+\.\d+\.\d+\.\d+)\)', line) or \
            re.match(r'(\d+\.\d+\.\d+\.\d+)\s', line)
        mac = re.search(r'(?:at|lladdr) ([0-9a-fA-F:]+)', line)
        if ip and mac and normal_mac(mac.group(1)):
            found[ip.group(1)] = normal_mac(mac.group(1))
    return found


def parse_interfaces(text):
    """`ifconfig` or `ip -4 -o addr` -> [IPv4Interface], without loopback."""
    found = []
    for m in re.finditer(r'inet (\d+\.\d+\.\d+\.\d+)(?:/(\d+)| netmask (0x[0-9a-f]+|'
                         r'\d+\.\d+\.\d+\.\d+))?', text):
        ip, prefix, mask = m.groups()
        if mask and mask.startswith('0x'):
            mask = str(ipaddress.IPv4Address(int(mask, 16)))
        iface = ipaddress.IPv4Interface(f'{ip}/{prefix or mask or 24}')
        if not iface.ip.is_loopback and not iface.ip.is_link_local:
            found.append(iface)
    return found


def parse_named_interfaces(text):
    """`ifconfig` or `ip -4 -o addr` -> [(name, IPv4Interface)], without loopback."""
    found = []
    name = None
    for line in text.splitlines():
        head = re.match(r'(?:\d+:\s+)?([A-Za-z][\w.\-]*):?\s', line)
        if head and not line.startswith((' ', '\t')):
            name = head.group(1)
        for iface in parse_interfaces(line):
            found.append((name, iface))
    return found


def route_interface(text):
    """Return the default route's interface, from `route -n get default` or `ip route`."""
    m = re.search(r'(?:interface:|dev) (\S+)', text)
    return m.group(1) if m else None


def choose_interface(named, internet=None, wanted=None):
    """
    Pick the network to sweep -> ((name, IPv4Interface), why).

    With a phone hotspot for the internet and an Ethernet cable to the car, the car is the
    network that *isn't* the default route. `wanted` (--interface) always wins.
    """
    if wanted:
        for item in named:
            if item[0] == wanted:
                return item, f'{wanted}, as asked'
        raise ValueError(f'no IPv4 address on {wanted}: have ' +
                         ', '.join(f'{n} {i}' for n, i in named))
    if len(named) == 1:
        return named[0], 'the only network'
    others = [item for item in named if item[0] != internet]
    if others:
        return others[0], (f'not {internet}, the internet route (the car is usually the '
                           'other network)')
    return named[0], 'the first network'


def domain_of_port(port):
    """Return the ROS domain whose discovery multicast uses this UDP port, or None."""
    domain, rest = divmod(port - DDS_BASE, DDS_DOMAIN_GAIN)
    return domain if rest == 0 and 0 <= domain <= 232 else None


def read_rtps(packet):
    """Read an RTPS (DDS) packet -> (vendor, participant id), or None if it isn't one."""
    if len(packet) < 20 or packet[:4] != b'RTPS':
        return None
    vendor = DDS_VENDORS.get(packet[6:8], f'vendor {packet[6:8].hex()}')
    return vendor, packet[8:20].hex()


def table(rows, header):
    """Rows of strings -> a Markdown table."""
    lines = ['| ' + ' | '.join(header) + ' |', '|' + '---|' * len(header)]
    lines += ['| ' + ' | '.join(r) + ' |' for r in rows]
    return '\n'.join(lines)


# ---- the network parts ----

def run(cmd, timeout=10):
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return done.stdout
    except (OSError, subprocess.TimeoutExpired):
        return ''


def my_interfaces():
    return parse_named_interfaces(run(['ip', '-4', '-o', 'addr', 'show']) or run(['ifconfig']))


def default_route():
    return run(['route', '-n', 'get', 'default']) if sys.platform == 'darwin' else \
        run(['ip', 'route', 'show', 'default'])


def gateway():
    m = re.search(r'(?:gateway:|via) (\d+\.\d+\.\d+\.\d+)', default_route())
    return m.group(1) if m else None


def ping(ip):
    wait = ['-t', '1'] if sys.platform == 'darwin' else ['-W', '1']
    return subprocess.run(['ping', '-c', '1', *wait, ip], capture_output=True).returncode == 0


def open_ports(ip, timeout=0.4):
    found = []
    for port, name in PORTS.items():
        try:
            with socket.create_connection((ip, port), timeout=timeout):
                found.append(f'{port} {name}')
        except OSError:
            pass
    return found


def name_of(ip):
    try:
        return socket.gethostbyaddr(ip)[0]
    except OSError:
        return ''


def listen_ros(seconds, my_ip, domains=range(0, 102)):
    """Listen for DDS discovery -> {ip: {domain: {participant: vendor}}}, and errors."""
    socks, heard, errors = {}, {}, []
    for domain in domains:
        port = DDS_BASE + DDS_DOMAIN_GAIN * domain
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, 'SO_REUSEPORT'):
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            s.bind(('', port))
            s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack(
                '4s4s', socket.inet_aton(DDS_GROUP), socket.inet_aton(my_ip)))
            s.setblocking(False)
            socks[s] = domain
        except OSError as e:
            errors.append(f'domain {domain}: {e}')
            s.close()
    end = time.monotonic() + seconds
    try:
        while socks and time.monotonic() < end:
            ready, _, _ = select.select(list(socks), [], [], 0.5)
            for s in ready:
                try:
                    packet, (ip, _) = s.recvfrom(65535)
                except OSError:
                    continue
                rtps = read_rtps(packet)
                if rtps:
                    vendor, who = rtps
                    heard.setdefault(ip, {}).setdefault(socks[s], {})[who] = vendor
    finally:
        for s in socks:
            s.close()
    return heard, errors


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    ap.add_argument('--subnet', help='the addresses to sweep, such as 192.168.1.0/24 '
                                     "(default: this computer's network)")
    ap.add_argument('--interface', help='the network to sweep, such as en7 (an Ethernet '
                    'dongle); default: the one that is not the internet route')
    ap.add_argument('--listen', type=float, default=10.0,
                    help='seconds to listen for ROS discovery (default 10; 0 = skip)')
    ap.add_argument('--out', help='also save the result to this Markdown file')
    ap.add_argument('--ping-only', action='store_true',
                    help='only ping every address (fills the ARP table), then stop')
    ap.add_argument('--arp', help="a file with `arp -an` or `ip neigh` output: don't ping, "
                                  'take the machines from it')
    args = ap.parse_args()

    mine = my_interfaces()
    if not mine:
        sys.exit("No network address: join the car's WiFi or plug in its Ethernet first.")
    raw = run(['ip', '-4', '-o', 'addr', 'show']) or run(['ifconfig'])
    if re.search(r'inet 169\.254\.', raw):
        print('Note: a network here has only a 169.254.x.x address: nothing on it gave this '
              'computer an address (a plain switch has no DHCP). Set one by hand: '
              'docs/history/friday-2026-10-16.md, "Ethernet dongle and phone hotspot".')
    internet = route_interface(default_route())
    try:
        (name, me), why = choose_interface(mine, internet, args.interface)
    except ValueError as e:
        sys.exit(str(e))
    if len(mine) > 1:
        print('Networks here: ' + ', '.join(f'{n} {i}' for n, i in mine) +
              f'. Sweeping {name} ({why}); another: --interface <name>.')
    net = ipaddress.ip_network(args.subnet) if args.subnet else me.network
    if net.num_addresses > 1024:
        sys.exit(f'{net} has {net.num_addresses} addresses: give a /22 or smaller with '
                 '--subnet (a car network is usually a /24).')
    gw = gateway()
    answered = set()
    if not args.arp:
        print(f'This computer: {me.ip} on {net}. Router: {gw or "none"}.')
        hosts = [str(h) for h in net.hosts()]
        print(f'Pinging {len(hosts)} addresses once each...', flush=True)
        with ThreadPoolExecutor(64) as pool:
            answered = {ip for ip, ok in zip(hosts, pool.map(ping, hosts)) if ok}
        print(f'{len(answered)} answered.')
        if args.ping_only:
            return
    if args.arp:
        with open(args.arp) as f:
            arp_text = f.read()
    else:
        arp_text = run(['arp', '-an']) or run(['ip', 'neigh'])
    macs = {ip: mac for ip, mac in parse_arp(arp_text).items()
            if ipaddress.ip_address(ip) in net}
    alive = sorted(answered | set(macs) | {str(me.ip)}, key=ipaddress.ip_address)

    print(f'Found {len(alive)}. Checking names and ports...', flush=True)
    with ThreadPoolExecutor(32) as pool:
        names = dict(zip(alive, pool.map(name_of, alive)))
        ports = dict(zip(alive, pool.map(open_ports, alive)))

    heard, errors = {}, []
    if args.listen > 0:
        print(f'Listening {args.listen:g} s for ROS discovery (domains 0-101)...', flush=True)
        heard, errors = listen_ros(args.listen, str(me.ip))
        for ip in heard:
            if ip not in alive and ipaddress.ip_address(ip) in net:
                alive.append(ip)
        alive.sort(key=ipaddress.ip_address)

    rows = []
    for ip in alive:
        notes = []
        if ip == str(me.ip):
            notes.append('this computer')
        if ip == gw:
            notes.append('router')
        ros = '; '.join(
            f'domain {d}: {len(who)} × {", ".join(sorted(set(who.values())))}'
            for d, who in sorted(heard.get(ip, {}).items()))
        rows.append([ip, names.get(ip, ''), macs.get(ip, ''),
                     ', '.join(filter(None, [maker(macs.get(ip, ''))] + notes)),
                     ', '.join(ports.get(ip, [])), ros])
    when = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
    report = (f'## Network map, {when}\n\n{net}, seen from {me.ip}\n\n' +
              table(rows, ['IP', 'Name', 'MAC', 'Maker (a guess)', 'Open ports',
                           'ROS: domain, participants × middleware']) + '\n')

    domains = sorted({d for by_d in heard.values() for d in by_d})
    report += '\n' + (
        f'ROS domains heard: {", ".join(map(str, domains))}. A participant is roughly '
        'one ROS program. Ours is 42: pick one nobody else uses.\n' if domains else
        'No ROS discovery heard. Either nothing ROS is running, multicast is blocked '
        '(WiFi often drops it), or the machines use static peers.\n')
    if errors:
        report += f'\nCould not listen on {len(errors)} domains (first: {errors[0]}).\n'
    if len(alive) <= 2 and sys.platform == 'darwin':
        report += ('\nAlmost nothing answered. On a Mac, check System Settings > Privacy & '
                   'Security > Local Network: your terminal app must be on.\n')
    print('\n' + report)
    if args.out:
        with open(args.out, 'w') as f:
            f.write(report)
        print(f'Saved to {args.out}')


if __name__ == '__main__':
    main()
