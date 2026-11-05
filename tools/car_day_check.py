#!/usr/bin/env python3
"""
Before you walk to the car: is this laptop ready for car day? Says what to fix.

usage: pixi run car-day-check          (or: python3 tools/car_day_check.py [--root DIR])

Each line says OK, MISSING or REMIND, and for MISSING the command that fixes it.
REMIND is a thing only a person can tick (print the sheet, copy the kit to the stick).
Nothing here changes anything, needs ROS, or reads a password: the Keychain check asks
only whether each entry exists (`security find-generic-password -s <entry>`, no `-w`).
The day itself: docs/car-day.md. `pixi run car-day-build` makes what it can.

Exit code: 0 when nothing is MISSING (reminders don't count), else 1.
"""

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
GITHUB = 'https://github.com/AAV-2025-2026/ecolo-av.git'
CAR_DAY = 'car-day'                 # pixi run car-day-build writes here; git ignores it
ESP32_DIR = f'{CAR_DAY}/esp32'
FIRMWARES = {                       # what car-day-build saves, and what each is for
    'spine-esp32-umod-v2.elf': 'Rust, version 2 frames (the heartbeat; the default)',
    'spine-esp32-umod-v1.elf': 'Rust, the 13-byte frames (link-v1)',
    'spine-esp32-umod-v2-uplink.elf': 'Rust, version 2 with the uplink (once H9, H10 say yes)',
}
LEGACY = 'firmware/legacy/aav_umod_esp32'
MODELS = {
    'models/rf-detr-nano.pth': ('the detector (RF-DETR Nano): the Orin builds its engine '
                                'from it', 100_000_000),
    'models/yolo11n.pt': ('the YOLO fallback (detector:=yolo)', 1_000_000),
}
LAYOUT_DIR = 'tools/foxglove'
LICHTBLICK_APPS = ('/Applications/Lichtblick.app', '~/Applications/Lichtblick.app')
HOSTS = '~/aav-audit/hosts.txt'
MY_CYCLONE = '~/.config/aav/cyclonedds.xml'          # tools/ros.sh reads it, if it's there
CAR_CYCLONE = '~/.config/aav/cyclonedds-car.xml'     # the same, kept aside until the car
MIN_FREE_GB = 20                    # bags: about 1 GB a minute with the lidar's points
STATUSES = ('OK', 'MISSING', 'REMIND')


@dataclass
class Line:
    """One check's answer."""

    status: str        # OK, MISSING or REMIND
    what: str          # short name
    detail: str        # what was found
    fix: str = ''      # the command (or the step) that fixes it


def run(cmd, cwd=None, timeout=20):
    """Return (exit code, output) of a command; (None, '') if it can't run."""
    try:
        done = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True,
                              timeout=timeout)
        return done.returncode, (done.stdout + done.stderr).strip()
    except (OSError, subprocess.TimeoutExpired):
        return None, ''


def git(root, *args, timeout=20):
    """Return git's output in `root`, or None if it failed."""
    code, out = run(['git', *args], cwd=root, timeout=timeout)
    return out if code == 0 else None


# --- the code ------------------------------------------------------------------------

# main only follows the team's main (CONTRIBUTING.md): update it, never push it.
UPDATE_MAIN = 'git fetch upstream && git merge --ff-only upstream/main'


def repo_line(branch, head, remote, relation, dirty):
    """
    Say whether this checkout is what GitHub's main has.

    relation: 'same', 'behind' (GitHub has more), 'ahead' (this has more), or 'apart'.
    """
    if remote is None:
        return Line('REMIND', 'repo', f'{branch} at {head[:8]}; GitHub not reached (no '
                    'internet?)', 'connect, then run this again')
    if branch != 'main':
        return Line('MISSING', 'repo', f'on branch {branch}, not main',
                    f'git switch main && {UPDATE_MAIN}')
    if dirty:
        return Line('MISSING', 'repo', 'uncommitted changes: the car kit and the builds '
                    'hold only commits', 'commit them, or git stash push -m car-day')
    if relation == 'same':
        return Line('OK', 'repo', f'main at {head[:8]}, the same as GitHub')
    if relation == 'ahead':
        return Line('MISSING', 'repo', f"main at {head[:8]} has commits GitHub hasn't",
                    'pixi run test, then move them to a branch and open a pull request '
                    '(CONTRIBUTING.md)')
    return Line('MISSING', 'repo', f'main at {head[:8]}, GitHub at {remote[:8]}',
                UPDATE_MAIN)


def check_repo(root):
    branch = git(root, 'rev-parse', '--abbrev-ref', 'HEAD') or '?'
    head = git(root, 'rev-parse', 'HEAD') or '?'
    out = git(root, 'ls-remote', GITHUB, 'refs/heads/main', timeout=15)
    remote = out.split()[0] if out else None
    relation = 'apart'
    if remote == head:
        relation = 'same'
    elif remote and git(root, 'cat-file', '-e', f'{remote}^{{commit}}') is not None:
        if git(root, 'merge-base', '--is-ancestor', head, remote) is not None:
            relation = 'behind'
        elif git(root, 'merge-base', '--is-ancestor', remote, head) is not None:
            relation = 'ahead'
    dirty = bool(git(root, 'status', '--porcelain', '--untracked-files=no'))
    return repo_line(branch, head, remote, relation, dirty)


def newest_tracked(root, folder):
    """Return the newest modification time of the files git tracks in `folder`, or 0."""
    out = git(root, 'ls-files', '-z', folder)
    newest = 0.0
    for name in (out or '').split('\0'):
        try:
            newest = max(newest, (root / name).stat().st_mtime)
        except OSError:
            pass
    return newest


def build_line(events, built_at, newest_src):
    """
    Say whether `pixi run build` finished, after the last change to src/.

    events: colcon's log/latest_build/events.log as text, or None if there's none.
    """
    if events is None:
        return Line('MISSING', 'build', 'never built here', 'pixi run build')
    failed = sorted({name for name, rc in
                     re.findall(r"\{'identifier': '([^']+)', 'rc': (\d+)\}", events)
                     if rc != '0'})
    when = time.strftime('%d %b %H:%M', time.localtime(built_at))
    if failed:
        return Line('MISSING', 'build', f'{len(failed)} failed on {when}: '
                    f'{", ".join(failed)}', 'pixi run build (the end of its output says why)')
    if newest_src > built_at:
        return Line('MISSING', 'build', f'ros/ changed since the last build ({when})',
                    'pixi run build')
    count = len(re.findall(r'JobEnded', events))
    return Line('OK', 'build', f'{count} packages built on {when}')


def check_build(root):
    events = root / 'log' / 'latest_build' / 'events.log'
    if not events.is_file() or not (root / 'install').is_dir():
        return build_line(None, 0, 0)
    return build_line(events.read_text(errors='replace'), events.stat().st_mtime,
                      newest_tracked(root, 'ros'))


def kit_line(kit_txt, head, lock_hash, kit_models):
    """
    Say whether car-kit/ (pixi run car-kit) was made from this commit and pixi.lock.

    kit_txt: car-kit/KIT.txt as text, or None. kit_models: file names in car-kit/models.
    """
    if kit_txt is None:
        return Line('MISSING', 'car kit', 'no car-kit/', 'pixi run car-kit')
    made = re.search(r'made (.+?) on', kit_txt)
    commit = re.search(r'^commit:\s+(\w+)', kit_txt, re.M)
    lock = re.search(r'^lock file:\s+pixi\.lock (\w+)', kit_txt, re.M)
    when = made.group(1) if made else '?'
    problems = []
    if not commit or not head.startswith(commit.group(1)):
        problems.append(f'its code is {commit.group(1) if commit else "?"}, '
                        f'main is {head[:8]}')
    if not lock or lock.group(1) != lock_hash:
        problems.append('pixi.lock changed since: its environment is old too')
    if 'rf-detr-nano.pth' not in kit_models:
        problems.append('no RF-DETR model in it (pixi run get models first)')
    if problems:
        return Line('MISSING', 'car kit', f'made {when}; ' + '; '.join(problems),
                    'pixi run car-kit')
    return Line('OK', 'car kit', f'made {when} from this commit')


def check_kit(root):
    kit = root / 'car-kit'
    txt = kit / 'KIT.txt'
    head = git(root, 'rev-parse', 'HEAD') or '?'
    lock = hashlib.sha256((root / 'pixi.lock').read_bytes()).hexdigest()[:12]
    models = [p.name for p in (kit / 'models').glob('*')] if (kit / 'models').is_dir() else []
    return kit_line(txt.read_text() if txt.is_file() else None, head, lock, models)


def stick_line(local_kit, sticks):
    """Is the same car kit on a mounted USB stick? sticks: {volume: its KIT.txt text}."""
    if local_kit is None:
        return Line('REMIND', 'USB stick', 'make the kit first, then copy car-kit/ to it')
    same = [v for v, txt in sticks.items() if txt == local_kit]
    if same:
        return Line('OK', 'USB stick', f'this kit is on {same[0]}')
    if sticks:
        return Line('REMIND', 'USB stick', f'an older kit on {", ".join(sticks)}',
                    'cp -R car-kit /Volumes/<stick>/')
    return Line('REMIND', 'USB stick', 'no stick with car-kit/ plugged in',
                'cp -R car-kit /Volumes/<stick>/')


def check_stick(root):
    local = root / 'car-kit' / 'KIT.txt'
    sticks = {}
    for kit in Path('/Volumes').glob('*/car-kit/KIT.txt'):
        try:
            sticks[str(kit.parents[1])] = kit.read_text()
        except OSError:
            pass
    return stick_line(local.read_text() if local.is_file() else None, sticks)


# --- models and the camera ---------------------------------------------------------

def check_models(root):
    lines = []
    for name, (purpose, least) in MODELS.items():
        path = root / name
        if path.is_file() and path.stat().st_size >= least:
            mb = path.stat().st_size / 1e6
            lines.append(Line('OK', 'model', f'{name} ({mb:.0f} MB): {purpose}'))
        else:
            lines.append(Line('MISSING', 'model', f'no {name}: {purpose}',
                              'pixi run get models'))
    onnx = root / 'models' / 'rf-detr-nano.onnx'
    if onnx.is_file():
        lines.append(Line('OK', 'model', f'models/rf-detr-nano.onnx '
                          f"({onnx.stat().st_size / 1e6:.0f} MB): the Orin's engine "
                          'needs only TensorRT from it (trtexec)'))
    else:
        lines.append(Line('REMIND', 'model', 'no models/rf-detr-nano.onnx: make it here, '
                          'and the Orin needs only TensorRT (else rfdetr and onnx too)',
                          'docs/car-day.md, session 0'))
    return lines


def check_mediamtx(root):
    exe = root / 'vendor' / 'mediamtx' / 'mediamtx'
    if exe.is_file() and os.access(exe, os.X_OK):
        return Line('OK', 'MediaMTX', 'vendor/mediamtx (to test a stream on the laptop)')
    return Line('MISSING', 'MediaMTX', 'no vendor/mediamtx', 'pixi run get mediamtx')


# The car has no internet: the pixi environments the laptop uses there go on at home.
CAR_ENVS = {
    'gst': ('GStreamer', "the camera stream's fast decoder (else ffmpeg, ~135 ms slower)"),
    'calib': ('calibrator', 'ROS camera_calibration, for session 5 (about 1 GB)'),
}


def check_envs(root):
    """Return a line for each pixi environment the laptop needs at the car."""
    lines = []
    for env, (what, why) in CAR_ENVS.items():
        if (root / '.pixi' / 'envs' / env / 'conda-meta').is_dir():
            lines.append(Line('OK', what, f'.pixi/envs/{env}: {why}'))
        else:
            lines.append(Line('MISSING', what, f'no .pixi/envs/{env}: {why}',
                              f'pixi install -e {env}'))
    return lines


# --- the ESP32 -----------------------------------------------------------------------

def tree_id(root, folder):
    """Return the committed tree of `folder`, '-dirty' if it has uncommitted changes."""
    tree = git(root, 'rev-parse', f'HEAD:{folder}') or '?'
    dirty = git(root, 'status', '--porcelain', '--', folder)
    return tree + ('-dirty' if dirty else '')


def parse_build_txt(text):
    """car-day/esp32/BUILD.txt's 'key: value' lines, as a dict."""
    out = {}
    for line in (text or '').splitlines():
        key, sep, value = line.partition(':')
        if sep:
            out[key.strip()] = value.strip()
    return out


def firmware_lines(build, files, spine_tree, legacy_tree):
    """
    Say whether the ESP32 builds of car-day-build are there, from today's source.

    build: BUILD.txt as a dict; files: the names in car-day/esp32.
    """
    fix = 'pixi run car-day-build'
    if not build:
        return [Line('MISSING', 'ESP32 Rust', 'nothing built for car day', fix),
                Line('MISSING', 'ESP32 QEMU', 'esp qemu-check not run for car day', fix),
                Line('MISSING', 'ESP32 C++', 'the way back not built', fix)]
    when = build.get('made', '?')
    lines = []
    spine_now = build.get('firmware/spine') == spine_tree and not spine_tree.endswith('dirty')
    missing = [n for n in FIRMWARES if n not in files]
    if missing:
        lines.append(Line('MISSING', 'ESP32 Rust', f'no {", ".join(missing)}', fix))
    elif not spine_now:
        lines.append(Line('MISSING', 'ESP32 Rust', f'built {when}, but firmware/spine '
                          'changed since (or has uncommitted changes)', fix))
    else:
        lines.append(Line('OK', 'ESP32 Rust',
                          f'v2, v1 and v2-uplink built {when}, in {ESP32_DIR}/'))
    qemu = build.get('esp qemu-check', '')
    if qemu.startswith('PASS') and spine_now:
        lines.append(Line('OK', 'ESP32 QEMU', f'esp qemu-check passed {when}: brakes '
                          'when the heartbeat stops'))
    elif qemu.startswith('skipped'):
        lines.append(Line('MISSING', 'ESP32 QEMU', 'skipped: no QEMU here',
                          'pixi run esp qemu-tools, then ' + fix))
    else:
        lines.append(Line('MISSING', 'ESP32 QEMU', f'esp qemu-check: {qemu or "not run"}'
                          + ('' if spine_now else ' (or the firmware changed since)'), fix))
    legacy_now = (build.get(LEGACY) == legacy_tree and not legacy_tree.endswith('dirty'))
    if build.get('legacy-esp32', '').startswith('PASS') and legacy_now:
        lines.append(Line('OK', 'ESP32 C++', f'the 2025-26 C++ builds (the way back), '
                          f'{when}'))
    else:
        lines.append(Line('MISSING', 'ESP32 C++', 'the way back: '
                          f'{build.get("legacy-esp32") or "not built"}', fix))
    return lines


def check_firmware(root):
    folder = root / ESP32_DIR
    build_txt = folder / 'BUILD.txt'
    build = parse_build_txt(build_txt.read_text()) if build_txt.is_file() else {}
    files = [p.name for p in folder.glob('*')] if folder.is_dir() else []
    return firmware_lines(build, files, tree_id(root, 'firmware/spine'),
                          tree_id(root, LEGACY))


def check_esp_tools(root, toolchains):
    """toolchains: `rustup toolchain list`'s output, or None without rustup."""
    have = [(root / 'vendor/esp/export-esp.sh').is_file(),
            (root / 'vendor/cargo-tools/bin/espflash').is_file(),
            bool(toolchains and re.search(r'^esp( |$)', toolchains, re.M))]
    if all(have):
        return Line('OK', 'ESP32 tools', 'Xtensa Rust, espflash (vendor/esp, '
                    'vendor/cargo-tools)')
    return Line('MISSING', 'ESP32 tools', 'no Xtensa Rust or espflash: needed to flash '
                'and to read the backup', 'pixi run esp tools (needs rustup; 1.5 GB)')


def secrets_line(secrets, example):
    """Compare the C++ firmware's WiFi file (bytes, or None) with the example."""
    if secrets is None or secrets == example:
        return Line('REMIND', 'ESP32 C++', 'secrets.h has the placeholders: a C++ built '
                    "again can't join the WiFi; the backup (session 6) restores it all",
                    f'{LEGACY}/src/secrets.h: the WiFi from the team, typed in, never '
                    'committed')
    return Line('OK', 'ESP32 C++', 'secrets.h filled in (not shown)')


def check_secrets(root):
    src = root / LEGACY / 'src'
    secrets = src / 'secrets.h'
    example = src / 'secrets.h.example'
    return secrets_line(secrets.read_bytes() if secrets.is_file() else None,
                        example.read_bytes() if example.is_file() else b'')


# --- watching, the docs --------------------------------------------------------------

def check_layouts(root):
    sys.path.insert(0, str(root / 'tools'))
    lines = []
    try:
        import layout_check
        counts = []
        for path in sorted((root / LAYOUT_DIR).glob('*.json')):
            topics = layout_check.shown_topics(json.loads(path.read_text()))
            counts.append(f'{path.stem} ({len(topics)} topics)')
        if len(counts) < 4:
            raise ValueError(f'{len(counts)} layouts')
        lines.append(Line('OK', 'layouts', ', '.join(counts) + '; checked live at the '
                          'car (session 8)'))
    except (ImportError, ValueError, OSError) as err:
        lines.append(Line('MISSING', 'layouts', f'tools/foxglove/: {err}',
                          'git checkout tools/foxglove'))
    apps = [a for a in LICHTBLICK_APPS if Path(a).expanduser().exists()]
    if apps:
        lines.append(Line('OK', 'Lichtblick', apps[0]))
    elif sys.platform == 'darwin':
        lines.append(Line('MISSING', 'Lichtblick', 'not in Applications',
                          'github.com/lichtblick-suite/lichtblick, Releases: '
                          'the mac-universal .dmg (docs/foxglove.md)'))
    else:
        lines.append(Line('REMIND', 'Lichtblick', 'install it (docs/foxglove.md)'))
    return lines


def docs_line(pdf_time, newest_page):
    """Is docs/html/aav-docs.pdf newer than every page? pdf_time None: no PDF."""
    if pdf_time is None:
        return Line('MISSING', 'docs PDF', 'no docs/html/aav-docs.pdf to read offline',
                    'pixi run docs')
    when = time.strftime('%d %b %H:%M', time.localtime(pdf_time))
    if newest_page > pdf_time:
        return Line('MISSING', 'docs PDF', f'built {when}; a page changed since',
                    'pixi run docs')
    return Line('OK', 'docs PDF', f'docs/html/aav-docs.pdf, built {when}')


def check_docs(root):
    pdf = root / 'docs' / 'html' / 'aav-docs.pdf'
    pages = list((root / 'docs').glob('*.md'))
    newest = max((p.stat().st_mtime for p in pages if p.is_file()), default=0)
    return docs_line(pdf.stat().st_mtime if pdf.is_file() else None, newest)


# --- this laptop on the car's network --------------------------------------------------

def card_named(xml):
    """Return the network card a Cyclone DDS settings file names, or None."""
    found = re.search(r'<NetworkInterface\b[^>]*\bname="([^"]+)"', xml or '')
    return found.group(1) if found else None


def cyclone_line(pixi_toml, mine, for_the_car):
    """
    Say whether Cyclone DDS is set, and the car's network card named for this laptop.

    mine: MY_CYCLONE's text (in use now), for_the_car: CAR_CYCLONE's (copied into place
    at the car), each None if there's none. A card named in MY_CYCLONE must exist, or
    every ROS program here fails: so at home it's kept as CAR_CYCLONE.
    """
    if 'RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"' not in pixi_toml:
        return Line('MISSING', 'Cyclone', "pixi.toml doesn't pick Cyclone DDS",
                    'git checkout pixi.toml')
    if card_named(mine):
        return Line('OK', 'Cyclone', f'Cyclone DDS on {card_named(mine)} ({MY_CYCLONE}, in '
                    'use now: move it away at home, where that card may be missing)')
    if card_named(for_the_car):
        return Line('OK', 'Cyclone', f'Cyclone DDS; {CAR_CYCLONE} names '
                    f'{card_named(for_the_car)}: at the car, cp it to {MY_CYCLONE}')
    return Line('REMIND', 'Cyclone', "Cyclone DDS uses one network card: name the car's "
                f'(the dongle) in {CAR_CYCLONE}', 'docs/car-day.md, session 0')


def check_cyclone(root):
    def text(path):
        path = Path(path).expanduser()
        return path.read_text() if path.is_file() else None
    return cyclone_line((root / 'pixi.toml').read_text(), text(MY_CYCLONE),
                        text(CAR_CYCLONE))


def keychain_entries(hosts_text):
    """Return the Keychain entry names in hosts.txt's third column: [(label, entry)]."""
    out = []
    for line in hosts_text.splitlines():
        cols = line.split('#')[0].split()
        if len(cols) >= 3:
            out.append((cols[0], cols[2]))
    return out


def keychain_line(entries, present):
    """entries: [(label, entry)]; present: the set of entries the Keychain has."""
    if not entries:
        return Line('MISSING', 'logins', f'no {HOSTS} (machine, user@ip, Keychain entry)',
                    'docs/history/friday-todo-2026-10-16.md: make the hosts file')
    gone = [e for _, e in entries if e not in present]
    if gone:
        return Line('MISSING', 'logins', f'no Keychain entry {", ".join(gone)}',
                    'security add-generic-password -s <entry> -a <user> -w '
                    '(it asks for the password)')
    return Line('OK', 'logins', f'{len(entries)} of {len(entries)} in the Keychain: '
                + ', '.join(e for _, e in entries))


def check_keychain():
    hosts = Path(HOSTS).expanduser()
    entries = keychain_entries(hosts.read_text()) if hosts.is_file() else []
    if sys.platform != 'darwin' or not shutil.which('security'):
        return Line('REMIND', 'logins', "not a Mac: have the car's logins with you")
    # Existence only: without -w, nothing secret is printed, and the output is dropped.
    present = {e for _, e in entries
               if run(['security', 'find-generic-password', '-s', e])[0] == 0}
    return keychain_line(entries, present)


def check_ssh_key():
    keys = sorted(Path('~/.ssh').expanduser().glob('id_*.pub'))
    if keys:
        return Line('OK', 'SSH key', f'~/.ssh/{keys[0].name}')
    return Line('MISSING', 'SSH key', 'none', 'ssh-keygen -t ed25519')


def disk_line(free_bytes):
    gb = free_bytes / 1e9
    if gb < MIN_FREE_GB:
        return Line('MISSING', 'disk', f'{gb:.0f} GB free; bags need {MIN_FREE_GB}+',
                    'make room (bags/ grows about 1 GB a minute with the lidar)')
    return Line('OK', 'disk', f'{gb:.0f} GB free for bags')


REMINDERS = [
    ('sheet', 'print docs/measure-the-car.md (2 copies) and a pen', 'session 2'),
    ('kit bag', 'stands, the RC transmitter, the e-stop, tape, plumb line, 4 scales, '
     'blocks, multimeter, dongle, cable', 'docs/car-day.md, "What to bring"'),
    ('questions', 'the hardware lead is coming? H1-H8 need them', 'session 3'),
]


def checks(root):
    """Every line, in the order of the day."""
    lines = [check_repo(root), check_build(root), check_kit(root), check_stick(root)]
    lines += check_models(root)
    lines.append(check_mediamtx(root))
    lines += check_envs(root)
    toolchains = run(['rustup', 'toolchain', 'list'])[1] if shutil.which('rustup') else None
    lines.append(check_esp_tools(root, toolchains))
    lines += check_firmware(root)
    lines.append(check_secrets(root))
    lines += check_layouts(root)
    lines.append(check_docs(root))
    lines += [check_cyclone(root), check_keychain(), check_ssh_key(),
              disk_line(shutil.disk_usage(root).free)]
    lines += [Line('REMIND', what, detail, fix) for what, detail, fix in REMINDERS]
    return lines


def show(lines, colour=False):
    """Print the lines, then the totals; return how many are MISSING."""
    paint = {'OK': '32', 'MISSING': '31', 'REMIND': '33'}
    for ln in lines:
        tag = f'{ln.status:<7}'
        if colour:
            tag = f'\033[1;{paint[ln.status]}m{tag}\033[0m'
        print(f'  {tag}  {ln.what:<12} {ln.detail}')
        if ln.fix and ln.status != 'OK':
            print(f'  {"":<7}  {"":<12} -> {ln.fix}')
    n = {s: sum(ln.status == s for ln in lines) for s in STATUSES}
    print(f'\n{n["OK"]} OK, {n["MISSING"]} missing, {n["REMIND"]} to tick by hand.')
    return n['MISSING']


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--root', type=Path, default=ROOT, help='the checkout to check')
    args = ap.parse_args()
    root = args.root.resolve()
    print(f'Car day: is {root} ready? (docs/car-day.md)\n')
    return 1 if show(checks(root), sys.stdout.isatty()) else 0


if __name__ == '__main__':
    sys.exit(main())
