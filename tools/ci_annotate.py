#!/usr/bin/env python3
"""
Turn a failed `pixi run test` into GitHub annotations, which anyone can read.

GitHub hides a job's log from anyone not signed in, but serves its annotations to all:
  https://api.github.com/repos/AAV-2025-2026/ecolo-av/commits/<sha>/check-runs
  https://api.github.com/repos/AAV-2025-2026/ecolo-av/check-runs/<id>/annotations
So the workflow runs this after a failed test step. It prints one `::error` line per
failing test case (from colcon's JUnit files in build/), and the end of the step's log
when no test case failed (tools/test.sh, lint or the build stopped first).

  python3 tools/ci_annotate.py [log] [build]      (defaults: test.log, build)

The nightly Ravensim job (`sim`) uses --sim: one annotation per FAIL line of sim-check's own
output (or its end, if none), then the errors in each launch log (Gazebo's crash, a node
that died), the last of them:

  python3 tools/ci_annotate.py --sim sim.log sim-logs/aav-sim-check.log
"""

import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

LIMIT = 9           # GitHub keeps 10 error annotations per step: the last is the log's end
MESSAGE_LINES = 25  # an annotation's message is cut at about 64 KB, but short reads better


def escape(text):
    """Escape text for a workflow command's message (GitHub's own rules)."""
    return text.replace('%', '%25').replace('\r', '%0D').replace('\n', '%0A')


def escape_property(text):
    return escape(text).replace(':', '%3A').replace(',', '%2C')


def failures(build):
    """Yield (title, message) for each failing or erroring test case under build/."""
    for path in sorted(glob.glob(os.path.join(build, '*', 'test_results', '**', '*.xml'),
                                 recursive=True)):
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as e:
            yield os.path.relpath(path, build), f'unreadable JUnit file: {e}'
            continue
        package = os.path.relpath(path, build).split(os.sep)[0]
        for case in root.iter('testcase'):
            for bad in list(case.findall('failure')) + list(case.findall('error')):
                name = case.get('name', '?')
                where = case.get('classname', '')
                body = (bad.get('message') or '') + '\n' + (bad.text or '')
                lines = [line for line in body.strip().splitlines() if line.strip()]
                # The end of a traceback says what went wrong; keep it.
                if len(lines) > MESSAGE_LINES:
                    lines = ['...'] + lines[-MESSAGE_LINES:]
                yield f'{package}: {where} {name}'.strip(), '\n'.join(lines)


def tail(log, count=40):
    try:
        with open(log, errors='replace') as f:
            return ''.join(f.readlines()[-count:])
    except OSError:
        return ''


def annotations(log, build):
    """Return the workflow command lines to print."""
    found = list(failures(build))
    out = [f'::error title={escape_property(t)}::{escape(m)}' for t, m in found[:LIMIT]]
    if len(found) > LIMIT:
        names = '\n'.join(t for t, _ in found[LIMIT:])
        out.append(f'::error title={len(found) - LIMIT} more failing tests::{escape(names)}')
    elif tail(log) or not found:
        # No failing test case: the build itself failed (23 Oct, twice). The tail alone
        # showed only "Starting >>>" lines: say which package failed and why, first.
        broken = build_errors(log)
        if broken:
            out.append(f'::error title=the build failed::{escape(broken)}')
        end = tail(log) or f'no {log}, and no failing test case in {build}/'
        out.append(f'::error title=the end of the test step::{escape(end)}')
    return out


# colcon's lines for a package that failed, and the compilers' errors.
BUILD_ERRORS = re.compile(r'^(Failed|Aborted) +<<<|^error(\[E\d+\])?:|^E: |CMake Error|'
                          r'^\S+:\d+:\d+: error|failed to (download|get|fetch|select)|'
                          r'Summary: .*failed')


def build_errors(log, count=MESSAGE_LINES):
    """Return a failed build's error lines, each with the line after it; '' if none."""
    try:
        with open(log, errors='replace') as f:
            lines = [COLOUR.sub('', line.rstrip()) for line in f]
    except OSError:
        return ''
    keep = set()
    for i, line in enumerate(lines):
        if BUILD_ERRORS.search(line):
            keep.update(range(i, min(i + 2, len(lines))))
    return '\n'.join(lines[i] for i in sorted(keep)[:count])


# A launch log's lines worth reading first: ROS's errors, Gazebo's ([Err]), a process
# that died, a crash.
ERRORS = re.compile(r'\[ERROR\]|\[Err\]|process has died|Segmentation fault|Traceback|'
                    r'Error:|error while loading')
COLOUR = re.compile(r'\x1b\[[0-9;]*m')


def error_lines(log, count=MESSAGE_LINES):
    """Return the last `count` error lines of a launch log, colours removed."""
    try:
        with open(log, errors='replace') as f:
            lines = [COLOUR.sub('', line).rstrip() for line in f]
    except OSError:
        return [f'no {log}']
    found = [line for line in lines if ERRORS.search(line)]
    return found[-count:] if found else [f'no errors in {len(lines)} lines']


def sim_annotations(log, launch_logs):
    """Return the workflow command lines for a failed sim-check."""
    try:
        with open(log, errors='replace') as f:
            failed = [line.strip() for line in f if line.startswith('FAIL ')]
    except OSError:
        failed = []
    room = LIMIT + 1 - len(launch_logs)
    out = [f'::error title=sim-check::{escape(line)}' for line in failed[:room]]
    if not failed:
        out.append(f'::error title=the end of sim-check::'
                   f'{escape(tail(log) or f"no {log}")}')
    for launch_log in launch_logs:
        title = f'{os.path.basename(launch_log)}, its last errors'
        out.append(f'::error title={escape_property(title)}::'
                   f'{escape(chr(10).join(error_lines(launch_log)))}')
    return out


def main(argv):
    if len(argv) > 1 and argv[1] == '--sim':
        log = argv[2] if len(argv) > 2 else 'sim.log'
        for line in sim_annotations(log, argv[3:]):
            print(line)
        return
    log = argv[1] if len(argv) > 1 else 'test.log'
    build = argv[2] if len(argv) > 2 else 'build'
    for line in annotations(log, build):
        print(line)


if __name__ == '__main__':
    main(sys.argv)
