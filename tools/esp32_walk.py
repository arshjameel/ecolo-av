#!/usr/bin/env python3
"""
Walk me through it: the Rust ESP32's first flash, one step at a time, shown and spoken.

`pixi run esp walk [port]` (tools/esp32.sh walk) walks the whole of docs/esp32-bench.md:

| Part | What happens |
|---|---|
| Before you start | wheels off the ground, a person at the e-stop, the RC: press Enter |
| Save the C++ | it builds the C++, finds the board, copies its flash, builds the Rust |
| Flash | "wheels off the ground, e-stop in hand": type `flash`, then it flashes |
| The first check | 30 s of the board's log; each row said aloud; SAFE TO GO ON or STOP |
| Checks, wheels off the ground | one question at a time: yes, no, skip or stop |
| Roll back | on a STOP or a "no": it puts the saved C++ back |

At the end it writes `bench-logs/<date>-esp32-first-flash.md`, and says where.

Each step is a few words on screen (bold, coloured when the terminal can), and the same
words said with macOS `say` (none if `say` isn't here, or with `--quiet`). Enter goes on;
`n` or `stop` stops. The steps and sentences are in esp32_walk_steps.py (no hardware).

    tools/esp32.sh walk                        # the first USB serial port found
    tools/esp32.sh walk /dev/cu.usbserial-0001 --voice Samantha --rate 170
    tools/esp32.sh walk --dry-run --file saved.log --quiet   # runs nothing; how tests run it
"""

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
from typing import Callable, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))
import esp32_first_check as fc  # noqa: E402
import esp32_walk_steps as ws  # noqa: E402

ROOT = fc.ROOT
COLOURS = {'ask': '\x1b[1;38;5;214m', 'go': '\x1b[1;32m', 'stop': '\x1b[1;31m'}
BOLD, DIM, RESET = '\x1b[1m', '\x1b[2m', '\x1b[0m'
MARK = {'PASS': 'go', 'FAIL': 'stop', 'WAIT': 'ask', 'WARN': 'ask', 'INFO': 'ask'}


class Speaker:
    """The Mac's `say`, one sentence at a time; silent without it."""

    def __init__(self, enabled: bool, voice: Optional[str] = None, rate: Optional[int] = None):
        self.cmd = shutil.which('say') if enabled else None
        self.opts = (['-v', voice] if voice else []) + (['-r', str(rate)] if rate else [])
        self.proc: Optional[subprocess.Popen] = None

    def say(self, text: str, wait: bool = False):
        self.hush()
        if not self.cmd:
            return
        try:
            proc = subprocess.Popen([self.cmd, *self.opts, text],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError:
            self.cmd = None
            return
        if wait:
            proc.wait()
        else:
            self.proc = proc

    def hush(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
        self.proc = None


class Io:
    """Everything the walk touches outside itself; the tests replace each part."""

    def __init__(self, speaker, colour: bool,
                 ask: Optional[Callable[[str], Optional[str]]] = None,
                 out: Callable[[str], None] = print):
        self.speaker = speaker
        self.colour = colour
        self._ask = ask or self._input
        self.out = out

    @staticmethod
    def _input(prompt: str) -> Optional[str]:
        try:
            return input(prompt)
        except EOFError:
            return None

    def ask(self, prompt: str) -> Optional[str]:
        answer = self._ask(prompt)
        self.speaker.hush()
        return answer

    def run(self, cmd: List[str]) -> int:
        try:
            return subprocess.run(cmd, cwd=ROOT).returncode
        except OSError as e:
            self.out(f'  could not run it: {e}')
            return 127

    def read_port(self, port: str, seconds: float, save: Optional[Path]) -> List[str]:
        return fc.read_port(port, seconds, save)

    def size(self, path: Path) -> Optional[int]:
        try:
            return path.stat().st_size
        except OSError:
            return None

    def now(self) -> datetime:
        return datetime.now()


def colour_ok() -> bool:
    return (sys.stdout.isatty() and os.environ.get('TERM', '') not in ('', 'dumb')
            and 'NO_COLOR' not in os.environ)


def shown(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


class Walk:
    """The steps of esp32_walk_steps, shown, said, and done in order."""

    def __init__(self, args, io: Io):
        self.a, self.io = args, io
        start = io.now()
        self.date = start.strftime('%Y-%m-%d')
        self.stamp = start.strftime('%Y-%m-%d-%H%M')
        self.out_dir = Path(args.out) if args.out else ROOT / 'bench-logs'
        self.port: Optional[str] = args.port
        self.backup: Optional[Path] = None
        self.backup_done = False
        self.flashed = False
        self.rec = ws.Record(started=start.strftime('%Y-%m-%d %H:%M'), dry_run=args.dry_run,
                             commit=args.build or fc.this_commit())

    # --- the screen and the voice -------------------------------------------------------

    def paint(self, text: str, colour: str) -> str:
        return f'{COLOURS[colour]}{text}{RESET}' if self.io.colour else text

    def dim(self, text: str) -> str:
        return f'{DIM}{text}{RESET}' if self.io.colour else text

    def banner(self, headline: str, colour: str):
        width = len(headline) + 6
        if self.io.colour:
            lines = ['┏' + '━' * width + '┓', f'┃   {headline}   ┃', '┗' + '━' * width + '┛']
        else:
            lines = ['=' * (width + 2), f'    {headline}', '=' * (width + 2)]
        self.io.out('')
        for line in lines:
            self.io.out('  ' + self.paint(line, colour))

    def tell(self, text: str, colour: str = 'ask', wait: bool = False,
             spoken: Optional[str] = None):
        """One sentence on screen, and aloud (`spoken` when the screen's has a path)."""
        self.io.out('  ' + (self.paint(text, colour) if colour != 'ask' else text))
        self.io.speaker.say(spoken or text, wait=wait)

    def show(self, step: ws.Step, label: str):
        self.io.out('')
        self.io.out('  ' + self.dim(f'{label}  ·  {step.section}'))
        self.banner(step.headline, step.colour)
        self.io.out('')
        self.tell(step.say)
        if step.detail:
            self.io.out('  ' + self.dim(step.detail))
        self.io.out('')

    def now(self) -> str:
        return self.io.now().strftime('%H:%M:%S')

    # --- doing one step -----------------------------------------------------------------

    def run(self, cmd: List[str]) -> int:
        line = ' '.join(shlex.quote(c) for c in cmd)
        if self.a.dry_run:
            self.io.out('  ' + self.paint(f'Would run: {line}', 'ask'))
            return 0
        self.io.out('  ' + self.dim(f'Running: {line}'))
        return self.io.run(cmd)

    def until_ok(self, step: ws.Step, attempt) -> str:
        """Try; on a failure say so, and try again on Enter, or stop."""
        while True:
            ok, note = attempt()
            if ok:
                self.io.out('  ' + self.paint(f'Done. {note}'.strip(), 'go'))
                self.rec.add(step, 'would run' if self.a.dry_run else 'done', note, self.now())
                return 'go'
            self.tell(f'That did not work. {note}'.strip(), 'stop')
            self.tell('Press Enter to try again, or type stop.')
            if ws.answer_go(self.io.ask('  > ')) == 'stop':
                self.rec.add(step, 'failed, stopped', note, self.now())
                return 'stop'

    def do(self, step: ws.Step, label: str) -> str:
        """'go' on, 'stop' (the person stopped), or 'fail' (a check said no)."""
        self.show(step, label)
        kind = step.kind
        if kind == 'do':
            answer = ws.answer_go(self.io.ask('  Enter when done.  stop to stop.  > '))
            self.rec.add(step, 'done' if answer == 'go' else 'stopped', at=self.now())
            return answer
        if kind == 'ask':
            return self.question(step)
        if kind == 'flash':
            return self.flash(step)
        if kind == 'check':
            return self.first_check(step)
        return self.until_ok(step, getattr(self, 'step_' + step.key.replace('-', '_')))

    def question(self, step: ws.Step) -> str:
        while True:
            answer = ws.answer_yes_no(self.io.ask('  yes, no, skip, or stop?  > '))
            if answer is not None:
                break
            self.tell('Yes or no?')
        self.rec.add(step, {'yes': 'yes', 'no': 'NO', 'skip': 'skipped',
                            'stop': 'stopped'}[answer], at=self.now())
        if answer == 'no':
            self.tell(f'{step.headline}: no. Stop here.', 'stop')
            return 'fail'
        return 'stop' if answer == 'stop' else 'go'

    def flash(self, step: ws.Step) -> str:
        restoring = step.key == 'rb-flash'
        if not restoring and not self.backup_done and not self.a.dry_run:
            self.tell('There is no copy of the C++. I will not flash.', 'stop')
            self.rec.add(step, 'refused', 'no copy of the C++', self.now())
            return 'fail'
        while True:
            answer = ws.answer_word(self.io.ask(f'  Type {ws.FLASH_WORD}, or stop.  > '))
            if answer == 'stop':
                self.rec.add(step, 'stopped', 'not flashed', self.now())
                return 'stop'
            if answer == 'go':
                break
            self.tell(f'Not flashed. Type the word {ws.FLASH_WORD} to flash.')
        if restoring:
            cmd = ws.restore_command(self.port, shown(self.backup) if self.backup else None)
        else:
            self.flashed = True     # from here, the board may hold the Rust
            cmd = ws.flash_command(self.port)
        self.tell('Flashing. Do not unplug the cable.', 'stop')
        return self.until_ok(step, lambda: (
            self.run(cmd) == 0,
            '' if restoring else 'If it will not connect: hold the BOOT button.'))

    # --- the run steps ------------------------------------------------------------------

    def step_cpp_builds(self):
        return self.run(ws.cpp_build_command()) == 0, ''

    def step_port(self):
        if self.run(ws.list_ports_command()) != 0 and not self.a.dry_run:
            return False, 'No espflash? Run pixi run esp tools first.'
        self.port = self.port or fc.default_port()
        self.rec.port = self.port
        if self.port:
            return True, f'The board is on {self.port}.'
        if self.a.dry_run:
            return True, 'No board here: a dry run goes on.'
        return False, 'No board found. Check the cable: some carry only power.'

    def step_backup(self):
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.backup = self.out_dir / f'{self.stamp}-backup-esp32.bin'
        self.rec.backup = shown(self.backup)
        if self.run(ws.backup_command(self.port, shown(self.backup))) != 0:
            return False, 'Hold the BOOT button while it starts.'
        if self.a.dry_run:
            self.backup_done = True
            return True, f'The copy goes to {shown(self.backup)}.'
        size = self.io.size(self.backup)
        if not ws.backup_ok(size):
            return False, f'The copy is {size} bytes, not {ws.FLASH_SIZE}.'
        self.backup_done = True
        return True, f'The copy is {shown(self.backup)}, 4 MB. Keep it off git.'

    def step_build(self):
        return self.run(ws.build_command()) == 0, ''

    def step_rb_look(self):
        if self.a.dry_run or self.a.file or not self.port:
            self.io.out('  ' + self.paint(f'Would listen for: {ws.CPP_BANNER}', 'ask'))
            return True, ''
        lines = self.io.read_port(self.port, 10.0, None)
        if any(ws.CPP_BANNER in line for line in lines):
            return True, 'The C++ is back.'
        return False, "Its first line wasn't seen. Press the board's reset button."

    # --- the first check ----------------------------------------------------------------

    def first_check(self, step: ws.Step) -> str:
        if self.a.file:
            lines = Path(self.a.file).read_text(errors='replace').splitlines()
            source = f'the saved log {self.a.file}'
        elif self.a.dry_run:
            self.io.out('  ' + self.paint(
                f'Would read {self.port or "the board"} for {self.a.seconds:.0f} s.', 'ask'))
            self.rec.add(step, 'not run', 'a dry run with no --file', self.now())
            return 'go'
        else:
            save = self.out_dir / f'{self.stamp}-first-check.log'
            try:
                lines = self.io.read_port(self.port, self.a.seconds, save)
            except OSError as e:
                self.tell(f'I could not read the board. {e}', 'stop')
                self.rec.add(step, 'not read', str(e), self.now())
                return 'fail'
            source = shown(save)
        rows = fc.judge(fc.parse(lines), self.rec.commit, self.a.features, self.a.pi)
        verdict = fc.verdict_line(rows)
        settings = ws.settings_from_lines(lines)
        self.rec.check_table, self.rec.check_verdict = fc.table(rows), verdict
        self.rec.settings = settings

        self.io.out('  ' + self.dim(f'Read from {source}.'))
        self.io.out('')
        self.io.out('  ' + self.paint("The board's settings", 'ask'))
        for line in ws.settings_table(settings).splitlines():
            self.io.out('    ' + line)
        self.io.out('')
        for sentence in ws.settings_sentences(settings):
            self.tell(sentence, wait=True)
        self.io.out('')
        for line in fc.table(rows).splitlines():
            self.io.out('    ' + line)
        self.io.out('')
        for row in rows:
            self.tell(ws.row_sentence(row), MARK.get(row.result, 'ask'), wait=True)
        safe, sentence = ws.verdict_sentence(rows)
        waiting = not safe and verdict.startswith('NOT YET')
        self.banner(verdict.split(':')[0] if waiting else verdict,
                    'go' if safe else 'ask' if waiting else 'stop')
        self.io.out('')
        self.tell(sentence, 'go' if safe else 'ask' if waiting else 'stop', wait=True)
        self.rec.add(step, 'SAFE TO GO ON' if safe else verdict, source, self.now())
        if waiting and not self.a.file:
            # Only the RC is lost: the board is braked and the firmware is fine. Rolling
            # back would throw a good flash away (21 Nov), so read again once it is set.
            self.tell('Transmitter on, CH6 low at about 1100, the left stick in the middle. '
                      'Enter to check again, or stop.')
            if ws.answer_go(self.io.ask('  Enter to check again.  stop to stop.  > ')) == 'go':
                return self.first_check(step)
            return 'stop'
        return 'go' if safe else 'stop' if waiting else 'fail'

    # --- the whole walk -----------------------------------------------------------------

    def roll_back(self):
        self.tell('Roll back to the C++.', 'stop')
        n = len(ws.ROLL_BACK)
        for i, step in enumerate(ws.ROLL_BACK, 1):
            if self.do(step, f'Roll back {i} of {n}') != 'go':
                self.rec.outcome = 'stopped during the roll back: the board may hold the Rust'
                return
        self.rec.outcome = 'rolled back to the C++'
        self.tell('Rolled back. The C++ is on the board.', 'go')

    def offer_roll_back(self):
        self.tell('The Rust is on the board. Roll back to the C++ now? Yes or no.')
        if ws.answer_yes_no(self.io.ask('  yes or no?  > ')) == 'yes':
            self.roll_back()
        else:
            self.rec.outcome = 'stopped: the Rust stays on the board'
            self.tell('The Rust stays on the board.')

    def walk(self) -> int:
        total = len(ws.MAIN)
        if self.a.dry_run:
            self.io.out('  ' + self.paint('A dry run: nothing is run, nothing flashed.', 'ask'))
        result = 'go'
        for i, step in enumerate(ws.MAIN, 1):
            result = self.do(step, f'Step {i} of {total}')
            if result != 'go':
                break
        if result == 'go':
            self.rec.outcome = 'all done: every check passed or skipped'
            self.banner('All done', 'go')
            self.tell('All done. Next: the same on the ground, at walking speed, with Ben.',
                      'go', wait=True)
        elif not self.flashed:
            self.rec.outcome = ('stopped before the flash: the C++ is still on the board'
                                if result == 'stop' else 'failed before the flash: the C++ '
                                'is still on the board')
            self.tell('Stopped. Nothing was flashed. The C++ is still on the board.', 'stop')
        elif result == 'fail':
            self.roll_back()
        else:
            self.offer_roll_back()
        return 0 if result == 'go' else 1

    def save(self) -> Path:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        base = f'{self.date}-esp32-first-flash' + ('-dry-run' if self.a.dry_run else '')
        path = self.out_dir / f'{base}.md'
        n = 2
        while path.exists():
            path = self.out_dir / f'{base}-{n}.md'
            n += 1
        path.write_text(ws.record_markdown(self.rec))
        return path


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n\n')[0])
    ap.add_argument('port', nargs='?', help="the board's serial port (default: the first "
                    'USB serial port found)')
    ap.add_argument('--dry-run', action='store_true',
                    help='walk every step, but run nothing: print what it would run')
    ap.add_argument('--file', help='the first check reads this saved log, not the port')
    ap.add_argument('--quiet', action='store_true', help='no speech')
    ap.add_argument('--voice', help="say's voice (say -v '?' lists them)")
    ap.add_argument('--rate', type=int, help="say's words a minute (about 175 by default)")
    ap.add_argument('--plain', action='store_true', help='no colours')
    ap.add_argument('--out', help='where the record goes (default: bench-logs/)')
    ap.add_argument('--seconds', type=float, default=fc.SECONDS,
                    help='how long the first check reads the port')
    ap.add_argument('--build', help="the commit expected (default: this checkout's)")
    ap.add_argument('--features', help='the exact features expected (as first-check)')
    ap.add_argument('--pi', choices=('any', 'on', 'off'), default='any',
                    help="the first check: is the Pi's bridge sending?")
    return ap


def main(argv=None, io: Optional[Io] = None) -> int:
    args = parser().parse_args(argv)
    if io is None:
        io = Io(Speaker(not args.quiet, args.voice, args.rate),
                colour=colour_ok() and not args.plain)
    w = Walk(args, io)
    try:
        code = w.walk()
    except KeyboardInterrupt:
        io.speaker.hush()
        io.out('')
        w.rec.outcome = w.rec.outcome or ('stopped (Ctrl-C)' + (
            ': the Rust may be on the board' if w.flashed else ': nothing was flashed'))
        code = 1
    path = w.save()
    io.out('')
    try:
        w.tell(f'The record is in {shown(path)}.', 'go', wait=True,
               spoken='The record is in the bench logs folder.')
    except KeyboardInterrupt:
        # A second Ctrl-C, during that last sentence: the record is saved; just be quiet.
        io.speaker.hush()
    return code


if __name__ == '__main__':
    sys.exit(main())
