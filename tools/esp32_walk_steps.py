"""
The guided first flash's logic: the steps, the answers, and the sentences to speak.

`tools/esp32_walk.py` (`pixi run esp walk`) shows and says these one at a time. This
module touches no hardware: no serial port, no speech, no command run. Its tests feed it
answers (tools/tests/test_esp32_walk.py).

| Part | What |
|---|---|
| `MAIN`, `ROLL_BACK` | the steps of docs/esp32-bench.md, in order: a few words each |
| `answer_*` | what the person typed: go, stop, yes, no, skip, the word `flash` |
| `row_sentence` | one first-check row as a plain spoken sentence |
| `settings_from_lines` | the board's CONFIG line, any keys, read generically |
| `record` | the written record, Markdown, for `bench-logs/` |
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import re
from typing import Dict, List, Optional, Sequence, Tuple

ESPFLASH = 'vendor/cargo-tools/bin/espflash'
ELF = 'firmware/spine/boards/esp32-umod/target/xtensa-esp32-none-elf/release/spine-esp32-umod'
FLASH_SIZE = 0x400000           # 4 MB: the whole flash
FLASH_WORD = 'flash'
CPP_BANNER = '=== ESP32: AAV Controller ==='


@dataclass(frozen=True)
class Step:
    """One thing to do, said in a few words."""

    key: str
    section: str
    headline: str       # bold, on screen: a few words
    say: str            # spoken: one or two short sentences
    detail: str = ''    # on screen only, smaller: what to look for
    kind: str = 'do'    # do (Enter), run, flash (type the word), check, ask (yes/no)
    colour: str = 'ask'     # ask (amber), go (green), stop (red)


BEFORE = 'Before you start'
SAVE = 'Save the C++'
FLASH = 'Flash'
GATE = 'The first check'
HANDS = 'Checks, wheels off the ground'

MAIN: Tuple[Step, ...] = (
    Step('wheels', BEFORE, 'Wheels off the ground',
         'Put the car on stands. All four wheels off the ground.'),
    Step('estop', BEFORE, 'A person at the e-stop',
         'A second person stands at the red e-stop button.',
         'It cuts the power by itself, whatever the program does.'),
    Step('rc', BEFORE, 'RC on and set up, override off',
         'Turn the RC transmitter on. The override switch off. The left stick in the middle.',
         'Its failsafe and End points must be set first: car day, checks T1 to T4. '
         'CH6 low must read about 1100, or the Rust reads a lost RC and holds its brake.'),
    Step('bridge-off', BEFORE, "The Pi's bridge off",
         "Turn the Motor Pi's bridge off for now.",
         'The first check wants it off: then NO FRAMES is right.'),
    Step('usb', BEFORE, 'Plug in the USB cable',
         "Plug the laptop's USB cable into the ESP32 board."),
    Step('cpp-builds', SAVE, 'Does the C++ still build?',
         'I check the old C++ still builds. It is the way back.',
         'pixi run -e firmware legacy-esp32', kind='run'),
    Step('port', SAVE, 'Find the board',
         'I look for the board on the USB cable.',
         'espflash list-ports', kind='run'),
    Step('backup', SAVE, 'Copy the C++ off the board',
         'I copy what is on the board now. About a minute.',
         'If it will not connect: hold the BOOT button while it starts.', kind='run'),
    Step('build', SAVE, 'Build the Rust',
         'I build the Rust program.', 'pixi run esp build', kind='run'),
    Step('flash', FLASH, 'Wheels off the ground, e-stop in hand',
         'Wheels off the ground. E-stop in hand. Type the word flash, then Enter.',
         'This writes over the C++. The copy is saved.', kind='flash', colour='stop'),
    Step('first-check', GATE, 'The board checks itself',
         'Now the board checks itself. Thirty seconds. Do not touch anything.',
         'It only reads the log. It sends nothing.', kind='check'),
    Step('h1', HANDS, 'Power on, nothing else',
         'Power on, nothing else. Is the brake on, and the steering straight?',
         'Brake 3500, steering 1950, DAC 0 V on the multimeter, status LED blinks fast.',
         kind='ask'),
    Step('h2', HANDS, "Unplug a Jrk's I2C, then back",
         "Unplug a Jrk's I2C wire, then plug it back. Did the red light come on while "
         'it was out?', 'Fault 8 (steering) or 16 (brake) while it is out.', kind='ask'),
    Step('h3', HANDS, "Start the Pi's bridge",
         "Start the Motor Pi's bridge, frame version 2. Does the brake stay on?",
         'LINK: frames= rising, bad=0. HEALTH link=ok with the Orin, counter_frozen '
         'without.', kind='ask'),
    Step('h4', HANDS, 'Drive slowly from ROS',
         'Drive slowly from ROS. Do the wheels turn forward?',
         '0.3 m/s. mode=Computer, green LED, the brake lets go, the DAC rises smoothly.',
         kind='ask'),
    Step('h5', HANDS, 'Steer left from ROS',
         'Steer left from ROS. Do the wheels turn left?',
         'If they turn right: the steering sign in legacy.yaml (Q2).', kind='ask'),
    Step('h6', HANDS, 'Stop from ROS',
         'Stop from ROS. Is the throttle zero, and the brake on?',
         "The bridge's hold brake: target 1050.", kind='ask'),
    Step('h7', HANDS, 'Reverse slowly',
         'Reverse slowly. Do the wheels turn backwards?',
         'The direction changes only with the DAC at 0.', kind='ask'),
    Step('h8', HANDS, 'Stop the bridge while driving',
         'While driving, stop the bridge with Control C. Did the brake go on at once?',
         'Within 0.2 s: throttle 0, brake full, faults has 1.', kind='ask'),
    Step('h9', HANDS, 'Start the bridge again',
         'Start the bridge again. Does the brake stay on until a stop?',
         'Braked until ego stands still, then it drives.', kind='ask'),
    Step('h9a', HANDS, "Stop the Orin's output mux",
         "While driving, stop the Orin's output mux. Did the brake go on?",
         'The bridge still running. Within 0.2 s; repeats= rises. The heartbeat.',
         kind='ask'),
    Step('h9b', HANDS, 'Bring the Orin back',
         'Bring the Orin back. Does it stay braked until ego stands still?', kind='ask'),
    Step('h9c', HANDS, 'The e-stop from ROS',
         'While driving, send the e-stop from ROS. Did the brake go on at once?',
         'pixi run estop. estop=true, brake full.', kind='ask'),
    Step('h10', HANDS, 'RC override on',
         'RC override switch on, a little throttle. Do the wheels turn?',
         'mode=Rc, blue LED. ROS commands ignored.', kind='ask'),
    Step('h11', HANDS, 'RC transmitter off',
         'Switch the RC transmitter off. Did the brake go on?',
         'Within 0.1 s: brake full, faults has 256.', kind='ask'),
    Step('h12', HANDS, 'RC override off',
         'RC override off. Is it back to the computer, or Manual?', kind='ask'),
    Step('h13', HANDS, 'The red e-stop button',
         'Press the red e-stop now. Did the car stop by itself?', kind='ask'),
    Step('h14', HANDS, 'Ten minutes, mixed',
         'Mix checks 1 to 13 for ten minutes. Did it run with no restart?',
         '"the last reset:" stays the power-on one.', kind='ask'),
)

ROLL_BACK: Tuple[Step, ...] = (
    Step('rb-estop', 'Roll back', 'Press the e-stop',
         "Press the e-stop. Turn the Motor Pi's bridge off.", colour='stop'),
    Step('rb-flash', 'Roll back', 'Put the C++ back',
         'I put the saved C++ back on the board. Type the word flash, then Enter.',
         'It writes the copy made before the flash.', kind='flash', colour='stop'),
    Step('rb-look', 'Roll back', 'Is the C++ back?',
         "I listen for the C++'s first line.", CPP_BANNER, kind='run'),
    Step('rb-frame', 'Roll back', 'Frame version 1 on the Pi',
         'On the Motor Pi, set frame version 1 in legacy.yaml. The C++ reads only that.',
         'Then ~/aav-jazzy/aav.sh build.'),
)


# --- The commands -----------------------------------------------------------------------

def port_args(port: Optional[str]) -> List[str]:
    return ['--port', port] if port else []


def cpp_build_command() -> List[str]:
    return ['pixi', 'run', '-e', 'firmware', 'legacy-esp32']


def list_ports_command() -> List[str]:
    return [ESPFLASH, 'list-ports']


def backup_command(port: Optional[str], backup: str) -> List[str]:
    return [ESPFLASH, 'read-flash', *port_args(port), '0', hex(FLASH_SIZE), backup]


def build_command() -> List[str]:
    return ['tools/esp32.sh', 'build']


def flash_command(port: Optional[str]) -> List[str]:
    """Flash the Rust, with no monitor after: the first check reads the log instead."""
    return [ESPFLASH, 'flash', '--chip', 'esp32', *port_args(port), ELF]


def restore_command(port: Optional[str], backup: Optional[str]) -> List[str]:
    """Put the C++ back: the saved copy, or else the C++ built from source."""
    if backup:
        return [ESPFLASH, 'write-bin', *port_args(port), '0', backup]
    return ['pixi', 'run', '-e', 'firmware', 'legacy-esp32', '-t', 'upload']


def backup_ok(size: Optional[int]) -> bool:
    return size == FLASH_SIZE


# --- The answers ------------------------------------------------------------------------

STOP_WORDS = {'n', 'no', 'stop', 'q', 'quit', 'exit'}
YES = {'y', 'yes', 'yeah', 'yep', 'ok', 'pass'}
NO = {'n', 'no', 'nope', 'fail'}
SKIP = {'s', 'skip', 'later'}


def _clean(text: Optional[str]) -> Optional[str]:
    return None if text is None else text.strip().lower().rstrip('.!')


def answer_go(text: Optional[str]) -> str:
    """Enter (or anything else) goes on; `n` or `stop` stops. No more input stops."""
    t = _clean(text)
    return 'stop' if t is None or t in STOP_WORDS else 'go'


def answer_yes_no(text: Optional[str]) -> Optional[str]:
    """Yes, no, skip or stop; None when it isn't clear, so ask again."""
    t = _clean(text)
    if t is None or t in {'stop', 'q', 'quit', 'exit'}:
        return 'stop'
    for word, words in (('yes', YES), ('no', NO), ('skip', SKIP)):
        if t in words:
            return word
    return None


def answer_word(text: Optional[str], word: str = FLASH_WORD) -> Optional[str]:
    """Go on the exact word; stop on stop; None (ask again) on anything else."""
    t = _clean(text)
    if t is None or t in STOP_WORDS:
        return 'stop'
    return 'go' if t == word else None


# --- The first check, in sentences ------------------------------------------------------

RESULT_WORDS = {'PASS': 'pass', 'FAIL': 'fail', 'WARN': 'warning', 'INFO': 'note'}

SENTENCES: Dict[str, Dict[str, str]] = {
    'Log': {'PASS': 'The board is talking.',
            'FAIL': 'Nothing came from the board. Check the cable and the port.'},
    'Build': {'PASS': 'It is the build you just made.',
              'FAIL': 'It is a different build. Flash it again.',
              'INFO': 'There was no commit to compare with.'},
    'Features': {'PASS': 'The features are right.',
                 'FAIL': 'The features are wrong. Build the right one.'},
    'Last reset': {'PASS': 'It started from power-on.',
                   'FAIL': 'It reset by itself.'},
    'Devices': {'PASS': 'All four devices answered.'},
    'Status lines': {'FAIL': 'No status lines came. Is it the right speed?'},
    'Brake': {'PASS': 'The brake is on.',
              'FAIL': 'The brake is not where it should be.'},
    'Throttle': {'PASS': 'The throttle is zero.',
                 'FAIL': 'The throttle is on. Press the e-stop now.'},
    'Steering': {'PASS': 'The steering is centred.',
                 'FAIL': 'The steering is not centred.'},
    'Mode': {'PASS': 'It is stopped, and not armed.',
             'WARN': "The Pi's frames armed it. Turn the bridge off for this check.",
             'FAIL': 'It is not stopped.'},
    'Faults': {'PASS': 'No unexpected faults.',
               'FAIL': 'There is an unexpected fault.'},
    'Tick': {'PASS': 'The timing is good.',
             'FAIL': 'The timing is too slow.'},
    'Late ticks': {'INFO': 'A few ticks were late. Only a note.',
                   'WARN': 'Some ticks were late.'},
    'Restarts': {'PASS': 'No restarts.',
                 'FAIL': 'It restarted by itself.'},
}

LINK_WORDS = {'OK': 'the link is OK', 'NO FRAMES': 'no frames',
              'STALE': 'the frames stopped', 'COUNTER FROZEN': 'the counter is frozen',
              'BAD FRAMES': 'bad frames'}


def row_sentence(row) -> str:
    """One first-check row, said plainly: "Brake: pass. The brake is on."."""
    check, result, detail = row.check, row.result, row.detail
    head = f'{check}: {RESULT_WORDS.get(result, result.lower())}.'
    if check == 'Devices' and result != 'PASS':
        missing = re.findall(r'(\w+) 0x[0-9a-f]+ MISSING', detail)
        if missing:
            return f"{head} Missing: {', '.join(missing)}."
        return f'{head} Not all four answered.'
    if check == 'Faults' and result == 'PASS' and 'RC lost' in detail:
        return f'{head} Only the RC: the transmitter is off.'
    if check == 'Pi link':
        word = detail.split(':')[0].strip()
        said = LINK_WORDS.get(word, word.lower())
        tail = ' That is right with the bridge off.' if word == 'NO FRAMES' else ''
        return f'{head} {said[0].upper()}{said[1:]}.{tail}'
    sentence = SENTENCES.get(check, {}).get(result)
    return f'{head} {sentence}' if sentence else head


def verdict_sentence(rows) -> Tuple[bool, str]:
    """(safe, the sentence): the same verdict as esp32_first_check's last line."""
    failed = next((r.check for r in rows if r.result == 'FAIL'), None)
    if failed:
        return False, f'Stop. {failed} failed. We roll back now.'
    if any(r.result == 'WAIT' for r in rows):
        return False, ('Not yet. The RC is lost, so the board holds its brake. Nothing is '
                       'wrong with the board: no roll back. Set the transmitter, then we '
                       'check again.')
    return True, 'Safe to go on.'


# --- The board's settings, from its CONFIG line -----------------------------------------

FIELD = re.compile(r'(\w+)=(\S+)')
ANSI = re.compile(r'\x1b\[[0-9;]*m')
# Said on screen but not aloud: names, versions, addresses.
NOT_SPOKEN = {'t', 'build', 'features', 'version', 'link', 'frame', 'last_reset'}
# The ones worth saying, in this order: whichever keys the line has.
GROUPS = (('Steering', ('steer',)), ('Speed', ('speed',)), ('Brake', ('brake',)),
          ('Throttle', ('throttle', 'dac_')), ('Rates', ('_ms', '_hz', '_khz', 'rate')))
UNITS = (('_ms', 'milliseconds'), ('_us', 'microseconds'), ('_khz', 'kilohertz'),
         ('_hz', 'hertz'), ('_mps', 'metres a second'), ('_m_s', 'metres a second'))


def _flatten(d: dict, prefix: str = '') -> Dict[str, str]:
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(_flatten(v, f'{prefix}{k}_'))
        elif not isinstance(v, list):
            out[f'{prefix}{k}'] = str(v).lower() if isinstance(v, bool) else str(v)
    return out


def settings_from_lines(lines: Sequence[str]) -> Dict[str, str]:
    """Read the last CONFIG line's key=value pairs (text or JSON form), whatever keys."""
    found: Dict[str, str] = {}
    for raw in lines:
        text = ANSI.sub('', raw).strip()
        brace = text.find('{"t":')
        if brace >= 0:
            try:
                obj = json.loads(text[brace:])
            except ValueError:
                obj = None
            if isinstance(obj, dict) and obj.get('t') == 'config':
                found = {k: v for k, v in _flatten(obj).items()
                         if k not in ('t', 'schema', 'up_ms', 'seq')}
            continue
        at = text.find('CONFIG ')
        if at >= 0 and ' t=' in text[at:at + 12]:
            found = dict(FIELD.findall(text[at + len('CONFIG'):]))
            found.pop('t', None)
    return found


SAY_AS = {'cmd': 'command', 'i2c': 'I2C', 'dac': 'DAC', 'rc': 'RC', 'jrk': 'Jrk'}


def _name(key: str, drop: Sequence[str] = ()) -> Tuple[str, str]:
    """
    Say a key's name and unit: tick_ms -> ('tick', 'milliseconds').

    The group's own word is left out ("Steering: min 400", not "steer min").
    """
    unit = ''
    for suffix, said in UNITS:
        if key.endswith(suffix):
            key, unit = key[:-len(suffix)], said
            break
    else:
        if 'speed' in key:
            unit = 'metres a second'
    words = key.split('_')
    kept = [w for w in words if not any(w.startswith(d) for d in drop)] or words
    return ' '.join(SAY_AS.get(w, w) for w in kept), unit


def _spoken_value(value: str) -> Optional[str]:
    if value.lower().startswith('0x'):
        return None             # an address: not worth saying
    try:
        number = float(value)
    except ValueError:
        return None
    return f'{number:g}'


def settings_groups(settings: Dict[str, str]) -> List[Tuple[str, List[str]]]:
    """Group the important settings, said plainly; each key in one group at most."""
    used, out = set(), []
    for title, needles in GROUPS:
        parts = []
        for key, value in settings.items():
            if (key in used or key in NOT_SPOKEN or 'address' in key
                    or not any(n in key for n in needles)):
                continue
            said = _spoken_value(value)
            if said is None:
                continue
            used.add(key)
            name, unit = _name(key, [n for n in needles if '_' not in n])
            parts.append(f'{name} {said}' + (f' {unit}' if unit else ''))
        if parts:
            out.append((title, parts))
    return out


def settings_sentences(settings: Dict[str, str]) -> List[str]:
    return [f"{title}: {', '.join(parts)}." for title, parts in settings_groups(settings)]


def settings_table(settings: Dict[str, str]) -> str:
    if not settings:
        return '(no CONFIG line: an older build?)'
    width = max(len(k) for k in settings)
    return '\n'.join(f'{k:<{width}}  {v}' for k, v in settings.items())


# --- The written record -----------------------------------------------------------------

@dataclass
class Entry:
    """One step as it went: what was asked, and what happened."""

    step: str
    section: str
    result: str         # done, yes, no, skipped, stopped, ran, failed, would run...
    note: str = ''
    at: str = ''


@dataclass
class Record:
    """Everything the walk saw, for bench-logs/."""

    started: str
    dry_run: bool = False
    port: Optional[str] = None
    commit: Optional[str] = None
    backup: Optional[str] = None
    entries: List[Entry] = field(default_factory=list)
    check_table: str = ''
    check_verdict: str = ''
    settings: Dict[str, str] = field(default_factory=dict)
    outcome: str = ''

    def add(self, step: Step, result: str, note: str = '', at: str = ''):
        self.entries.append(Entry(step.headline, step.section, result, note, at))


def record_markdown(r: Record) -> str:
    def cell(s):
        return str(s).replace('|', '\\|').replace('\n', ' ')

    title = 'ESP32 first flash' + (' (dry run: nothing was run)' if r.dry_run else '')
    out = [f'# {title}', '',
           '| | |', '|---|---|',
           f'| Started | {r.started} |',
           f"| Outcome | **{cell(r.outcome or 'not finished')}** |",
           f"| Port | {cell(r.port or 'not found')} |",
           f"| This checkout | {cell(r.commit or 'unknown')} |",
           f"| The C++'s copy | {cell(r.backup or 'none')} (holds a WiFi password: "
           'never in git) |', '',
           '## Every step', '',
           '| # | When | Section | Step | Result | Note |', '|---|---|---|---|---|---|']
    out += [f'| {i} | {e.at} | {cell(e.section)} | {cell(e.step)} | {cell(e.result)} | '
            f'{cell(e.note)} |' for i, e in enumerate(r.entries, 1)]
    out += ['', '## The first check', '']
    if r.check_table:
        out += ['```text', r.check_table, '', r.check_verdict, '```']
    else:
        out += ['Not run.']
    out += ['', "## The board's settings (its CONFIG line)", '']
    if r.settings:
        out += ['| Key | Value |', '|---|---|']
        out += [f'| {cell(k)} | {cell(v)} |' for k, v in r.settings.items()]
    else:
        out += ['Not seen.']
    return '\n'.join(out) + '\n'
