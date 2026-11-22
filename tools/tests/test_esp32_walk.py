"""
Tests for the guided first flash (tools/esp32_walk.py, tools/esp32_walk_steps.py).

Answers are fed in as a list; nothing is spoken (a fake speaker), no serial port is
opened, and no command runs (a fake runner, or --dry-run).
"""

from datetime import datetime
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import esp32_first_check as fc  # noqa: E402
import esp32_walk as walk  # noqa: E402
import esp32_walk_steps as ws  # noqa: E402
import pytest  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
TEXT_LOG = HERE / 'esp32_first_check_qemu_text.txt'
JSON_LOG = HERE / 'esp32_first_check_qemu_json.txt'
QEMU = ['--build', '216ca6b', '--features', 'legacy-uart,qemu,uplink']
HANDS = sum(1 for s in ws.MAIN if s.kind == 'ask')
BEFORE_FLASH = sum(1 for s in ws.MAIN if s.kind == 'do')


class FakeSpeaker:
    def __init__(self):
        self.said = []

    def say(self, text, wait=False):
        self.said.append(text)

    def hush(self):
        pass


class FakeIo(walk.Io):
    """Answers from a list (None when it runs out, as at the end of stdin)."""

    def __init__(self, answers, codes=None, size=ws.FLASH_SIZE, port_lines=None):
        self.answers = list(answers)
        self.lines = []
        self.runs = []
        self.codes = codes or {}
        self._size = size
        self.port_lines = port_lines
        self.port_reads = []
        super().__init__(FakeSpeaker(), colour=False, ask=self._next, out=self.lines.append)

    def _next(self, prompt):
        self.lines.append(prompt)
        return self.answers.pop(0) if self.answers else None

    def run(self, cmd):
        self.runs.append(cmd)
        for word, code in self.codes.items():
            if word in cmd:
                return code
        return 0

    def read_port(self, port, seconds, save):
        if self.port_lines is None:
            raise AssertionError('a test must never read a real port')
        self.port_reads.append((port, seconds))
        return self.port_lines

    def size(self, path):
        return self._size

    def now(self):
        return datetime(2026, 11, 7, 14, 30)

    @property
    def said(self):
        return self.speaker.said

    @property
    def screen(self):
        return '\n'.join(self.lines)


def run_walk(tmp_path, answers, *extra, **io_kw):
    io = FakeIo(answers, **io_kw)
    code = walk.main(['--out', str(tmp_path), *extra], io=io)
    records = sorted(tmp_path.glob('*-esp32-first-flash*.md'))
    return code, io, records


def all_yes():
    return [''] * BEFORE_FLASH + ['flash'] + ['yes'] * HANDS


# --- The answers ------------------------------------------------------------------------

@pytest.mark.parametrize('text, want', [
    ('', 'go'), ('  ', 'go'), ('done', 'go'), ('n', 'stop'), ('STOP', 'stop'),
    ('stop.', 'stop'), (None, 'stop')])
def test_enter_goes_on_and_n_or_stop_stops(text, want):
    assert ws.answer_go(text) == want


@pytest.mark.parametrize('text, want', [
    ('y', 'yes'), ('Yes', 'yes'), ('n', 'no'), ('NO', 'no'), ('skip', 'skip'),
    ('stop', 'stop'), (None, 'stop'), ('', None), ('maybe', None)])
def test_yes_no_skip_stop_or_ask_again(text, want):
    assert ws.answer_yes_no(text) == want


@pytest.mark.parametrize('text, want', [
    ('flash', 'go'), (' Flash ', 'go'), ('', None), ('y', None), ('flsh', None),
    ('yes', None), ('stop', 'stop'), ('n', 'stop'), (None, 'stop')])
def test_only_the_word_flash_flashes(text, want):
    assert ws.answer_word(text) == want


# --- The first check, in sentences ------------------------------------------------------

def test_a_row_is_a_plain_sentence():
    assert ws.row_sentence(fc.Row('Brake', 'PASS', '1.00 (IDLE_BRAKE 1.0)')) == \
        'Brake: pass. The brake is on.'
    assert ws.row_sentence(fc.Row('Throttle', 'FAIL', 'NOT 0')) == \
        'Throttle: fail. The throttle is on. Press the e-stop now.'


def test_a_missing_device_is_named():
    row = fc.Row('Devices', 'FAIL', '3/4: steering 0x05 answered; dac 0x0c MISSING')
    assert ws.row_sentence(row) == 'Devices: fail. Missing: dac.'


def test_the_pi_link_says_its_verdict():
    row = fc.Row('Pi link', 'INFO', "NO FRAMES: right while the Pi's bridge is off")
    assert ws.row_sentence(row) == 'Pi link: note. No frames. That is right with the bridge off.'
    row = fc.Row('Pi link', 'FAIL', "COUNTER FROZEN: is the Orin's output mux running?")
    assert ws.row_sentence(row) == 'Pi link: fail. The counter is frozen.'


def test_a_check_it_does_not_know_still_gets_a_sentence():
    assert ws.row_sentence(fc.Row('Something new', 'WARN', 'x')) == 'Something new: warning.'


@pytest.mark.parametrize('log', [TEXT_LOG, JSON_LOG])
def test_every_row_of_a_real_log_is_spoken(log):
    rows = fc.judge(fc.parse(log.read_text().splitlines()), None, None, 'any')
    for row in rows:
        sentence = ws.row_sentence(row)
        assert sentence.startswith(f'{row.check}: ') and sentence.endswith('.')
        assert '0x' not in sentence


def test_the_verdict_matches_the_first_check():
    rows = [fc.Row('Log', 'PASS', ''), fc.Row('Brake', 'FAIL', ''), fc.Row('Tick', 'FAIL', '')]
    assert ws.verdict_sentence(rows) == (False, 'Stop. Brake failed. We roll back now.')
    assert fc.verdict_line(rows).startswith('STOP: Brake')
    assert ws.verdict_sentence(rows[:1]) == (True, 'Safe to go on.')


# --- The board's settings ---------------------------------------------------------------

def test_the_settings_are_read_from_either_log_alike():
    text = ws.settings_from_lines(TEXT_LOG.read_text().splitlines())
    js = ws.settings_from_lines(JSON_LOG.read_text().splitlines())
    assert text['steer_centre'] == '1950' and js['steering_centre'] == '1950'
    assert ws.settings_sentences(text)[0] == 'Steering: min 400, centre 1950, max 3500.'
    assert ws.settings_sentences(js)[0] == 'Steering: min 400, centre 1950, max 3500.'
    said = ' '.join(ws.settings_sentences(text))
    assert 'Brake: idle 1, released 400, full 3500.' in said
    assert 'watchdog 100 milliseconds' in said and 'command timeout 100' in said
    assert '0x' not in said and 'build' not in said


def test_new_keys_are_said_and_missing_ones_are_fine():
    line = '\x1b[32mINFO - CONFIG t=5 build=abc max_speed=1.00 steer_rate_hz=50 ' \
           'brake_hold=1050 new_thing=7 last_reset=ChipPowerOn\x1b[0m'
    settings = ws.settings_from_lines([line])
    assert settings['new_thing'] == '7'          # shown, though no group says it
    assert ws.settings_sentences(settings) == [
        'Steering: rate 50 hertz.', 'Speed: max 1 metres a second.', 'Brake: hold 1050.']
    assert 'new_thing' in ws.settings_table(settings)


def test_no_config_line_says_so():
    assert ws.settings_from_lines(['nothing here']) == {}
    assert ws.settings_sentences({}) == []
    assert 'no CONFIG' in ws.settings_table({})


def test_the_last_config_line_wins():
    lines = ['CONFIG t=1 idle_brake=0.50', 'CONFIG t=2 idle_brake=1.00']
    assert ws.settings_from_lines(lines) == {'idle_brake': '1.00'}


# --- The steps --------------------------------------------------------------------------

def test_the_steps_follow_the_bench_page():
    keys = [s.key for s in ws.MAIN]
    for before, after in (('wheels', 'backup'), ('backup', 'flash'), ('build', 'flash'),
                          ('flash', 'first-check'), ('first-check', 'h1'), ('h13', 'h14')):
        assert keys.index(before) < keys.index(after)
    flash = next(s for s in ws.MAIN if s.key == 'flash')
    assert 'Wheels off the ground' in flash.say and 'E-stop in hand' in flash.say
    assert flash.colour == 'stop'
    assert any('e-stop now' in s.say for s in ws.MAIN)
    for s in ws.MAIN + ws.ROLL_BACK:
        assert len(s.headline.split()) <= 7, s.headline
        assert s.kind in ('do', 'run', 'flash', 'check', 'ask')


def test_the_commands():
    assert ws.backup_command('/dev/x', 'b.bin') == [
        ws.ESPFLASH, 'read-flash', '--port', '/dev/x', '0', '0x400000', 'b.bin']
    assert ws.flash_command(None) == [ws.ESPFLASH, 'flash', '--chip', 'esp32', ws.ELF]
    assert ws.restore_command('/dev/x', 'b.bin')[:2] == [ws.ESPFLASH, 'write-bin']
    assert ws.restore_command(None, None)[-2:] == ['-t', 'upload']
    assert ws.backup_ok(4194304) and not ws.backup_ok(1000) and not ws.backup_ok(None)


# --- The whole walk -----------------------------------------------------------------------

def test_a_dry_run_all_yes_is_all_done(tmp_path):
    code, io, records = run_walk(tmp_path, all_yes(), '--dry-run', '--file', str(TEXT_LOG),
                                 *QEMU)
    assert code == 0
    assert io.runs == []                        # a dry run runs nothing
    assert 'Would run: vendor/cargo-tools/bin/espflash flash --chip esp32' in io.screen
    assert 'Brake: pass. The brake is on.' in io.said
    assert 'Steering: min 400, centre 1950, max 3500.' in io.said
    assert 'SAFE TO GO ON to the next step' in io.screen
    assert io.said[-1] == 'The record is in the bench logs folder.'
    [record] = records
    assert record.name == '2026-11-07-esp32-first-flash-dry-run.md'
    text = record.read_text()
    assert '(dry run: nothing was run)' in text and 'all done' in text
    assert text.count('| yes |') == HANDS
    assert 'Pi link     INFO    NO FRAMES' in text and '| steer_centre | 1950 |' in text


def test_the_flash_waits_for_the_word(tmp_path):
    answers = [''] * BEFORE_FLASH + ['', 'yes', 'y', 'flash'] + ['yes'] * HANDS
    code, io, _ = run_walk(tmp_path, answers, '--dry-run', '--file', str(TEXT_LOG), *QEMU)
    assert code == 0
    assert io.screen.count('Not flashed. Type the word flash') == 3
    said = ' '.join(io.said)
    assert said.index('Wheels off the ground. E-stop in hand.') < said.index('Flashing.')


def test_stopping_before_the_flash_flashes_nothing(tmp_path):
    code, io, records = run_walk(tmp_path, ['', '', 'stop'], '--dry-run')
    assert code == 1
    assert 'espflash flash' not in io.screen and 'write-bin' not in io.screen
    assert 'Nothing was flashed' in ' '.join(io.said)
    assert 'stopped before the flash' in records[0].read_text()


def test_no_more_answers_stops(tmp_path):
    code, io, records = run_walk(tmp_path, [''] * BEFORE_FLASH, '--dry-run')
    assert code == 1
    assert 'espflash flash' not in io.screen
    assert 'not flashed' in records[0].read_text()


def test_a_stop_from_the_first_check_rolls_back(tmp_path):
    # This checkout's commit is not the fixture's: Build fails, so STOP.
    answers = [''] * BEFORE_FLASH + ['flash', '', 'flash', '', '']
    code, io, records = run_walk(tmp_path, answers, '--dry-run', '--file', str(TEXT_LOG),
                                 '--build', 'fffffff')
    assert code == 1
    assert 'STOP: Build, roll back (docs/esp32-bench.md)' in io.screen
    assert 'Build: fail. It is a different build. Flash it again.' in io.said
    assert 'espflash write-bin 0' in io.screen
    text = records[0].read_text()
    assert 'rolled back to the C++' in text and 'STOP: Build' in text
    assert 'Power on, nothing else' not in text      # no hands-on check after a STOP


def test_a_no_on_the_bench_rolls_back(tmp_path):
    answers = [''] * BEFORE_FLASH + ['flash', 'yes', 'skip', 'no', '', 'flash', '', '']
    code, io, records = run_walk(tmp_path, answers, '--dry-run', '--file', str(TEXT_LOG),
                                 *QEMU)
    assert code == 1
    text = records[0].read_text()
    assert '| skipped |' in text and '| NO |' in text
    assert 'rolled back to the C++' in text


def test_a_stop_after_the_flash_offers_the_roll_back(tmp_path):
    answers = [''] * BEFORE_FLASH + ['flash', 'stop', 'no']
    code, io, records = run_walk(tmp_path, answers, '--dry-run', '--file', str(TEXT_LOG),
                                 *QEMU)
    assert code == 1
    assert 'write-bin' not in io.screen
    assert 'the Rust stays on the board' in records[0].read_text()


def test_for_real_the_commands_run_in_order(tmp_path):
    lines = TEXT_LOG.read_text().splitlines()
    code, io, records = run_walk(tmp_path, all_yes(), '/dev/cu.fake', *QEMU,
                                 port_lines=lines)
    assert code == 0
    firsts = [cmd[:2] for cmd in io.runs]
    assert firsts == [['pixi', 'run'], [ws.ESPFLASH, 'list-ports'], [ws.ESPFLASH, 'read-flash'],
                      ['tools/esp32.sh', 'build'], [ws.ESPFLASH, 'flash']]
    assert io.runs[2][2:4] == ['--port', '/dev/cu.fake']
    assert io.port_reads == [('/dev/cu.fake', fc.SECONDS)]
    text = records[0].read_text()
    assert records[0].name == '2026-11-07-esp32-first-flash.md'
    assert '/dev/cu.fake' in text and '2026-11-07-1430-backup-esp32.bin' in text


def test_a_short_backup_never_flashes(tmp_path):
    answers = [''] * BEFORE_FLASH + ['', 'stop']     # try the copy again once, then stop
    code, io, _ = run_walk(tmp_path, answers, '/dev/cu.fake', size=1000)
    assert code == 1
    assert [c[1] for c in io.runs].count('read-flash') == 2
    assert not any(c[1] == 'flash' for c in io.runs)
    assert 'not 4194304' in ' '.join(io.said)


def test_a_failed_build_never_flashes(tmp_path):
    answers = [''] * BEFORE_FLASH + ['stop']
    code, io, _ = run_walk(tmp_path, answers, '/dev/cu.fake', codes={'build': 1})
    assert code == 1
    assert not any(c[1] == 'flash' for c in io.runs)


def test_the_roll_back_listens_for_the_cpp(tmp_path):
    lines = TEXT_LOG.read_text().splitlines()

    class Io(FakeIo):
        def read_port(self, port, seconds, save):
            self.port_reads.append((port, seconds))
            return lines if len(self.port_reads) == 1 else ['=== ESP32: AAV Controller ===']

    io = Io([''] * BEFORE_FLASH + ['flash', '', 'flash', ''], port_lines=lines)
    code = walk.main(['--out', str(tmp_path), '/dev/cu.fake', '--build', 'fffffff'], io=io)
    assert code == 1
    assert [c[1] for c in io.runs][-1] == 'write-bin'
    assert io.port_reads[-1] == ('/dev/cu.fake', 10.0)
    assert 'The C++ is back.' in io.screen


def test_a_second_record_the_same_day_gets_its_own_name(tmp_path):
    run_walk(tmp_path, ['stop'], '--dry-run')
    _, _, records = run_walk(tmp_path, ['stop'], '--dry-run')
    assert [r.name for r in records] == ['2026-11-07-esp32-first-flash-dry-run-2.md',
                                         '2026-11-07-esp32-first-flash-dry-run.md']


def test_quiet_means_no_say():
    assert walk.Speaker(False).cmd is None
    walk.Speaker(False).say('nothing')          # silent, and no error


def test_esp32_sh_walk_dry_run(tmp_path):
    answers = '\n'.join(all_yes()) + '\n'
    out = subprocess.run(
        ['bash', str(ROOT / 'tools/esp32.sh'), 'walk', '--dry-run', '--quiet', '--plain',
         '--file', str(TEXT_LOG), '--out', str(tmp_path), *QEMU],
        input=answers, capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert 'SAFE TO GO ON to the next step' in out.stdout
    assert 'Brake: pass. The brake is on.' in out.stdout
    assert '\x1b[' not in out.stdout
    assert len(list(tmp_path.glob('*-esp32-first-flash-dry-run.md'))) == 1


def test_a_lost_rc_is_said_as_not_yet_and_never_as_a_roll_back():
    rows = [fc.Row('Log', 'PASS', ''), fc.Row('Mode', 'WAIT', 'the RC is lost')]
    safe, sentence = ws.verdict_sentence(rows)
    assert not safe and sentence.startswith('Not yet.') and 'no roll back' in sentence
    assert fc.verdict_line(rows).startswith('NOT YET')
    # A real failure beside it wins.
    rows.append(fc.Row('Brake', 'FAIL', ''))
    assert ws.verdict_sentence(rows) == (False, 'Stop. Brake failed. We roll back now.')


def test_the_rc_step_says_the_transmitter_must_be_set_up():
    step = next(s for s in ws.MAIN if s.key == 'rc')
    assert '1100' in step.detail and 'T1 to T4' in step.detail


def test_a_lost_rc_after_the_flash_reads_again_and_does_not_roll_back(tmp_path):
    good = TEXT_LOG.read_text().splitlines()
    lost = [ln.replace('safety=Latched', 'safety=Degraded').replace('faults=0x0000',
                                                                    'faults=0x0100')
            + (' rc_fitted=true' if 'CONFIG' in ln else '') for ln in good]

    class Io(FakeIo):
        def read_port(self, port, seconds, save):
            self.port_reads.append((port, seconds))
            return lost if len(self.port_reads) == 1 else good

    # The flash, the first check (the RC lost), Enter to check again, then stop.
    io = Io([''] * BEFORE_FLASH + ['flash', '', 'stop', 'no'], port_lines=good)
    walk.main(['--out', str(tmp_path), '/dev/cu.fake', *QEMU], io=io)
    assert len(io.port_reads) == 2
    assert 'NOT YET' in io.screen and 'Not yet.' in ' '.join(io.said)
    assert 'write-bin' not in [c[1] for c in io.runs]
    assert 'Roll back to the C++.' not in io.said
