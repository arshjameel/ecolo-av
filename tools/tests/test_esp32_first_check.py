"""
Tests for tools/esp32_first_check.py: the first check after a flash, on made-up logs.

The text lines are written as src/main.rs writes them (esp-println's `INFO - ` prefix and
its colours included); the JSON lines are firmware/status/golden.jsonl's.
"""

import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import esp32_first_check as fc  # noqa: E402

GOLDEN = Path(__file__).resolve().parents[2] / 'firmware/status/golden.jsonl'
BUILD = 'abc1234'


def info(text):
    return f'\x1b[32mINFO - {text}\x1b[0m'


def config(t=0, reset='ChipPowerOn', features='legacy-uart', build=BUILD, idle='1.00'):
    return info(
        f'CONFIG t={t} build={build} features={features} version=0.1.0 link=uart frame=2 '
        f'tick_ms=10 watchdog_ms=100 cmd_timeout_ms=100 idle_brake={idle} '
        'full_throttle_speed=3.00 i2c_khz=100 steering_jrk=0x05 brake_jrk=0x04 dac=0x0c '
        'expander=0x20 steer_min=400 steer_centre=1950 steer_max=3500 brake_released=400 '
        f'brake_full=3500 throttle_full_code=700 last_reset={reset}')


def devices(t=0, dac='answered'):
    n = 4 if dac == 'answered' else 3
    return info(f'DEVICES {n}/4 answered t={t} steering=0x05,answered,analog '
                f'brake=0x04,answered,none dac=0x0c,{dac} expander=0x20,answered')


def boot(reset='ChipPowerOn'):
    return info("spine-esp32-umod 0.1.0, link: Motor-Pi UART, version 2 frames (the Orin's "
                f'heartbeat); the last reset: Some({reset})')


def status(t, mode='Manual', armed='false', throttle='0.00', brake='1.00', steer='0.000',
           faults='0x0000'):
    return info(f'STATUS t={t} mode={mode} armed={armed} estop=false throttle={throttle} '
                f'brake={brake} reverse=false steer={steer} faults={faults} counter=-1')


def fsm(t, safety='Latched'):
    return info(f'FSM t={t} lifecycle=Running safety={safety} authority=Idle motion=Stopped '
                'confidence=Unknown')


def tick(t, gap=10500, overruns=0):
    return info(f'TICK t={t} worst_gap_us={gap} overruns={overruns}')


def health(t, link='no_frames', faults='0x0000'):
    return info(f'HEALTH t={t} link={link} age_ms=none counter_moving=unknown frames=0 '
                f'bad=0 skipped=0 repeats=0 devices=4/4 faults={faults} mode=Manual '
                'armed=false')


def text_log(seconds=30, **kw):
    """Return a healthy board's log, idle, the Pi off: the default build's."""
    lines = ['ets Jul 29 2019 12:21:46', 'rst:0x1 (POWERON_RESET),boot:0x12', boot(),
             config(10), devices(10),
             info('FSM t=12 lifecycle=SelfTest safety=Estop authority=Idle motion=Stopped '
                  'confidence=Degraded')]
    for t in range(0, seconds * 1000, 200):
        lines += [status(t, **kw), fsm(t)]
        if t % 1000 == 400:
            lines.append(tick(t + 100))
            lines.append(health(t + 200, faults=kw.get('faults', '0x0000')))
        if t % 10_000 == 0 and t:
            lines += [config(t + 250), devices(t + 350)]
    return lines


def run(lines, build=BUILD, features=None, pi='any'):
    rows = fc.judge(fc.parse(lines), build, features, pi)
    return {r.check: r for r in rows}, fc.verdict_line(rows)


def test_a_healthy_idle_board_is_safe_to_go_on():
    rows, final = run(text_log())
    assert final == 'SAFE TO GO ON to the next step', fc.table(list(rows.values()))
    for check in ('Build', 'Features', 'Last reset', 'Devices', 'Brake', 'Throttle',
                  'Steering', 'Mode', 'Faults', 'Tick', 'Restarts'):
        assert rows[check].result == 'PASS', rows[check]
    assert rows['Pi link'].result == 'INFO'
    assert 'NO FRAMES' in rows['Pi link'].detail
    assert 'feedback analog' in rows['Devices'].detail


def test_the_default_build_with_the_uplink_is_safe_to_go_on():
    # Since #23 the default build is legacy-uart and uplink.
    lines = [config(10, features='legacy-uart,uplink') if 'CONFIG' in ln else ln
             for ln in text_log()]
    rows, final = run(lines)
    assert rows['Features'].result == 'PASS', rows['Features']
    assert final.startswith('SAFE TO GO ON'), final


def test_the_rc_lost_bit_is_expected_and_said():
    rows, final = run(text_log(faults='0x0100'))
    assert rows['Faults'].result == 'PASS'
    assert 'transmitter' in rows['Faults'].detail
    assert final.startswith('SAFE')


def test_a_device_fault_stops():
    rows, final = run(text_log(faults='0x0010'))
    assert rows['Faults'].result == 'FAIL'
    assert 'brake Jrk' in rows['Faults'].detail
    assert final == 'STOP: Faults, roll back (docs/esp32-bench.md)'


def test_a_missing_device_stops():
    lines = [ln if 'DEVICES' not in ln else devices(10, dac='missing') for ln in text_log()]
    rows, final = run(lines)
    assert rows['Devices'].result == 'FAIL'
    assert 'dac 0x0c MISSING' in rows['Devices'].detail
    assert final.startswith('STOP: Devices')


def test_a_watchdog_reset_stops():
    lines = [boot('CoreMwdt1') if 'spine-esp32-umod' in ln else
             config(10, reset='CoreMwdt1') if 'CONFIG t=10 ' in ln else ln
             for ln in text_log()]
    rows, final = run(lines)
    assert rows['Last reset'].result == 'FAIL'
    assert 'watchdog' in rows['Last reset'].detail
    assert final.startswith('STOP: Last reset')


def test_a_restart_in_the_window_stops():
    lines = text_log(10) + [boot('CoreMwdt1')] + text_log(5)[3:]
    rows, final = run(lines)
    assert rows['Restarts'].result == 'FAIL'
    assert 'second boot' in rows['Restarts'].detail
    # The uptime going back, with no boot line seen (cut short), is one too.
    lines = text_log(10) + text_log(5)[3:]
    rows, _ = run(lines)
    assert rows['Restarts'].result == 'FAIL'
    assert 'went back' in rows['Restarts'].detail


def test_the_wrong_build_or_a_qemu_build_stops():
    rows, final = run(text_log(), build='fff0000')
    assert rows['Build'].result == 'FAIL' and 'flash it again' in rows['Build'].detail
    lines = [config(10, features='qemu,uplink') if 'CONFIG' in ln else ln for ln in text_log()]
    rows, _ = run(lines)
    assert rows['Features'].result == 'FAIL' and 'QEMU' in rows['Features'].detail
    rows, final = run(lines, features='uplink,qemu')
    assert rows['Features'].result == 'PASS'


def test_a_self_test_that_never_passes_stops():
    lines = [ln.replace('lifecycle=Running', 'lifecycle=SelfTest') for ln in text_log()]
    rows, final = run(lines)
    assert rows['Mode'].result == 'FAIL' and 'self-test' in rows['Mode'].detail


def test_throttle_brake_steering_and_mode_are_each_checked():
    rows, final = run(text_log(throttle='0.20'))
    assert rows['Throttle'].result == 'FAIL'
    rows, _ = run(text_log(brake='0.00'))
    assert rows['Brake'].result == 'FAIL'
    rows, _ = run(text_log(steer='0.120'))
    assert rows['Steering'].result == 'FAIL'
    rows, _ = run(text_log(mode='Rc'))
    assert rows['Mode'].result == 'FAIL'
    lines = [fsm(0, safety='Ok') if ln.startswith(fsm(0)[:20]) and 'FSM t=0 ' in ln else ln
             for ln in text_log()]
    rows, _ = run(lines)
    assert rows['Mode'].result == 'FAIL' and 'NOT latched' in rows['Mode'].detail


def test_a_long_tick_gap_stops_and_late_ticks_are_said():
    lines = text_log() + [tick(31_000, gap=60_000, overruns=3)]
    rows, final = run(lines)
    assert rows['Tick'].result == 'FAIL'
    lines = text_log() + [tick(31_000, gap=35_000, overruns=12)]
    rows, final = run(lines)
    assert rows['Tick'].result == 'PASS'
    assert rows['Late ticks'].result == 'INFO' and '12 ticks' in rows['Late ticks'].detail
    assert final.startswith('SAFE')


def test_the_pi_link_is_said_and_can_be_asked_for():
    lines = [ln.replace('link=no_frames', 'link=counter_frozen') for ln in text_log()]
    rows, final = run(lines)
    assert rows['Pi link'].result == 'WARN' and 'mux' in rows['Pi link'].detail
    assert final.startswith('SAFE'), 'a warning is no stop'
    rows, final = run(text_log(), pi='on')
    assert rows['Pi link'].result == 'FAIL'
    assert final.startswith('STOP: Pi link')
    ok = [ln.replace('link=no_frames', 'link=ok') for ln in text_log()]
    rows, final = run(ok, pi='on')
    assert rows['Pi link'].result == 'PASS'


def test_nothing_read_or_an_old_build_stops():
    rows, final = run([])
    assert rows['Log'].result == 'FAIL' and final.startswith('STOP: Log')
    old = [ln for ln in text_log() if 'CONFIG' not in ln and 'DEVICES' not in ln
           and 'HEALTH' not in ln]
    rows, final = run(old)
    assert rows['Build'].result == 'FAIL' and 'older build' in rows['Build'].detail


def golden():
    return [json.loads(ln) for ln in GOLDEN.read_text().splitlines() if ln.strip()]


def json_log(seconds=30):
    """Return the json-log build's log: the text boot lines, then golden lines, moving on."""
    gold = {}
    for line in golden():
        if line['t'] == 'status' and line['rx'] is not None:
            continue                       # the driving one: not idle
        gold.setdefault(line['t'], line)
    lines = [boot(), config(10, features='json-log,legacy-uart'), devices(10)]
    seq = 0
    for t in range(0, seconds * 1000, 200):
        kinds = ['status'] + (['hb', 'health'] if t % 1000 == 0 else []) + (
            ['config', 'devices'] if t % 10_000 == 0 else [])
        for kind in kinds:
            line = dict(gold[kind], seq=seq, up_ms=t)
            if kind == 'config':
                line['features'] = 'json-log,legacy-uart'
            if kind == 'health':
                line['link'] = 'no_frames'
            seq += 1
            lines.append(json.dumps(line, separators=(',', ':')))
    return lines


def test_the_json_log_is_read_too():
    rows, final = run(json_log())
    assert final.startswith('SAFE'), fc.table(list(rows.values()))
    assert 'json' in rows['Log'].detail
    assert rows['Brake'].result == 'PASS' and '3500' in rows['Brake'].detail
    assert rows['Steering'].result == 'PASS'
    assert rows['Devices'].result == 'PASS'


def test_a_new_boot_id_in_the_json_log_stops():
    lines = json_log(5)
    later = []
    for ln in json_log(5)[3:]:
        line = json.loads(ln)
        line['up_ms'] += 10_000
        if line['t'] == 'hb':
            line['boot_id'] = 'deadbeef'
        later.append(json.dumps(line))
    rows, final = run(lines + later)
    assert rows['Restarts'].result == 'FAIL' and 'boot_id' in rows['Restarts'].detail


def test_main_reads_a_file_and_says_so_in_json(tmp_path, capsys):
    log = tmp_path / 'board.log'
    log.write_text('\n'.join(text_log()) + '\n')
    assert fc.main(['--file', str(log), '--build', BUILD]) == 0
    out = capsys.readouterr().out
    assert out.rstrip().endswith('SAFE TO GO ON to the next step')
    assert 'Check' in out and 'PASS' in out
    assert fc.main(['--file', str(log), '--build', BUILD, '--pi', 'on', '--json']) == 1
    got = json.loads(capsys.readouterr().out)
    assert got['safe'] is False and got['verdict'].startswith('STOP: Pi link')
    assert fc.main(['--file', str(tmp_path / 'none.log')]) == 2


# Recorded on 7 Nov: the QEMU build booted idle (tools/esp32.sh first-check --qemu), its
# log as the cable carried it, the ROM's and the bootloader's lines included.
QEMU_TEXT = Path(__file__).with_name('esp32_first_check_qemu_text.txt')
QEMU_JSON = Path(__file__).with_name('esp32_first_check_qemu_json.txt')


def test_the_recorded_qemu_logs_are_safe_to_go_on():
    for log, features in ((QEMU_TEXT, 'legacy-uart,qemu,uplink'),
                          (QEMU_JSON, 'json-log,legacy-uart,qemu')):
        rows, final = run(log.read_text().splitlines(), build='216ca6b', features=features)
        assert final.startswith('SAFE'), (log.name, fc.table(list(rows.values())))
        assert rows['Devices'].detail.startswith('4/4')
        assert 'NO FRAMES' in rows['Pi link'].detail
    rows, final = run(QEMU_TEXT.read_text().splitlines(), build='216ca6b')
    assert rows['Features'].result == 'FAIL', 'a QEMU build is never for the car'


# --- With a receiver fitted (21 Nov: the RC rework, #35 and #36) ---------------------

def with_receiver(lines):
    """Return the log of a board whose CONFIG line says rc_fitted=true, as the car's does."""
    return [ln + ' rc_fitted=true rc_lost_below=1050' if 'CONFIG' in ln else ln for ln in lines]


def test_the_rc_holding_the_car_braked_at_power_up_is_safe_to_go_on():
    # Normal mode's arbiter: the transmitter on, nobody asking: mode Rc, throttle off,
    # brake on. The first check said STOP here until 21 Nov, so the walk rolled back.
    rows, final = run(with_receiver(text_log(mode='Rc')))
    assert final == 'SAFE TO GO ON to the next step', fc.table(list(rows.values()))
    assert rows['Mode'].result == 'PASS' and 'the RC holds the car' in rows['Mode'].detail


def test_mode_rc_is_still_a_stop_without_a_receiver_or_with_any_throttle():
    rows, final = run(text_log(mode='Rc'))
    assert rows['Mode'].result == 'FAIL' and final.startswith('STOP: Mode')
    rows, final = run(with_receiver(text_log(mode='Rc', throttle='0.20')))
    assert rows['Mode'].result == 'FAIL' and final.startswith('STOP')


def test_a_lost_rc_alone_waits_and_does_not_roll_back():
    # The transmitter off, or CH6's End points not set: safety Degraded, fault 256, braked.
    lines = [ln.replace('safety=Latched', 'safety=Degraded')
             for ln in with_receiver(text_log(faults='0x0100'))]
    rows, final = run(lines)
    assert rows['Mode'].result == 'WAIT', rows['Mode']
    assert final.startswith('NOT YET') and 'No roll back' in final
    assert 'roll back (docs' not in final


def test_degraded_without_the_rc_lost_bit_is_a_stop():
    lines = [ln.replace('safety=Latched', 'safety=Degraded') for ln in with_receiver(text_log())]
    rows, final = run(lines)
    assert rows['Mode'].result == 'FAIL' and final.startswith('STOP: Mode')


def test_a_failure_beside_a_lost_rc_is_still_a_stop():
    lines = [ln.replace('safety=Latched', 'safety=Degraded')
             for ln in with_receiver(text_log(faults='0x0100', brake='0.00'))]
    _, final = run(lines)
    assert final.startswith('STOP: Brake')
