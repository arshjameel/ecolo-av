"""esp32_qemu: reading the firmware's log, and judging a heartbeat cycle from it."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import esp32_qemu as q  # noqa: E402
from aav_legacy.legacy_protocol import unpack_frame_v2  # noqa: E402,I100
import pytest  # noqa: E402

# Lines as the firmware prints them on UART0, colour codes and all.
EVENT = ('\x1b[32mINFO - EVENT t=1415 mode=Manual armed=false estop=true throttle=0.00 '
         'brake=1.00 reverse=false steer=0.000 faults=0x0001\x1b[0m')
DEVICES = '\x1b[32mINFO - DEVICES t=1415 steering=1950 brake=3500 dac=0 expander=0xffff\x1b[0m'


def status(t, *, armed, throttle, brake, faults=0, kind='STATUS', mode=None):
    """Return a STATUS or EVENT line, as the firmware writes it."""
    mode = mode or ('Computer' if armed else 'Manual')
    return (f'INFO - {kind} t={t} mode={mode} armed={str(armed).lower()} estop=false '
            f'throttle={throttle:.2f} brake={brake:.2f} reverse=false steer=0.000 '
            f'faults={faults:#06x}')


def devices(t, brake, dac):
    return f'INFO - DEVICES t={t} steering=1950 brake={brake} dac={dac} expander=0xffff'


def firmware(timeout_after=110, latched=True, brake_on_timeout=True):
    """
    Return the lines of one cycle, with the host seeing each 1 ms after it was printed.

    The firmware's clock is the host's minus 100 s. Drive from 1.0 s to 2.0 s (the last
    frame), silence, frames again from 2.6 s to 3.2 s.
    """
    texts = []
    for t in range(1000, 2001, 200):
        texts.append((t, status(t, armed=True, throttle=0.2, brake=0.0)))
        texts.append((t, devices(t, 400, min(140, (t - 1000) // 2 + 6))))
    trip = 2000 + timeout_after
    brake = 1.0 if brake_on_timeout else 0.0
    texts.append((trip, status(trip, armed=False, throttle=0.0, brake=brake,
                               faults=q.CMD_TIMEOUT, kind='EVENT')))
    texts.append((trip, devices(trip, q.BRAKE_FULL if brake_on_timeout else 400, 0)))
    for t in range(2600, 3201, 200):
        if latched:
            texts.append((t, status(t, armed=False, throttle=0.0, brake=1.0,
                                    faults=q.CMD_TIMEOUT)))
        else:
            texts.append((t, status(t, armed=True, throttle=0.2, brake=0.0)))
    return [q.parse_line(text, 100 + t / 1000 + 0.001) for t, text in texts]


CYCLE = q.Cycle(stop_from=100.7, drive_from=101.0, last_frame=102.0, resume_from=102.6,
                end=103.2)


def judged(**kwargs):
    lines = firmware(**kwargs)
    return q.judge(CYCLE, lines, q.ClockMap.fit(lines))


def test_lines_are_read_with_their_numbers():
    e = q.parse_line(EVENT, 5.0)
    assert (e.kind, e.t, e.host) == ('EVENT', 1415, 5.0)
    assert e.get('faults') == q.CMD_TIMEOUT
    assert e.get('brake') == 1.0 and e.get('throttle') == 0.0
    assert e.get('armed') is False and e.get('mode') == 'Manual'
    d = q.parse_line(DEVICES, 5.0)
    assert (d.get('brake'), d.get('dac'), d.get('expander')) == (3500, 0, 0xFFFF)
    assert q.parse_line('I (833) boot: Loaded app', 5.0).kind == 'TEXT'


def test_a_line_cut_short_is_not_judged():
    """Skip a line QEMU cut short on a busy computer (3 Nov: a DEVICES line with no dac)."""
    assert q.parse_line('INFO - DEVICES t=1415 steering=1950 brake=35', 5.0).kind == 'CUT'
    assert q.parse_line('INFO - STATUS t=1415 mode=Computer armed=true', 5.0).kind == 'CUT'
    assert q.parse_line(DEVICES, 5.0).kind == 'DEVICES'


def test_the_clocks_map_through_the_fastest_line():
    lines = [q.parse_line(status(t, armed=False, throttle=0, brake=1), 100 + t / 1000 + lag)
             for t, lag in [(0, 0.004), (200, 0.001), (400, 0.003)]]
    clock = q.ClockMap.fit(lines)
    assert clock.offset == pytest.approx(100.001)
    assert clock.to_firmware_ms(100.5) == pytest.approx(499)
    assert clock.rate == pytest.approx(0.4 / 0.399)


def test_a_good_cycle_passes():
    r = judged()
    assert r.ok, r.problems
    assert r.drove and r.dac_rose == 140
    assert r.timeout_fw_ms == pytest.approx(111)    # 110 ms, and the 1 ms lag
    assert r.timeout_host_ms == pytest.approx(111)
    assert r.brake_full_fw_ms == pytest.approx(111)
    assert r.fault_bit and r.stopped and r.latched


def test_a_late_timeout_fails():
    r = judged(timeout_after=250)
    assert not r.ok
    assert any('not 100-200' in p for p in r.problems)


def test_a_timeout_that_does_not_brake_fails():
    r = judged(brake_on_timeout=False)
    assert not r.stopped
    assert any('did not stop the car' in p for p in r.problems)
    assert any('never got 3500' in p for p in r.problems)


def test_driving_again_without_a_stop_fails():
    r = judged(latched=False)
    assert r.latched is False
    assert any('without a stop drove again' in p for p in r.problems)


def test_no_timeout_at_all_fails():
    lines = [ln for ln in firmware() if not (ln.kind == 'EVENT')]
    r = q.judge(CYCLE, lines, q.ClockMap.fit(lines))
    assert any('no timeout event' in p for p in r.problems)


def test_the_frames_are_the_bridges():
    # What legacy_uart_bridge sends: 0.6 m/s straight, and its stale stop.
    m = q.LegacyMapping()
    assert q.to_legacy(*q.DRIVE, m) == pytest.approx((0.0, 0.2, 0.0))
    assert q.stop_command(m) == (0.0, 0.0, -1.0)
    assert len(q.pack_frame(*q.stop_command(m))) == 13


def test_the_table_has_a_row_per_cycle():
    lines = firmware()
    text = q.table([judged(), judged(latched=False)], q.ClockMap.fit(lines))
    rows = [r for r in text.splitlines() if r.startswith('| ') and r[2].isdigit()]
    assert len(rows) == 2
    assert rows[0].endswith('| pass |') and rows[1].endswith('| FAIL |')


# --- Version 2: the Orin's counter (TODO 50) ---

def link(t, *, bad=0, repeats=0):
    return (f'INFO - LINK t={t} frames=100 bad={bad} skipped=0 repeats={repeats} '
            f'counter=7')


def with_links(lines, before, after):
    """Add a LINK line before the gap (at 1.8 s) and one after it (at 2.8 s)."""
    extra = [q.parse_line(link(1800, **before), 101.8 + 0.001),
             q.parse_line(link(2800, **after), 102.8 + 0.001)]
    return sorted(lines + extra, key=lambda ln: ln.host)


def test_link_lines_are_read():
    ln = q.parse_line(link(5, bad=3), 1.0)
    assert (ln.kind, ln.get('bad'), ln.get('counter')) == ('LINK', 3, 7)


@pytest.mark.parametrize('kind,field', [('frozen', 'repeats'), ('bad_crc', 'bad')])
def test_a_gap_of_frames_must_be_counted(kind, field):
    cycle = q.Cycle(100.7, 101.0, 102.0, 102.6, 103.2, kind=kind, gap_frames=30)
    lines = with_links(firmware(), {field: 1}, {field: 31})
    r = q.judge(cycle, lines, q.ClockMap.fit(lines))
    assert r.ok, r.problems
    assert r.counted == 30
    lines = with_links(firmware(), {field: 1}, {field: 11})
    r = q.judge(cycle, lines, q.ClockMap.fit(lines))
    assert any(f'only 10 counted as {field}' in p for p in r.problems)


def test_the_orins_frames_move_freeze_or_break():
    orin = q.Orin()
    values = q.to_legacy(*q.DRIVE, q.LegacyMapping())
    moving, frozen = orin.frames(values), orin.frames(values, move=False)
    counters = [unpack_frame_v2(f()).counter for f in (moving, moving, frozen, frozen)]
    assert counters == [1, 2, 2, 2]
    with pytest.raises(ValueError):
        unpack_frame_v2(orin.frames(values, bad_crc=True)())


# --- The uplink and the RC (TODO 50, 48) ---

def up(**changes):
    fields = {'sequence': 0, 'echo': None, 'mode': q.MODE_COMPUTER, 'switch': q.SWITCH_NONE,
              'armed': True, 'estop': False, 'faults': 0, 'steering': 0.0,
              'brake_position': 0.0, 'throttle_out': 0.2, 'brake_out': 0.0,
              'rc_throttle': 0.0, 'rc_steering': 0.0, 'rc_brake': False}
    fields.update(changes)
    return q.Uplink(**fields)


def uplink_run(stop_after=0.12, echo_lag=1, frozen=False):
    """
    Return one cycle's frames sent and uplinks seen, at 50 Hz each, as CYCLE times them.

    The echo is the counter sent `echo_lag` frames before; `stop_after` s after the last
    frame an uplink says stopped with fault 1.
    """
    kind = 'frozen' if frozen else 'silence'
    cycle = q.Cycle(100.7, 101.0, 102.0, 102.6, 103.2, kind=kind)
    sent = [q.Sent(100.7 + i * 0.02, i + 1) for i in range(66)]   # to 102.0
    ups, seq = [], 0
    for i in range(125):
        h = 100.705 + i * 0.02
        before = [s for s in sent if s.host <= h]
        echo = before[-1 - echo_lag].counter if len(before) > echo_lag else None
        stopped = h > 102.0 + stop_after
        ups.append((h, up(sequence=seq, echo=echo,
                          mode=q.MODE_STOPPED if stopped else q.MODE_COMPUTER,
                          armed=not stopped, faults=q.CMD_TIMEOUT if stopped else 0)))
        seq = (seq + 1) & 0xFF
    return cycle, ups, sent


def test_a_good_uplink_passes():
    r = q.judge_uplink(*uplink_run())
    assert r.ok, r.problems
    assert 45 <= r.rate_hz <= 55
    assert r.echo_ok and r.computer
    assert 120 <= r.stopped_ms <= 140


def test_a_stale_echo_fails():
    r = q.judge_uplink(*uplink_run(echo_lag=40))   # 0.8 s old
    assert not r.echo_ok
    assert any('echoes not among' in p for p in r.problems)


def test_a_late_stop_in_the_uplink_fails():
    r = q.judge_uplink(*uplink_run(stop_after=0.4))
    assert any('said stopped after' in p for p in r.problems)


def test_the_echo_must_freeze_with_the_counter():
    cycle, ups, sent = uplink_run(frozen=True, echo_lag=0)
    r = q.judge_uplink(cycle, ups, sent)
    assert r.frozen_echo, r.problems     # nothing sent in the gap: the echo stays
    sent += [q.Sent(102.1, 200)]         # a new counter in the gap: it moves
    ups = [(h, up(sequence=u.sequence, echo=200 if h > 102.1 else u.echo, mode=u.mode,
                  armed=u.armed, faults=u.faults)) for h, u in ups]
    assert q.judge_uplink(cycle, ups, sent).frozen_echo is False


def test_lost_uplinks_are_counted_through_the_wrap():
    seqs = [253, 254, 255, 0, 1, 3]
    assert q.lost_uplinks([(0.0, up(sequence=s)) for s in seqs]) == 1


def test_the_ibus_frame_is_the_receivers():
    frame = q.ibus_frame(q.rc_channels('via_ros'))
    assert len(frame) == 32 and frame[:2] == b'\x20\x40'
    assert int.from_bytes(frame[30:], 'little') == (0xFFFF - sum(frame[:30])) & 0xFFFF
    # CH6 (index 5) is the mode switch, in the middle for RC via ROS.
    assert int.from_bytes(frame[12:14], 'little') == 1500


def rc_lines(dac, brake=400, start=100.0, end=101.0):
    """DEVICES lines every 50 ms; the firmware's clock is the host's minus 100 s."""
    return [q.parse_line(devices(int((t - 100) * 1000), brake, dac), t)
            for t in [start + i * 0.05 for i in range(int((end - start) / 0.05) + 1)]]


def test_rc_via_ros_must_not_drive_from_the_sticks():
    phase = q.RcPhase('via_ros_still', 100.0, 101.0)
    ups = [(100.0 + i * 0.02, up(switch=q.SWITCH_RC_VIA_ROS, rc_throttle=1.0))
           for i in range(50)]
    lines = rc_lines(dac=0)
    clock = q.ClockMap.fit(lines)
    assert q.judge_rc(phase, ups, lines, clock).ok
    lines = rc_lines(dac=200)
    r = q.judge_rc(phase, ups, lines, q.ClockMap.fit(lines))
    assert any('drove from the sticks' in p for p in r.problems)


def test_a_lost_rc_must_say_fault_256_and_brake():
    phase = q.RcPhase('direct_lost', 100.0, 101.0, rc_stop=100.0)
    lost = up(switch=q.SWITCH_NONE, mode=q.MODE_RC_DIRECT, faults=q.RC_SIGNAL_LOST,
              throttle_out=0.0, brake_out=1.0)
    ups = [(100.0 + i * 0.02, lost if i >= 6 else up(switch=q.SWITCH_RC_DIRECT))
           for i in range(50)]
    lines = rc_lines(dac=0, brake=q.BRAKE_FULL)
    r = q.judge_rc(phase, ups, lines, q.ClockMap.fit(lines))
    assert r.ok, r.problems
    assert r.ms == pytest.approx(120)
    ups = [(h, up(switch=q.SWITCH_RC_DIRECT)) for h, _ in ups]   # never lost
    r = q.judge_rc(phase, ups, lines, q.ClockMap.fit(lines))
    assert any('fault 256' in p for p in r.problems)


# --- A loaded computer (24 Oct): the next cycle's stop must not count as driving again ---

def with_counters(lines, resume=7, after=8):
    """Give each status line a counter=: `resume` from 2.6 s; add the next cycle's stop."""
    out = []
    for ln in lines:
        if ln.kind in ('EVENT', 'STATUS'):
            c = resume if ln.t >= 2600 else 3
            ln = q.parse_line(ln.text + f' counter={c}', ln.host)
        out.append(ln)
    # The next cycle's stand-still frame, counter 8: armed, the hold brake. On a loaded
    # computer its line can fall inside the resume window by the clocks.
    nxt = status(3150, armed=True, throttle=0.0, brake=0.21, kind='EVENT') + f' counter={after}'
    return sorted(out + [q.parse_line(nxt, 103.15 + 0.001)], key=lambda ln: ln.host)


def test_the_next_cycles_stop_is_not_driving_again_with_counters():
    lines = with_counters(firmware())
    cycle = q.Cycle(100.7, 101.0, 102.0, 102.6, 103.2, resume_counters=frozenset({7}))
    r = q.judge(cycle, lines, q.ClockMap.fit(lines))
    assert r.ok, r.problems
    # By the clocks alone (no counters known), the same line looks like driving again.
    r = q.judge(q.Cycle(100.7, 101.0, 102.0, 102.6, 103.2), lines, q.ClockMap.fit(lines))
    assert any('drove again' in p for p in r.problems)


def test_a_later_cycle_on_the_same_counter_after_the_wrap_does_not_count():
    # The counter is one byte: two cycles on, a drive frame carries 7 again.
    lines = with_counters(firmware())
    later = status(3400, armed=True, throttle=0.2, brake=0.0, kind='EVENT') + ' counter=7'
    lines = lines + [q.parse_line(later, 103.4 + 0.001)]
    cycle = q.Cycle(100.7, 101.0, 102.0, 102.6, 103.2, resume_counters=frozenset({7}))
    r = q.judge(cycle, lines, q.ClockMap.fit(lines))
    assert r.ok, r.problems


def test_driving_again_on_a_resume_counter_still_fails():
    lines = with_counters(firmware(latched=False))
    cycle = q.Cycle(100.7, 101.0, 102.0, 102.6, 103.2, resume_counters=frozenset({7}))
    r = q.judge(cycle, lines, q.ClockMap.fit(lines))
    assert any('drove again' in p for p in r.problems)


def test_a_local_clock_fit_ignores_a_drift_elsewhere_in_the_run():
    # Early lines arrive 1 ms after printing; later the clocks have drifted 300 ms.
    early = [q.parse_line(status(t, armed=False, throttle=0, brake=1), 100 + t / 1000 + 0.001)
             for t in range(0, 1000, 200)]
    late = [q.parse_line(status(t, armed=False, throttle=0, brake=1), 100.301 + t / 1000)
            for t in range(10000, 11000, 200)]
    lines = early + late
    assert q.ClockMap.fit(lines).offset == pytest.approx(100.001)
    assert q.ClockMap.fit_local(lines, 110.3, 111.3).offset == pytest.approx(100.301)
