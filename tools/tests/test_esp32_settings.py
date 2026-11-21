"""
The ESP32's settings line (CONFIG / config) against the files it copies or must match.

| Check | Against |
|---|---|
| `legacy_link::FRAME_HZ` | the Motor-Pi's `rate_hz` and `baud` in legacy.yaml |
| the golden `config` line | every setting Ben asked for (7 Nov) is there |
| the car-day build | never builds `wifi-status` or `wifi-update` |
"""

import json
from pathlib import Path
import re

import yaml

ROOT = Path(__file__).resolve().parents[2]
FRAME_RS = ROOT / 'firmware/spine/crates/legacy-link/src/frame.rs'
LEGACY_YAML = ROOT / 'ros/can_bridge/aav_legacy/config/legacy.yaml'
GOLDEN = ROOT / 'firmware/status/golden.jsonl'
CAR_DAY = ROOT / 'tools/car_day_build.sh'


def rust_const(path, name):
    found = re.search(rf'pub const {name}: u32 = ([0-9_]+);', path.read_text())
    assert found, f'{name} in {path}'
    return int(found.group(1).replace('_', ''))


def test_the_frame_rate_and_baud_are_the_motor_pis():
    bridge = yaml.safe_load(LEGACY_YAML.read_text())['legacy_uart_bridge']['ros__parameters']
    assert rust_const(FRAME_RS, 'FRAME_HZ') == bridge['rate_hz']
    assert rust_const(FRAME_RS, 'BAUD') == bridge['baud']


def test_the_golden_config_line_has_every_setting():
    config = next(json.loads(line) for line in GOLDEN.read_text().splitlines()
                  if '"t":"config"' in line)
    want = {
        'steering': ['min', 'centre', 'max', 'counts_per_rad', 'max_angle_rad',
                     'rate_rad_s', 'cal_rate', 'cal_timeout_ms', 'on_stop'],
        'brake': ['released', 'full', 'stop', 'overspeed', 'rc_band', 'rc_brake_ms',
                  'rc_lost_below', 'rc_fitted'],
        'speed': ['max_forward', 'max_reverse', 'cap_code', 'reverse_cap_code',
                  'frame_dac_cap', 'rise_per_s', 'max_accel', 'max_decel',
                  'overspeed_margin'],
        'timeouts': ['rc_lost_ms', 'speed_lost_ms', 'overspeed_ms', 'estop_release_ms',
                     'bad_frames_ms'],
        'rates': ['uart_baud', 'frame_hz', 'uplink_ms', 'can_kbit', 'ibus_baud',
                  'jrk_write_ms', 'dac_write_ms', 'status_ms'],
    }
    for group, keys in want.items():
        assert list(config[group]) == keys, group
    for key in ('cmd_timeout_ms', 'watchdog_ms', 'tick_ms', 'i2c_khz', 'idle_brake',
                'full_throttle_speed', 'throttle_full_code', 'frame_version'):
        assert key in config, key
    # The keys the first check has always read come first, as they were.
    assert list(config).index('last_reset') < list(config).index('speed')


def test_the_car_day_build_has_no_wifi():
    text = CAR_DAY.read_text()
    assert 'wifi-status' not in text
    assert 'wifi-update' not in text
