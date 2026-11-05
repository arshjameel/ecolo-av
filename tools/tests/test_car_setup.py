"""
Tests for tools/car_setup.sh: its dry run changes nothing, and --apply needs the role typed.

Each run gets an empty HOME in a temporary folder: the test checks it stays empty. The
hostname is the role's own (AAV_HOSTNAME) unless a test says otherwise, and no reboot is
waiting (AAV_REBOOT_FLAG points at a file that isn't there).
"""

import configparser
from pathlib import Path
import platform
import re
import subprocess

import pytest

TOOLS = Path(__file__).resolve().parents[1]
SCRIPT = TOOLS / 'car_setup.sh'
ROLES = ['orin', 'motor_pi', 'localization', 'dash', 'camera_pi', 'lab_desktop']
# The hostnames the 21 Oct audit found (car_setup.sh's expected_host).
HOSTS = {'orin': 'ubuntu', 'motor_pi': 'ubuntu-cortex', 'localization': 'localhost.localdomain',
         'dash': 'aav-desktop', 'camera_pi': 'aav-cam1', 'lab_desktop': 'lab'}
LINUX = platform.system() == 'Linux'
# Day A's steps: upgrade is Day B, and refuses on two roles (below).
DAY_A_STEPS = ('backup', 'install', 'services', 'check')


def run(home, *args, stdin='', os_release=None, host=None, reboot=False):
    env = {'PATH': '/usr/bin:/bin:/usr/sbin:/sbin', 'HOME': str(home),
           'AAV_HOSTNAME': host or HOSTS.get(args[0] if args else '', 'lab'),
           'AAV_REBOOT_FLAG': str(home.parent / 'reboot-required')}
    if reboot:
        (home.parent / 'reboot-required').write_text('')
    if os_release:
        path = home.parent / 'os-release'
        path.write_text(os_release)
        env['AAV_OS_RELEASE'] = str(path)
    return subprocess.run(['bash', str(SCRIPT), *args], input=stdin, capture_output=True,
                          text=True, timeout=60, env=env)


@pytest.fixture
def home(tmp_path):
    h = tmp_path / 'home'
    h.mkdir()
    return h


UBUNTU_22 = 'ID=ubuntu\nVERSION_ID="22.04"\nPRETTY_NAME="Ubuntu 22.04.5 LTS"\n'
PI_OS = 'ID=debian\nVERSION_ID="13"\nPRETTY_NAME="Debian GNU/Linux 13 (trixie)"\n'


@pytest.mark.parametrize('role', ROLES)
def test_a_dry_run_prints_every_step_and_changes_nothing(home, role):
    done = run(home, role)
    assert 'Dry run: nothing changes' in done.stdout
    for step in ('backup', 'upgrade', 'install', 'services', 'check'):
        assert f'== {step}' in done.stdout
    assert list(home.iterdir()) == []
    # Only the upgrade's two refusals may stop a whole dry run.
    if role in ('motor_pi', 'localization'):
        assert done.returncode == 1 and 'refused' in done.stdout
    else:
        assert done.returncode == 0, done.stdout + done.stderr


@pytest.mark.parametrize('role', ROLES)
def test_day_a_steps_dry_run_with_no_stop(home, role):
    done = run(home, role, *DAY_A_STEPS)
    stops = re.findall(r'^\s*STOP\s+(\S+)', done.stdout, re.M)
    # A Linux test machine has no Pi UART: check says so, rightly.
    if LINUX and role == 'motor_pi' and not Path('/dev/ttyAMA0').exists():
        assert stops == ['UART'], done.stdout
    else:
        assert stops == [] and done.returncode == 0, done.stdout + done.stderr


def test_a_pending_reboot_is_a_stop(home):
    done = run(home, 'dash', 'check', reboot=True)
    assert done.returncode == 1
    assert re.search(r'STOP\s+reboot', done.stdout)
    applied = run(home, 'dash', 'backup', '--apply', stdin='dash\n', reboot=True)
    assert applied.returncode == 1
    assert 'Refused' in applied.stderr and 'Nothing changed' in applied.stderr
    assert list(home.iterdir()) == []


def test_a_wrong_hostname_is_a_stop_on_linux_unless_host_ok(home):
    done = run(home, 'orin', 'check', host='cam2')
    if LINUX:
        assert done.returncode == 1
        assert re.search(r'STOP\s+hostname', done.stdout) and '--host-ok' in done.stdout
    else:   # a rehearsal on a Mac: only a note
        assert done.returncode == 0 and re.search(r'NOTE\s+hostname', done.stdout)
    ok = run(home, 'orin', 'check', '--host-ok', host='cam2')
    assert ok.returncode == 0 and 'STOP' not in ok.stdout
    assert re.search(r'OK\s+hostname', run(home, 'orin', 'check').stdout)


def test_upgrade_refuses_the_localization_pi_until_its_machine_file_exists(home):
    machines = TOOLS.parent / 'ros' / 'bringup' / 'aav_bringup' / 'config' / 'machines'
    if (machines / 'localization.yaml').is_file():
        pytest.skip('the localization machine file exists now: the refusal is lifted')
    done = run(home, 'localization', 'upgrade', os_release=UBUNTU_22)
    assert done.returncode == 1
    assert 'machines/localization.yaml' in done.stdout and 'IMU and GPS' in done.stdout
    assert 'apt-get full-upgrade' not in done.stdout


def test_upgrade_refuses_the_motor_pi_until_after_the_esp32(home):
    done = run(home, 'motor_pi', 'upgrade', os_release=UBUNTU_22)
    assert done.returncode == 1
    assert '--after-esp32' in done.stdout and 'aav_serial_bridge' in done.stdout
    after = run(home, 'motor_pi', 'upgrade', '--after-esp32', os_release=UBUNTU_22)
    assert after.returncode == 0 and 'apt-get full-upgrade' in after.stdout


@pytest.mark.skipif(not LINUX, reason='the UART check is a STOP only on Linux')
def test_after_the_upgrade_a_missing_uart_is_a_stop(home):
    if Path('/dev/ttyAMA0').exists():
        pytest.skip('this computer has a UART')
    ubuntu_24 = 'ID=ubuntu\nVERSION_ID="24.04"\nPRETTY_NAME="Ubuntu 24.04.3 LTS"\n'
    done = run(home, 'motor_pi', 'check', os_release=ubuntu_24)
    assert done.returncode == 1 and 'after the upgrade' in done.stdout


def test_no_role_or_a_wrong_one_lists_the_roles(home):
    for args in ((), ('jetson',)):
        done = run(home, *args)
        assert done.returncode == 2
        assert all(r in done.stderr for r in ROLES)


def test_a_wrong_step_or_option_is_refused(home):
    assert run(home, 'orin', 'reflash').returncode == 2
    assert run(home, 'orin', '--yes').returncode == 2
    assert run(home, 'orin', '--address', '192.168.1.140').returncode == 2


def test_steps_can_be_picked(home):
    done = run(home, 'orin', 'check')
    assert '== check' in done.stdout and '== backup' not in done.stdout


def test_apply_without_the_role_typed_changes_nothing(home):
    # On Linux: the wrong word stops it. On a Mac: refused before it asks.
    done = run(home, 'motor_pi', 'backup', 'services', '--apply', stdin='orin\n')
    assert done.returncode == 1
    assert 'Nothing changed' in done.stderr or 'Refused' in done.stderr
    assert list(home.iterdir()) == []


def test_ubuntu_22_prints_the_release_upgrade_but_runs_it_only_when_asked(home):
    plain = run(home, 'dash', 'upgrade', os_release=UBUNTU_22).stdout
    assert 'would run: sudo do-release-upgrade' not in plain
    assert 'sudo do-release-upgrade' in plain and 'add --release-upgrade' in plain
    asked = run(home, 'dash', 'upgrade', '--release-upgrade', os_release=UBUNTU_22).stdout
    assert 'would run: sudo do-release-upgrade' in asked
    assert 'port 1022' in asked


def test_the_orin_and_pi_os_never_release_upgrade(home):
    orin = run(home, 'orin', 'upgrade', '--release-upgrade', os_release=UBUNTU_22).stdout
    assert 'do-release-upgrade' in orin and 'would run: sudo do-release-upgrade' not in orin
    assert 'reflash' in orin
    pi = run(home, 'camera_pi', 'upgrade', '--release-upgrade', os_release=PI_OS).stdout
    assert 'would run: sudo do-release-upgrade' not in pi
    assert 'new SD card' in pi


def test_services_by_role(home):
    orin = run(home, 'orin', 'services').stdout
    assert 'allow 192.168.1.0/24' in orin
    assert 'AAV_MACHINE=orin' in orin
    assert 'loginctl enable-linger' in orin
    assert 'systemctl --user enable aav-car' in orin
    pi = run(home, 'motor_pi', 'services', '--machine', 'motor_pi_alone').stdout
    assert 'server 192.168.1.140' in pi and 'AAV_MACHINE=motor_pi_alone' in pi
    for role in ('localization', 'dash', 'camera_pi', 'lab_desktop'):
        assert 'enable aav-car' not in run(home, role, 'services').stdout
    assert 'enable mediamtx' in run(home, 'camera_pi', 'services').stdout


def test_a_fixed_address_goes_beside_dhcp(home):
    out = run(home, 'localization', 'services', '--address', '192.168.1.113/24',
              '--interface', 'eth0', os_release=UBUNTU_22).stdout
    assert 'dhcp4: true' in out and 'addresses: [192.168.1.113/24]' in out
    assert 'netplan try' in out


def test_bash_3_2_only():
    body = SCRIPT.read_text()
    for bash4 in (r'declare -A', r'\$\{\w+,,\}', r'\$\{\w+\^\^\}', r'\bmapfile\b',
                  r'\breadarray\b', r'\|&', r'&>>', r'\bcoproc\b'):
        assert not re.search(bash4, body), bash4
    subprocess.run(['bash', '-n', str(SCRIPT)], check=True)


def test_the_unit_runs_our_stack_from_car_env():
    unit = configparser.ConfigParser(interpolation=None, strict=False)
    unit.read(TOOLS / 'systemd' / 'aav-car.service')
    assert unit['Service']['EnvironmentFile'] == '%h/.config/aav/car.env'
    assert unit['Service']['ExecStart'].startswith(
        '%h/aav-jazzy/aav.sh ros2 launch aav_bringup car.launch.py machine:=${AAV_MACHINE}')
    assert unit['Service']['KillSignal'] == 'SIGINT'
    assert unit['Install']['WantedBy'] == 'default.target'
    env = (TOOLS / 'systemd' / 'car.env').read_text()
    keys = [line.split('=', 1)[0] for line in env.splitlines()
            if line and not line.startswith('#')]
    assert keys == ['AAV_MACHINE', 'AAV_CAR_ARGS']


def test_every_machine_the_units_name_exists():
    machines = TOOLS.parent / 'ros' / 'bringup' / 'aav_bringup' / 'config' / 'machines'
    for name in ('orin', 'motor_pi', 'motor_pi_alone'):
        assert (machines / f'{name}.yaml').is_file()
