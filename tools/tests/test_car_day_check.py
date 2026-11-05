"""Tests for tools/car_day_check.py: the laptop's preflight before car day."""

from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import car_day_check as cdc  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
HEAD = 'f5730bef0123456789abcdef0123456789abcdef'


# --- the code --------------------------------------------------------------------------

def test_the_same_commit_as_github_is_ok():
    line = cdc.repo_line('main', HEAD, HEAD, 'same', dirty=False)
    assert line.status == 'OK'
    assert 'f5730bef' in line.detail


def test_behind_github_says_update_from_upstream():
    line = cdc.repo_line('main', HEAD, 'a' * 40, 'behind', dirty=False)
    assert (line.status, line.fix) == ('MISSING', cdc.UPDATE_MAIN)
    assert 'upstream/main' in line.fix


def test_ahead_of_github_says_test_then_pull_request_never_push_main():
    line = cdc.repo_line('main', HEAD, 'a' * 40, 'ahead', dirty=False)
    assert line.status == 'MISSING'
    assert 'pixi run test' in line.fix and 'pull request' in line.fix
    assert 'git push' not in line.fix


def test_another_branch_or_uncommitted_changes_are_missing():
    assert cdc.repo_line('wt/x', HEAD, HEAD, 'same', False).status == 'MISSING'
    line = cdc.repo_line('main', HEAD, HEAD, 'same', dirty=True)
    assert line.status == 'MISSING'
    assert 'uncommitted' in line.detail


def test_no_internet_is_a_reminder_not_a_failure():
    line = cdc.repo_line('main', HEAD, None, 'apart', dirty=False)
    assert line.status == 'REMIND'


EVENTS_OK = ("[0.6] (aav_ego) JobEnded: {'identifier': 'aav_ego', 'rc': 0}\n"
             "[0.7] (aav_command_mux) JobEnded: {'identifier': 'aav_command_mux', 'rc': 0}\n")


def test_a_build_after_the_last_change_is_ok():
    line = cdc.build_line(EVENTS_OK, built_at=2000, newest_src=1000)
    assert line.status == 'OK'
    assert '2 packages' in line.detail


def test_a_change_after_the_build_says_build_again():
    line = cdc.build_line(EVENTS_OK, built_at=1000, newest_src=2000)
    assert (line.status, line.fix) == ('MISSING', 'pixi run build')


def test_a_failed_package_is_named():
    events = EVENTS_OK + "[1.0] (aav_hud) JobEnded: {'identifier': 'aav_hud', 'rc': 2}\n"
    line = cdc.build_line(events, built_at=2000, newest_src=1000)
    assert line.status == 'MISSING'
    assert '1 failed' in line.detail and 'aav_hud' in line.detail


def test_never_built():
    assert cdc.build_line(None, 0, 0).fix == 'pixi run build'


KIT = """AAV car kit, made 2026-10-20 13:54 on Bens-MacBook-Pro.local
commit:     f5730bef Merge remote-tracking branch 'origin/wt/arm-gate'
branch:     main
lock file:  pixi.lock ef172a1478b0
"""


def test_a_kit_from_this_commit_and_lock_is_ok():
    line = cdc.kit_line(KIT, HEAD, 'ef172a1478b0', ['rf-detr-nano.pth', 'yolo11n.pt'])
    assert line.status == 'OK'


def test_an_old_kit_says_what_is_old():
    line = cdc.kit_line(KIT, 'b' * 40, '000000000000', [])
    assert line.status == 'MISSING'
    assert 'its code is f5730bef' in line.detail
    assert 'pixi.lock changed' in line.detail
    assert 'no RF-DETR model' in line.detail
    assert line.fix == 'pixi run car-kit'


def test_no_kit():
    assert cdc.kit_line(None, HEAD, 'x', []).status == 'MISSING'


def test_the_stick_has_this_kit():
    line = cdc.stick_line(KIT, {'/Volumes/AAV': KIT, '/Volumes/OLD': 'other'})
    assert line.status == 'OK'
    assert '/Volumes/AAV' in line.detail
    assert cdc.stick_line(KIT, {'/Volumes/OLD': 'other'}).detail.startswith('an older kit')
    assert cdc.stick_line(KIT, {}).status == 'REMIND'


# --- the ESP32 -------------------------------------------------------------------------

BUILD = {'made': '24 Oct 2026 16:00', 'firmware/spine': 'tree1',
         cdc.LEGACY: 'tree2', 'v2': 'PASS', 'v1': 'PASS',
         'esp qemu-check': 'PASS', 'legacy-esp32': 'PASS'}
FILES = list(cdc.FIRMWARES) + ['BUILD.txt', 'qemu-check.log']


def test_all_firmware_built_from_todays_source_is_ok():
    lines = cdc.firmware_lines(BUILD, FILES, 'tree1', 'tree2')
    assert [ln.status for ln in lines] == ['OK', 'OK', 'OK']
    assert 'v2, v1 and v2-uplink' in lines[0].detail


def test_a_firmware_change_since_makes_the_rust_and_qemu_out_of_date():
    lines = cdc.firmware_lines(BUILD, FILES, 'tree9', 'tree2')
    assert [ln.status for ln in lines] == ['MISSING', 'MISSING', 'OK']
    assert all(ln.fix == 'pixi run car-day-build' for ln in lines[:2])


def test_uncommitted_firmware_is_never_up_to_date():
    build = {**BUILD, 'firmware/spine': 'tree1-dirty'}
    lines = cdc.firmware_lines(build, FILES, 'tree1-dirty', 'tree2')
    assert lines[0].status == 'MISSING'


def test_a_missing_link_version_is_named():
    lines = cdc.firmware_lines(BUILD, ['spine-esp32-umod-v2.elf'], 'tree1', 'tree2')
    assert lines[0].status == 'MISSING'
    assert 'spine-esp32-umod-v1.elf' in lines[0].detail


def test_a_failed_or_skipped_qemu_check_is_missing():
    failed = cdc.firmware_lines({**BUILD, 'esp qemu-check': 'FAIL (exit 1)'}, FILES,
                                'tree1', 'tree2')
    assert failed[1].status == 'MISSING' and 'FAIL' in failed[1].detail
    skipped = cdc.firmware_lines({**BUILD, 'esp qemu-check': 'skipped: no QEMU here'},
                                 FILES, 'tree1', 'tree2')
    assert 'esp qemu-tools' in skipped[1].fix


def test_nothing_built_yet():
    lines = cdc.firmware_lines({}, [], 'tree1', 'tree2')
    assert [ln.status for ln in lines] == ['MISSING'] * 3


def test_build_txt_is_read_as_key_value_lines():
    text = 'made: 24 Oct 2026 16:00\ncommit: f5730bef Merge: a, b\nv2: PASS\n'
    got = cdc.parse_build_txt(text)
    assert got['commit'] == 'f5730bef Merge: a, b'
    assert got['v2'] == 'PASS'


def test_the_build_script_writes_the_keys_the_check_reads():
    script = (ROOT / 'tools/car_day_build.sh').read_text()
    for key in ('firmware/spine:', '$LEGACY:', 'esp qemu-check:', 'legacy-esp32:',
                'made:'):
        assert f'\n{key} ' in script
    for name in cdc.FIRMWARES:
        assert f'$OUT/{name}' in script
    assert cdc.ESP32_DIR == 'car-day/esp32' and 'OUT=car-day/esp32' in script


def test_esp_tools_need_all_three(tmp_path):
    line = cdc.check_esp_tools(tmp_path, 'stable-aarch64-apple-darwin\nesp\n')
    assert line.status == 'MISSING'
    (tmp_path / 'vendor/esp').mkdir(parents=True)
    (tmp_path / 'vendor/esp/export-esp.sh').write_text('')
    (tmp_path / 'vendor/cargo-tools/bin').mkdir(parents=True)
    (tmp_path / 'vendor/cargo-tools/bin/espflash').write_text('')
    assert cdc.check_esp_tools(tmp_path, 'stable\nesp\n').status == 'OK'
    assert cdc.check_esp_tools(tmp_path, 'stable\nespresso\n').status == 'MISSING'
    assert cdc.check_esp_tools(tmp_path, None).status == 'MISSING'


def test_secrets_are_compared_never_shown():
    example = b'#define WIFI_PASSWORD "changeme"\n'
    line = cdc.secrets_line(b'#define WIFI_PASSWORD "hunter2"\n', example)
    assert line.status == 'OK'
    assert 'hunter2' not in line.detail + line.fix
    assert cdc.secrets_line(example, example).status == 'REMIND'
    assert cdc.secrets_line(None, example).status == 'REMIND'


# --- the rest --------------------------------------------------------------------------

def test_docs_pdf_older_than_a_page_is_missing():
    assert cdc.docs_line(None, 5).fix == 'pixi run docs'
    assert cdc.docs_line(10, 20).status == 'MISSING'
    assert cdc.docs_line(20, 10).status == 'OK'


PIXI = 'RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"\n'
MINE = """<CycloneDDS><Domain id="any"><General><Interfaces>
  <NetworkInterface name="en7" priority="default" multicast="default"/>
</Interfaces></General></Domain></CycloneDDS>"""


def test_cyclone_with_this_laptops_card_named_is_ok():
    line = cdc.cyclone_line(PIXI, MINE, None)
    assert line.status == 'OK'
    assert 'en7' in line.detail and 'in use now' in line.detail


def test_cyclone_kept_aside_for_the_car_is_ok_and_says_copy_it():
    line = cdc.cyclone_line(PIXI, None, MINE)
    assert line.status == 'OK'
    assert 'en7' in line.detail and 'cp it to' in line.detail


def test_cyclone_without_a_card_is_a_reminder():
    assert cdc.cyclone_line(PIXI, None, None).status == 'REMIND'
    assert cdc.cyclone_line(PIXI, '<CycloneDDS/>', None).status == 'REMIND'
    assert cdc.cyclone_line('RMW_IMPLEMENTATION = "x"', MINE, None).status == 'MISSING'


def test_the_repos_pixi_toml_picks_cyclone():
    line = cdc.cyclone_line((ROOT / 'pixi.toml').read_text(), None, None)
    assert line.status == 'REMIND' and 'Cyclone DDS' in line.detail


def test_the_runbooks_cyclone_file_names_a_card_the_check_reads():
    # The runbook's example is what the team copies: the check must understand it.
    runbook = (ROOT / 'docs/car-day.md').read_text()
    xml = re.search(r'(<CycloneDDS.*?</CycloneDDS>)', runbook, re.S).group(1)
    assert cdc.card_named(xml) == 'en7'
    assert cdc.CAR_CYCLONE.replace('~/', '') in runbook


HOSTS = """# label  user@ip  keychain entry
CAM2-PI   someone@192.168.1.118   aav-cam2
JETSON    someone@192.168.1.140   aav-jetson   # the Orin
WIFI-ESP  192.168.1.132
"""


def test_keychain_entries_come_from_the_third_column():
    assert cdc.keychain_entries(HOSTS) == [('CAM2-PI', 'aav-cam2'), ('JETSON', 'aav-jetson')]


def test_keychain_names_only_the_missing_entries_never_a_user():
    entries = cdc.keychain_entries(HOSTS)
    line = cdc.keychain_line(entries, {'aav-cam2'})
    assert line.status == 'MISSING'
    assert 'aav-jetson' in line.detail and 'aav-cam2' not in line.detail
    assert 'someone' not in line.detail + line.fix
    assert '-w' in line.fix   # it asks, so the password isn't in the shell's history
    ok = cdc.keychain_line(entries, {'aav-cam2', 'aav-jetson'})
    assert ok.status == 'OK' and '2 of 2' in ok.detail
    assert cdc.keychain_line([], set()).status == 'MISSING'


def test_the_keychain_is_only_asked_whether_an_entry_exists():
    source = (ROOT / 'tools/car_day_check.py').read_text()
    calls = [ln for ln in source.splitlines() if "'find-generic-password'" in ln]
    assert calls and all("'-w'" not in ln for ln in calls)


def test_disk():
    assert cdc.disk_line(5e9).status == 'MISSING'
    assert cdc.disk_line(100e9).status == 'OK'


def test_show_counts_and_prints_the_fix(capsys):
    lines = [cdc.Line('OK', 'a', 'fine'), cdc.Line('MISSING', 'b', 'gone', 'make it'),
             cdc.Line('REMIND', 'c', 'tick it', 'by hand')]
    assert cdc.show(lines) == 1
    out = capsys.readouterr().out
    assert '-> make it' in out and '-> by hand' in out
    assert '1 OK, 1 missing, 1 to tick by hand.' in out


def test_every_line_has_a_known_status_on_this_checkout():
    # The whole run, on this repository: nothing may crash, whatever is installed.
    lines = cdc.checks(ROOT)
    assert {ln.status for ln in lines} <= set(cdc.STATUSES)
    assert all(ln.fix for ln in lines if ln.status == 'MISSING')


def test_the_car_needs_its_pixi_environments_installed_at_home(tmp_path):
    (tmp_path / '.pixi' / 'envs' / 'gst' / 'conda-meta').mkdir(parents=True)
    gst, calib = cdc.check_envs(tmp_path)
    assert gst.status == 'OK'
    assert calib.status == 'MISSING' and calib.fix == 'pixi install -e calib'
