"""
Tests for the Foxglove/Lichtblick layouts in tools/foxglove/.

Every topic they show is one the contract has, and one the bridge's WiFi list lets
through.
"""

import json
from pathlib import Path
import re
import struct
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import layout_check  # noqa: E402, I100

ROOT = Path(__file__).resolve().parents[2]
TOPICS = yaml.safe_load((ROOT / 'docs/interfaces.yaml').read_text())['topics']
BRIDGE = yaml.safe_load((ROOT / 'ros/bringup/aav_bringup/config/foxglove_bridge.yaml').read_text())
WIFI = BRIDGE['foxglove_bridge']['ros__parameters']['topic_whitelist']
LAYOUTS = {p.stem: json.loads(p.read_text()) for p in layout_check.LAYOUTS}


SERVICES = set(yaml.safe_load((ROOT / 'docs/interfaces.yaml').read_text())['services'])


def test_the_four_layouts_are_there():
    assert set(LAYOUTS) == {'driving', 'sensors-health', 'map', 'cockpit'}


@pytest.mark.parametrize('name', sorted(LAYOUTS))
def test_every_topic_is_in_the_contract(name):
    shown = layout_check.shown_topics(LAYOUTS[name])
    assert shown, 'the layout shows no topics'
    unknown = shown - set(TOPICS) - layout_check.NAV2_OWN
    assert not unknown, f'not in docs/interfaces.yaml: {sorted(unknown)}'


@pytest.mark.parametrize('name', sorted(LAYOUTS))
def test_every_topic_gets_through_the_wifi_list(name):
    # The car's bridge leaves the big topics out: a layout that shows one shows nothing.
    blocked = {t for t in layout_check.shown_topics(LAYOUTS[name])
               if not any(re.fullmatch(p, t) for p in WIFI)}
    assert not blocked, f'the bridge does not send these over WiFi: {sorted(blocked)}'


@pytest.mark.parametrize('name', sorted(LAYOUTS))
def test_every_panel_in_the_layout_has_its_settings(name):
    def leaves(node):
        if isinstance(node, str):
            yield node
            # A Tab panel holds a layout per tab: its panels are panels of the screen too.
            for tab in layout['configById'].get(node, {}).get('tabs', []):
                yield from leaves(tab['layout'])
        else:
            yield from leaves(node['first'])
            yield from leaves(node['second'])
    layout = LAYOUTS[name]
    assert sorted(leaves(layout['layout'])) == sorted(layout['configById'])


def test_the_optional_topics_are_ones_a_layout_shows():
    shown = set().union(*(layout_check.shown_topics(x) for x in LAYOUTS.values()))
    assert set(layout_check.OPTIONAL) <= shown
    assert layout_check.NAV2_OWN <= shown


def test_the_driving_layout_shows_the_cameras_preview_not_its_pictures():
    shown = layout_check.shown_topics(LAYOUTS['driving'])
    assert '/perception/camera/front_left/preview/compressed' in shown
    assert not any(t.endswith('/image_raw') or t.endswith('/image') for t in shown)


def test_the_cockpit_has_every_part_on_one_screen():
    layout = LAYOUTS['cockpit']
    kinds = {panel.split('!')[0] for panel in layout['configById']}
    assert {'3D', 'Image', 'map', 'Plot', 'Indicator', 'DiagnosticSummary', 'CallService',
            'Table'} <= kinds
    shown = layout_check.shown_topics(layout)
    assert {'/vehicle/gazebo/chase_camera/compressed', '/perception/voxels/cubes',
            '/localization/health', '/plan', '/goal_pose', '/diagnostics'} <= shown


def test_the_cockpits_3d_panel_sends_goals_where_nav2_listens():
    config = LAYOUTS['cockpit']['configById']['3D!cockpit']
    assert config['publish']['type'] == 'pose'
    assert config['publish']['poseTopic'] == '/goal_pose'


def test_the_cockpit_swaps_the_tracker_through_its_parameter_service():
    config = LAYOUTS['cockpit']['configById']['CallService!tracker']
    assert layout_check.shown_services(LAYOUTS['cockpit']) <= SERVICES
    request = json.loads(config['requestPayload'])
    (param,) = request['parameters']
    assert param['name'] == 'tracker'
    assert param['value']['type'] == 4                 # PARAMETER_STRING
    assert param['value']['string_value'] in ('pure_pursuit', 'stanley', 'lqr', 'mpc',
                                              'mpc_full')


def test_the_launch_switches_script_reads_a_topic_every_run_has():
    layout = LAYOUTS['cockpit']
    script = layout['userNodes']['launch-switches']
    assert layout_check.script_inputs({'userNodes': {'s': script}}) == {'/diagnostics'}
    for switch in ('planner', 'ndt', 'ndt_ekf', 'kiss_icp', 'kiss_ekf', 'voxels', 'amcl'):
        assert f'"{switch}:="' in script['sourceCode']
    assert 'pixi run pick' in script['sourceCode']
    # The Table shows the script's output: the app's own topic, never the bridge's.
    assert layout['configById']['Table!switches']['topicPath'].startswith(
        layout_check.SCRIPT_PREFIX)


def test_the_cockpit_shows_which_algorithms_run():
    config = LAYOUTS['cockpit']['configById']['Table!algorithms']
    assert config['topicPath'] == '/system/algorithms.values'
    assert TOPICS['/system/algorithms']['type'] == 'diagnostic_msgs/msg/DiagnosticStatus'


def test_the_cockpit_shows_every_localizer():
    layout = LAYOUTS['cockpit']
    shown = layout_check.shown_topics(layout)
    assert {'/localization/odom/local', '/localization/odom/global', '/localization/ndt/pose',
            '/localization/ndt/status', '/localization/kiss_icp/odom',
            '/localization/kiss_icp/status', '/localization/amcl/pose', '/map'} <= shown
    # The 3D panel draws no Odometry: the EKFs' arrows and NDT's scan come from scripts.
    topics = layout['configById']['3D!cockpit']['topics']
    for script in ('ekf_poses', 'ndt_scan'):
        assert f'{layout_check.SCRIPT_PREFIX}{script}' in topics


def test_the_chase_cameras_raw_picture_stays_off_the_wifi():
    raw = '/vehicle/gazebo/chase_camera/image_raw'
    assert raw in TOPICS
    assert not any(re.fullmatch(p, raw) for p in WIFI)
    assert TOPICS['/vehicle/gazebo/chase_camera/compressed']['live'] == ['road']


def test_an_unknown_panel_is_refused():
    with pytest.raises(ValueError, match='Gauge!x'):
        layout_check.shown_topics({'configById': {'Gauge!x': {}}})


def test_the_bridge_topics_come_from_its_advertise_messages():
    payloads = [
        json.dumps({'op': 'serverInfo', 'name': 'foxglove_bridge'}),
        json.dumps({'op': 'advertise', 'channels': [{'id': 1, 'topic': '/vehicle/odom'},
                                                    {'id': 2, 'topic': '/plan'}]}),
        json.dumps({'op': 'advertise', 'channels': [{'id': 3, 'topic': '/tf'}]}),
        json.dumps({'op': 'unadvertise', 'channelIds': [2]}),
    ]
    assert layout_check.advertised(payloads) == {'/vehicle/odom': 1, '/tf': 3}


def test_the_bridge_services_come_from_its_advertise_services_messages():
    services = set()
    payloads = [json.dumps({'op': 'advertiseServices', 'services': [
        {'id': 1, 'name': '/track/set_parameters', 'type': 'rcl_interfaces/srv/SetParameters'}]})]
    assert layout_check.advertised(payloads, services) == {}
    assert services == {'/track/set_parameters'}


def test_message_data_is_the_subscription_and_the_message_size():
    frame = bytes([0x01]) + struct.pack('<IQ', 7, 123) + b'abcde'
    assert layout_check.message_data(frame) == (7, 5)
    assert layout_check.message_data(bytes([0x02, 0, 0])) is None   # another op
