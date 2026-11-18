from aav_sim_models import sources


def names(entries):
    return [n for n, _ in entries]


def test_the_empty_world_has_the_basic_sensors_only():
    got = names(sources.simulated(rendered=False, lidar=True))
    assert got == ['vehicle', 'odometry', 'wheels', 'imu', 'gps']


def test_the_road_world_adds_cameras_and_radar_and_lidar_when_asked():
    plain = names(sources.simulated(rendered=True, lidar=False))
    assert 'radar' in plain and 'camera_rear' in plain and 'lidar' not in plain
    assert names(sources.simulated(rendered=True, lidar=True))[-1] == 'lidar'


def test_every_topic_is_absolute_and_in_an_area():
    for _, topic in sources.simulated(rendered=True, lidar=True):
        assert topic.split('/')[1] in ('vehicle', 'sensors'), topic


def test_radar_false_leaves_only_the_radar_out():
    got = names(sources.simulated(rendered=True, lidar=True, radar=False))
    assert 'radar' not in got
    assert 'camera_front_left' in got and 'lidar' in got


def test_with_the_spine_in_the_loop_spine_bridge_announces_the_car():
    got = names(sources.simulated(rendered=True, lidar=True, vehicle=False))
    assert not {'vehicle', 'odometry', 'wheels'} & set(got)
    assert 'imu' in got and 'gps' in got and 'camera_front_left' in got


def test_without_cameras_only_the_rays_are_announced():
    topics = {t for _, t in sources.simulated(rendered=True, lidar=True, cameras=False)}
    assert {'/sensors/radar/points', '/sensors/lidar/points'} <= topics
    assert not any('camera' in t for t in topics)
