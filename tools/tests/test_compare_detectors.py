"""Tests for tools/compare_detectors.py and tools/sim_frames.py: no models, no network."""

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import compare_detectors as cd  # noqa: E402
import sim_frames  # noqa: E402


def test_class_names_from_every_model():
    assert cd.canonical('person') == 'person'
    assert cd.canonical('Stop Sign') == 'stop sign'
    assert cd.canonical('stop_sign') == 'stop sign'
    assert cd.canonical(' stop  sign ') == 'stop sign'
    assert cd.canonical('car') is None
    assert cd.canonical(11) is None          # a bare COCO number is not a name


def test_best_scores_and_counts():
    found = [('person', 0.4), ('person', 0.9), ('stop sign', 0.55), ('car', 0.99)]
    assert cd.best_scores(found) == {'person': 0.9, 'stop sign': 0.55}
    assert cd.best_scores([]) == {'person': 0.0, 'stop sign': 0.0}
    assert cd.counts(found) == {'person': 1, 'stop sign': 1}
    assert cd.counts(found, threshold=0.3) == {'person': 2, 'stop sign': 1}


def test_detr_logits_need_a_sigmoid_and_known_labels():
    labels = {1: 'person', 13: 'stop sign'}
    logit_09 = math.log(0.9 / 0.1)
    row = [0.0] * 14
    row[1], row[13], row[3] = logit_09, -5.0, 10.0    # index 3 (car) is not asked for
    found = cd.detr_detections([row], labels, threshold=0.5)
    assert len(found) == 1
    assert found[0][0] == 'person'
    assert math.isclose(found[0][1], 0.9)


def picture(name, person=0.0, sign=0.0, ms=(10.0, 12.0, 30.0)):
    return {'image': name, 'best': {'person': person, 'stop sign': sign},
            'n': {}, 'ms': list(ms)}


def test_summary_counts_hits_misses_and_false_finds():
    expect = {'a': ['person'], 'b': ['stop sign'], 'c': [], 'd': ['person', 'stop sign']}
    per_image = [picture('a', person=0.8), picture('b', sign=0.3, person=0.6),
                 picture('c', sign=0.7), picture('d', person=0.2, sign=0.9),
                 picture('unlisted', person=0.9)]
    row = cd.summarise('m', 'cpu', per_image, expect)
    assert row['person'] == {'found': 1, 'of': 2, 'mean_best': 0.5, 'false': 1}
    assert row['stop sign'] == {'found': 1, 'of': 2, 'mean_best': 0.6, 'false': 1}
    assert row['ms'] == 12.0        # the median of each picture's times, then their mean
    assert cd.cell(row['person']) == '1/2 (0.50)'
    assert cd.cell({'found': 0, 'of': 0, 'mean_best': None, 'false': 0}) == '-'


def test_gazebo_frames_and_how_far_the_sign_is_seen():
    assert cd.frame_distance('road_sign_15.0m.png') == 15.0
    assert cd.frame_distance('road_sign_07.0m') == 7.0
    assert cd.frame_distance('dashcam-stop') is None
    assert cd.frame_distance('road_sign_far.png') is None
    distances = {'road_sign_20.0m': 20.0, 'road_sign_10.0m': 10.0, 'road_sign_05.0m': 5.0}
    per_image = [picture('road_sign_20.0m', sign=0.4), picture('road_sign_10.0m', sign=0.6),
                 picture('road_sign_05.0m', sign=0.9), picture('photo', sign=0.99)]
    assert cd.frames_reach(per_image, distances) == 10.0
    assert cd.frames_reach(per_image[:1], distances) is None


def test_tables_are_markdown():
    expect = {'a': ['person']}
    rows = [cd.summarise('yolo11n', 'cpu', [picture('a', person=0.8)], expect)]
    text = cd.main_table(rows, {'yolo11n': '6 MB file'}, {'yolo11n': 'AGPL-3.0'},
                         {('yolo11n', 'cpu'): 15.0})
    lines = text.splitlines()
    assert lines[0].startswith('| Model | Device |')
    assert len({line.count('|') for line in lines}) == 1
    assert '| yolo11n | cpu | 1/1 (0.80) | - | 0 | 15 m | 12.0 | 6 MB file | AGPL-3.0 |' in text
    per = cd.image_table({('yolo11n', 'cpu'): [picture('a', person=0.8)]}, expect)
    assert '| a | person | 0.80 / 0.00 |' in per


def test_sizes():
    assert cd.size_words(5.6, None) == '6 MB file'
    assert cd.size_words(366.0, 30_500_000) == '366 MB file; 30.5 M weights (122 MB at fp32)'


def test_photos_are_credited_and_freely_licensed():
    free = {'CC0', 'CC BY 2.0', 'CC BY 4.0', 'CC BY-SA 2.0', 'CC BY-SA 4.0'}
    names = [p['name'] for p in cd.PHOTOS]
    assert len(names) == len(set(names))
    for p in cd.PHOTOS:
        assert p['title'].startswith('File:')
        assert p['licence'] in free, p
        assert p['author'] and p['author'] != '?', p
        assert set(p['expect']) <= set(cd.CLASSES)
    assert any(not p['expect'] for p in cd.PHOTOS)     # one picture with nothing in it


def test_commons_query_and_jpeg_check():
    url = cd.commons_query_url(['File:A b.jpg', 'File:C.jpg'])
    assert url.startswith('https://commons.wikimedia.org/w/api.php?')
    assert 'iiurlwidth=640' in url and 'File%3AA+b.jpg%7CFile%3AC.jpg' in url
    assert cd.looks_like_jpeg(b'\xff\xd8\xff\xe0' + b'x' * 10 + b'\xff\xd9')
    assert not cd.looks_like_jpeg(b'<html>not a picture</html>')


def test_every_model_is_pinned_and_licensed():
    for name, spec in cd.MODELS.items():
        assert spec['family'] in ('ultralytics', 'rfdetr'), name
        assert ('sha256' in spec) != ('md5' in spec), name
        assert spec['url'].startswith('https://'), name
        assert spec['licence'], name
    assert cd.MODELS['yolo11n']['sha256'] == (
        '0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1')  # get_models.sh


def test_checked_weights_refuses_a_wrong_file(tmp_path):
    (tmp_path / 'weights').mkdir()
    (tmp_path / 'weights' / 'yolo11n.pt').write_bytes(b'not the model')
    try:
        cd.checked_weights(tmp_path, 'yolo11n')
    except SystemExit as e:
        assert 'not the expected download' in str(e)
    else:
        raise AssertionError('a wrong file was accepted')


def test_sim_frames_puts_the_camera_at_each_distance():
    for d in sim_frames.DISTANCES:
        assert math.isclose(sim_frames.car_x(d) + sim_frames.CAMERA_X + d, sim_frames.SIGN_X)
