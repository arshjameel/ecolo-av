import importlib.util
import os

from aav_detector.boxes import Box
from aav_detector.detectors import keep_labels, load_detector, RfdetrExported
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))   # the repository
PTH = os.path.join(ROOT, 'models', 'rf-detr-nano.pth')
needs_cv2 = pytest.mark.skipif(importlib.util.find_spec('cv2') is None, reason='no OpenCV')
# The real model: only in the ml environment, after pixi run get models.
needs_rfdetr = pytest.mark.skipif(
    not (importlib.util.find_spec('rfdetr') and importlib.util.find_spec('cv2')
         and os.path.exists(PTH)),
    reason='needs the ml environment (rfdetr) and models/rf-detr-nano.pth')


def test_keep_labels():
    assert keep_labels(None) is None
    assert keep_labels('') is None
    assert keep_labels('Stop_Sign, person') == ['stop sign', 'person']
    assert keep_labels(['STOP sign']) == ['stop sign']
    assert keep_labels([]) is None


class Fake(RfdetrExported):
    """An exported RF-DETR that always answers the same: a stop sign and a person."""

    height = width = 384
    names = {1: 'person', 13: 'stop sign'}

    def __init__(self):
        self.seen = None

    def run(self, picture):
        self.seen = picture.shape
        logits = np.full((1, 300, 91), -20.0, dtype=np.float32)
        logits[0, 0, 13], logits[0, 1, 1] = 3.0, 1.0       # 0.95 and 0.73
        boxes = np.zeros((1, 300, 4), dtype=np.float32)
        boxes[0, 0], boxes[0, 1] = [0.5, 0.5, 0.25, 0.5], [0.1, 0.9, 0.1, 0.2]
        return {'concat_2': boxes, 'linear_102': logits}   # Core ML's names


@needs_cv2
def test_an_exported_model_gives_boxes_in_the_pictures_pixels():
    fake = Fake()
    boxes = fake.detect(np.zeros((360, 640, 3), dtype=np.uint8))
    assert fake.seen == (1, 3, 384, 384)                   # resized to the model's square
    assert [b.label for b in boxes] == ['stop sign', 'person']
    sign = boxes[0]
    assert (sign.cx, sign.cy, sign.width, sign.height) == pytest.approx((320, 180, 160, 180))
    assert sign.score == pytest.approx(0.9526, abs=1e-4)


@needs_cv2
def test_an_exported_model_keeps_the_confidence_and_labels_asked_for():
    picture = np.zeros((360, 640, 3), dtype=np.uint8)
    assert [b.label for b in Fake().detect(picture, confidence=0.8)] == ['stop sign']
    assert [b.label for b in Fake().detect(picture, labels='person')] == ['person']
    assert Fake().detect(picture, labels=['bus']) == []


def test_load_detector_says_what_is_wrong_before_loading_anything(tmp_path):
    with pytest.raises(ValueError, match='use rfdetr or yolo'):
        load_detector('detr')
    with pytest.raises(ValueError, match='use detector:=yolo'):
        load_detector('rfdetr', 'models/yolo11n.engine')
    with pytest.raises(FileNotFoundError, match='pixi run get models'):
        load_detector('rfdetr', 'models/rf-detr-none.pth', root=str(tmp_path))
    (tmp_path / 'rf-detr-nano.engine').write_text('')
    with pytest.raises(ValueError, match="isn't a device name"):
        load_detector('rfdetr', 'rf-detr-nano.engine', device='gpu', root=str(tmp_path))


@needs_rfdetr
def test_rfdetr_nano_finds_the_test_stop_sign():
    import cv2
    model = load_detector('rfdetr', root=ROOT)
    picture = cv2.imread(os.path.join(HERE, 'stop_sign.jpg'))
    boxes = model.detect(picture, 0.5, 'stop sign')
    assert len(boxes) == 1 and boxes[0].score > 0.9
    assert isinstance(boxes[0], Box) and 230 < boxes[0].height < 260
