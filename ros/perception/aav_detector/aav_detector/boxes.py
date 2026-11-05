"""
The detectors' boxes as plain values, which labels to keep, and where the model runs.

Two detectors, picked by the detector parameter:
  rfdetr  RF-DETR (Roboflow, Apache-2.0), a transformer: the default since 20 Oct 2026, and
          the car's (Ben, 23 Oct). It saw Gazebo's stop sign from 20 m, YOLO from 10 m
          (docs/model-comparison.md). .pth (PyTorch), .mlpackage (Core ML) or .engine
          (TensorRT), the last two named after it: rf-detr-nano.mlpackage
  yolo   YOLO (Ultralytics, AGPL-3.0): .pt, .mlpackage (Core ML) or .engine (TensorRT)

No ROS or PyTorch imports, so the tests run anywhere.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import re
from typing import Dict, List, Mapping, Optional, Sequence

# Each detector's default model file (pixi run get models downloads both), and the file
# types it can load.
DEFAULT_MODELS = {
    'rfdetr': 'models/rf-detr-nano.pth',
    'yolo': 'models/yolo11n.pt',
}
MODEL_TYPES = {
    'rfdetr': ('.pth', '.mlpackage', '.engine'),
    'yolo': ('.pt', '.mlpackage', '.engine'),
}
# RF-DETR's sizes, as the rfdetr library names their classes. The size is in the file's
# name: rf-detr-nano.pth, rf-detr-small.pth...
RFDETR_SIZES = {'nano': 'RFDETRNano', 'small': 'RFDETRSmall', 'medium': 'RFDETRMedium',
                'base': 'RFDETRBase', 'large': 'RFDETRLarge'}
# An exported model (.mlpackage, .engine) is RF-DETR's when its name says so.
RFDETR_NAME = re.compile(r'rf[-_]?detr', re.IGNORECASE)


def family(model: str) -> Optional[str]:
    """
    Return which detector a model file is for, from its name: 'rfdetr', 'yolo' or None.

    A .pth is RF-DETR's and a .pt YOLO's. A .mlpackage or an .engine can be either: it is
    RF-DETR's when its name has rf-detr in it (rf-detr-nano.engine), else YOLO's
    (yolo11n.engine).
    """
    name = os.path.basename(model.rstrip('/'))
    if name.endswith('.pth'):
        return 'rfdetr'
    if name.endswith('.pt'):
        return 'yolo'
    if name.endswith(('.mlpackage', '.engine')):
        return 'rfdetr' if RFDETR_NAME.search(name) else 'yolo'
    return None


def check_detector(detector: str, model: str) -> str:
    """
    Return the model file to load: `model`, or the detector's default when it's empty.

    Raises ValueError, saying what to use instead, for an unknown detector or a model file
    that detector can't load.
    """
    name = detector.strip().lower()
    if name not in DEFAULT_MODELS:
        raise ValueError(f"detector '{detector}' isn't one we have: use "
                         f"{' or '.join(DEFAULT_MODELS)}")
    model = model or DEFAULT_MODELS[name]
    other = family(model)
    if other != name:
        hint = f': it is a {other} model, so use detector:={other}' if other else ''
        named = ', named rf-detr-...' if name == 'rfdetr' else ''
        raise ValueError(f"{name} can't load {model} (it takes "
                         f"{', '.join(MODEL_TYPES[name])}{named}){hint}")
    return model


def rfdetr_size(model: str) -> str:
    """Return the rfdetr class for a weights file, from the size in its name."""
    words = re.split(r'[^a-z]+', os.path.basename(model).lower())
    found = [RFDETR_SIZES[w] for w in words if w in RFDETR_SIZES]
    if len(found) != 1:
        raise ValueError(f'{model}: name the file after its size, such as rf-detr-nano.pth '
                         f"({', '.join(RFDETR_SIZES)})")
    return found[0]


@dataclass
class Box:
    """One detection, in pixels of the image it was found in."""

    label: str
    score: float
    cx: float       # centre
    cy: float
    width: float
    height: float


def normal_label(label: str) -> str:
    """'Stop_Sign' (the 2024-25 model) and 'stop sign' (the stock model) are the same label."""
    return ' '.join(label.replace('_', ' ').replace('-', ' ').lower().split())


def parse_labels(text: str) -> Optional[List[str]]:
    """Read the labels parameter: 'stop sign, person' -> both; '' -> None (keep them all)."""
    labels = [normal_label(s) for s in text.split(',') if s.strip()]
    return labels or None


def boxes_from_result(result, labels: Optional[Sequence[str]] = None) -> List[Box]:
    """
    Return the boxes in one Ultralytics result, keeping only `labels` if given.

    result.boxes holds xywh (centre x, centre y, width, height), conf and cls, as tensors;
    result.names maps a class number to its label.
    """
    keep = {normal_label(s) for s in labels} if labels else None
    boxes = result.boxes
    out = []
    for (cx, cy, w, h), score, cls in zip(boxes.xywh.tolist(), boxes.conf.tolist(),
                                          boxes.cls.tolist()):
        label = result.names[int(cls)]
        if keep is None or normal_label(label) in keep:
            out.append(Box(label, float(score), float(cx), float(cy), float(w), float(h)))
    return out


def boxes_from_corners(xyxy: Sequence[Sequence[float]], scores: Sequence[float],
                       class_ids: Sequence[int], names: Mapping[int, str],
                       labels: Optional[Sequence[str]] = None) -> List[Box]:
    """
    Return the boxes given as corners (x1, y1, x2, y2), keeping only `labels` if given.

    RF-DETR answers this way (its supervision Detections: xyxy, confidence, class_id), with
    COCO's own class numbers, which `names` maps to labels.
    """
    keep = {normal_label(s) for s in labels} if labels else None
    out = []
    for (x1, y1, x2, y2), score, cls in zip(xyxy, scores, class_ids):
        label = names.get(int(cls), str(int(cls)))
        if keep is None or normal_label(label) in keep:
            out.append(Box(label, float(score), (x1 + x2) / 2.0, (y1 + y2) / 2.0,
                           float(x2 - x1), float(y2 - y1)))
    return out


def corners(box: Box) -> Dict[str, int]:
    """Return a box's corners in whole pixels, for drawing it."""
    return {'x1': round(box.cx - box.width / 2), 'y1': round(box.cy - box.height / 2),
            'x2': round(box.cx + box.width / 2), 'y2': round(box.cy + box.height / 2)}


DEVICES = 'auto, cuda, cuda:N, mps or cpu'
CUDA_N = re.compile(r'cuda:(\d+)')


def pick_device(requested: str, model: str, cuda_devices: int, mps: bool) -> Optional[str]:
    """
    Choose where a PyTorch model (YOLO's .pt, RF-DETR's .pth) runs, from this computer.

    'auto' (the default) picks the GPU: CUDA (an NVIDIA card, or the Orin) first, then
    Metal on a Mac ('mps'), else the CPU. The other names are PyTorch's own, passed on as
    they are: 'cuda' (the first NVIDIA card), 'cuda:N', 'mps' and 'cpu'.

    A .engine (TensorRT) or .mlpackage (Core ML) model picks its own hardware, so it gets
    None, whatever was asked. Raises ValueError, saying what to use instead, for a name
    PyTorch doesn't know or a device this computer doesn't have.
    """
    name = requested.strip()
    if name not in ('auto', 'cuda', 'mps', 'cpu') and not CUDA_N.fullmatch(name):
        raise ValueError(f"device '{requested}' isn't a device name: use {DEVICES}")
    if not model.endswith(('.pt', '.pth')):
        return None
    if name == 'auto':
        return 'cuda:0' if cuda_devices else 'mps' if mps else 'cpu'
    if name == 'mps' and not mps:
        raise ValueError("device 'mps' is a Mac's GPU, and this computer isn't a Mac with "
                         f"Metal: use auto, or {'cuda' if cuda_devices else 'cpu'}")
    if name.startswith('cuda'):
        if not cuda_devices:
            instead = "mps (this Mac's GPU)" if mps else 'cpu'
            raise ValueError(f"device '{name}' needs an NVIDIA GPU, and this computer has "
                             f'none: use auto or {instead}')
        number = CUDA_N.fullmatch(name)
        if number and int(number.group(1)) >= cuda_devices:
            raise ValueError(f"device '{name}': this computer has {cuda_devices} NVIDIA "
                             f'GPU(s), cuda:0 to cuda:{cuda_devices - 1}')
    return name
