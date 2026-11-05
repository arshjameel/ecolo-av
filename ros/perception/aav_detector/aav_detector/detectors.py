"""
The detectors as plain Python objects: load a model once, then detect(picture) -> boxes.

No ROS here, so any program can use them: the detector node (detector.py), a camera
stream's own program, a script.

    from aav_detector.detectors import load_detector
    model = load_detector()                     # RF-DETR Nano: models/rf-detr-nano.pth
    boxes = model.detect(bgr)                   # an OpenCV picture: H x W x 3, BGR, uint8
    for box in boxes:                           # aav_detector.boxes.Box, in that picture's pixels
        print(box.label, box.score, box.cx, box.cy, box.width, box.height)

load_detector(detector, model, device) takes the node's own parameters:

  detector  model file             runs on                                  class
  rfdetr    .pth (the default)     PyTorch: CUDA, a Mac's GPU (mps), or CPU  RfdetrTorch
  rfdetr    rf-detr-*.mlpackage    Core ML, a Mac's GPU and Neural Engine   RfdetrCoreml
  rfdetr    rf-detr-*.engine       TensorRT, the Orin's GPU                 RfdetrTensorrt
  yolo      .pt, .mlpackage, .engine   Ultralytics picks                    Yolo

Every class has detect(bgr, confidence=0.5, labels=None) -> List[Box]: labels keeps only
those names ('stop sign', or ['stop sign', 'person']); the labels are COCO's ('stop sign',
'person', 'car'...), the same for both detectors. Each also says what it is: .detector,
.model (the file), .device (where it runs: 'mps', 'cuda:0', 'coreml', 'tensorrt'...).

Each needs its own library: PyTorch and rfdetr or Ultralytics (the ml environment);
coremltools for Core ML; TensorRT (JetPack's) for an .engine. They're imported only when
a model is loaded.
"""

from __future__ import annotations

import os
from typing import Callable, List, Optional, Sequence, Union

from aav_detector.boxes import (Box, boxes_from_corners, boxes_from_result, check_detector,
                                normal_label, parse_labels, pick_device, rfdetr_size)
from aav_detector.detr import decode, input_size, preprocess, split_outputs
import numpy as np

Labels = Union[None, str, Sequence[str]]


def keep_labels(labels: Labels) -> Optional[List[str]]:
    """Return the labels to keep: None (all), from a list or a 'stop sign, person' string."""
    if labels is None:
        return None
    if isinstance(labels, str):
        return parse_labels(labels)
    return [normal_label(s) for s in labels] or None


def rgb_of(bgr: np.ndarray) -> np.ndarray:
    """Return an OpenCV picture (BGR) as RGB, which the models read."""
    return np.ascontiguousarray(bgr[:, :, ::-1])


def coco_names():
    """Return rfdetr's COCO class names, by COCO's own numbers (1 = person, 13 = stop sign)."""
    from rfdetr.assets.coco_classes import COCO_CLASSES
    return dict(COCO_CLASSES)


class Yolo:
    """YOLO, through Ultralytics: any of its file types (.pt, .mlpackage, .engine)."""

    detector = 'yolo'

    def __init__(self, model: str, device: Optional[str], image_size: int = 640):
        from ultralytics import YOLO
        self.model, self.device = model, device or 'its own choice of hardware'
        self.yolo = YOLO(model, task='detect')
        self.kwargs = {'imgsz': image_size, **({'device': device} if device else {})}

    def detect(self, bgr, confidence: float = 0.5, labels: Labels = None) -> List[Box]:
        result = self.yolo.predict(bgr, conf=confidence, verbose=False, **self.kwargs)[0]
        return boxes_from_result(result, keep_labels(labels))


class RfdetrTorch:
    """RF-DETR, through Roboflow's rfdetr library: a .pth file, on PyTorch."""

    detector = 'rfdetr'

    def __init__(self, model: str, device: str, log: Callable[[str], None] = print):
        import rfdetr
        self.model, self.device = model, device
        self.names = coco_names()
        self.rfdetr = getattr(rfdetr, rfdetr_size(model))(pretrain_weights=model, device=device)
        # Traced (TorchScript) is the fast path; a device it can't trace runs it plain.
        # rfdetr 1.11 calls it inference(), older versions optimize_for_inference().
        optimise = getattr(self.rfdetr, 'inference', None) or self.rfdetr.optimize_for_inference
        try:
            optimise()
        except Exception as e:  # noqa: B902 - any tracing failure: run untraced instead
            log(f'RF-DETR runs untraced here ({e.__class__.__name__})')
            optimise(compile=False)

    def detect(self, bgr, confidence: float = 0.5, labels: Labels = None) -> List[Box]:
        found = self.rfdetr.predict(rgb_of(bgr), threshold=confidence)
        return boxes_from_corners(found.xyxy.tolist(), found.confidence.tolist(),
                                  found.class_id.tolist(), self.names, keep_labels(labels))


class RfdetrExported:
    """An exported RF-DETR: the subclass runs the model; detr.py does the rest."""

    detector = 'rfdetr'
    height = width = 0

    def run(self, picture: np.ndarray) -> dict:
        """Return the model's raw outputs, by name, for one input (1, 3, H, W)."""
        raise NotImplementedError

    def detect(self, bgr, confidence: float = 0.5, labels: Labels = None) -> List[Box]:
        rgb = rgb_of(bgr)
        boxes, logits = split_outputs(self.run(preprocess(rgb, self.height, self.width)))
        corners, scores, classes = decode(boxes, logits, rgb.shape[1], rgb.shape[0],
                                          threshold=confidence)
        return boxes_from_corners(corners.tolist(), scores.tolist(), classes.tolist(),
                                  self.names, keep_labels(labels))


class RfdetrCoreml(RfdetrExported):
    """RF-DETR exported to Core ML (tools/export_model.py --format coreml): a Mac's GPU and ANE."""

    def __init__(self, model: str):
        import coremltools as ct
        self.model, self.device = model, 'coreml'
        self.names = coco_names()
        self.mlmodel = ct.models.MLModel(model, compute_units=ct.ComputeUnit.ALL)
        first = self.mlmodel.get_spec().description.input[0]
        self.input = first.name
        self.height, self.width = input_size(first.type.multiArrayType.shape)

    def run(self, picture):
        return self.mlmodel.predict({self.input: picture})


class RfdetrTensorrt(RfdetrExported):
    """
    RF-DETR as a TensorRT engine (tools/build_engine.py, on the Orin): the Orin's GPU.

    PyTorch holds the GPU's buffers, so no pycuda is needed. Untested: it needs the Orin.
    """

    def __init__(self, model: str):
        import tensorrt as trt
        import torch
        self.torch = torch
        self.model, self.device = model, 'tensorrt'
        self.names = coco_names()
        # The logger and runtime must outlive the engine they made: keep all three.
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        with open(model, 'rb') as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"{model}: TensorRT can't load it. An engine runs only on "
                               'the GPU and TensorRT version that built it: build it here, '
                               'with tools/build_engine.py')
        self.context = self.engine.create_execution_context()
        self.stream = torch.cuda.Stream()
        types = {trt.float32: torch.float32, trt.float16: torch.float16,
                 trt.int32: torch.int32, trt.int64: torch.int64}
        self.buffers, self.input = {}, None
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            if min(shape) < 0:
                raise RuntimeError(f'{model}: {name} has no fixed size {shape}: build the '
                                   'engine for one picture at a time (tools/build_engine.py)')
            buffer = torch.empty(shape, dtype=types[self.engine.get_tensor_dtype(name)],
                                 device='cuda')
            self.context.set_tensor_address(name, buffer.data_ptr())
            self.buffers[name] = buffer
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input = name
                self.height, self.width = input_size(shape)

    def run(self, picture):
        torch = self.torch
        with torch.cuda.stream(self.stream):
            self.buffers[self.input].copy_(torch.from_numpy(picture), non_blocking=False)
            if not self.context.execute_async_v3(self.stream.cuda_stream):
                raise RuntimeError('TensorRT could not run the engine')
        self.stream.synchronize()
        return {name: buffer.float().cpu().numpy() for name, buffer in self.buffers.items()
                if name != self.input}


def load_detector(detector: str = 'rfdetr', model: str = '', device: str = 'auto',
                  image_size: int = 640, root: Optional[str] = None,
                  log: Callable[[str], None] = print):
    """
    Load a detector once; then call its detect(bgr, confidence, labels) for each picture.

    detector  'rfdetr' (the default) or 'yolo'
    model     the model file; '' = the detector's own (models/rf-detr-nano.pth). A relative
              path is from `root`: the repository ($PIXI_PROJECT_ROOT, else this folder)
    device    PyTorch's: 'auto' (the GPU this computer has), 'cuda', 'cuda:N', 'mps' or
              'cpu'. A .mlpackage or .engine picks its own hardware
    image_size  YOLO's input size; RF-DETR's is fixed by the model (384 px for Nano)

    Raises ValueError, saying what to use instead, for a wrong detector, model or device;
    ImportError when this Python lacks the model's library; FileNotFoundError without the
    model file.
    """
    model = check_detector(detector, model)
    root = root or os.environ.get('PIXI_PROJECT_ROOT', os.getcwd())
    model = os.path.join(root, os.path.expanduser(model))
    if not os.path.exists(model):
        raise FileNotFoundError(f'{model} is missing: pixi run get models, or '
                                'docs/perception.md ("Models") for the exported ones')
    if model.endswith(('.pt', '.pth')):
        import torch
        where = pick_device(device, model, torch.cuda.device_count(),
                            torch.backends.mps.is_available())
    else:
        where = pick_device(device, model, 0, False)   # checks the name only: None
    if detector.strip().lower() == 'yolo':
        return Yolo(model, where, image_size)
    if model.endswith('.mlpackage'):
        return RfdetrCoreml(model)
    if model.endswith('.engine'):
        return RfdetrTensorrt(model)
    return RfdetrTorch(model, where, log)
