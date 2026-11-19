#!/usr/bin/env python3
"""
Export a detector model for a faster runtime: Core ML on a Mac, ONNX for the Orin's TensorRT.

usage (Core ML in the ml environment, on a Mac; no pixi task):
    pixi run -e ml python3 tools/export_model.py --format coreml
        models/rf-detr-nano.pth by default: makes models/rf-detr-nano.mlpackage
    pixi run -e ml python3 tools/export_model.py --format coreml models/yolo11n.pt
        makes models/yolo11n.mlpackage
    pixi run -e ml python3 tools/export_model.py --format onnx [models/rf-detr-nano.pth]

  .pth  RF-DETR, through Roboflow's rfdetr (Apache-2.0): its own model.export()
  .pt   YOLO, through Ultralytics (AGPL-3.0)

The result goes next to the model, under its name: models/rf-detr-nano.mlpackage. The
detector loads it with model:=models/rf-detr-nano.mlpackage; laptop.yaml says how.

  coreml  a Mac only (coremltools). RF-DETR Nano: about 4 s, 108 MB, full precision (half
          precision was no faster on an M4 Max)
  onnx    in the ml environment (onnx, onnxscript; 17 Nov): RF-DETR Nano in 5 s, 108 MB,
          opset 17, input (1, 3, 384, 384). ONNX runs anywhere: the Rust detector
          (aav_detector_rs, docs/onnx.md) reads it, and it is the step before the Orin's
          TensorRT engine, which tools/build_engine.py makes on the Orin itself (from the
          .pth, or from this .onnx copied over)
"""

import argparse
import importlib.util
from pathlib import Path
import shutil
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros' / 'perception' / 'aav_detector'))
FORMATS = {'coreml': '.mlpackage', 'onnx': '.onnx'}
NEEDS = {'coreml': 'coremltools', 'onnx': 'onnx'}


def export_rfdetr(pth: Path, fmt: str, out: Path) -> Path:
    """Export an RF-DETR .pth to `fmt`, as the file `out`; return `out`."""
    import rfdetr

    from aav_detector.boxes import rfdetr_size
    model = getattr(rfdetr, rfdetr_size(str(pth)))(pretrain_weights=str(pth), device='cpu')
    # rfdetr names its file itself (rfdetr-nano_fp32.mlpackage): make it in a folder of
    # its own, then move it to our name.
    with tempfile.TemporaryDirectory(dir=out.parent) as folder:
        made = Path(model.export(output_dir=folder, format=fmt, verbose=False))
        if made.is_dir() and made.suffix != out.suffix:
            made = next(made.glob('*' + out.suffix))
        if out.is_dir():
            shutil.rmtree(out)
        elif out.exists():
            out.unlink()
        shutil.move(str(made), str(out))
    return out


def export_yolo(pt: Path, fmt: str, out: Path) -> Path:
    """Export a YOLO .pt with Ultralytics, at 640 px as the detector runs; return the file."""
    from ultralytics import YOLO
    return Path(YOLO(str(pt)).export(format=fmt, imgsz=640))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    p.add_argument('model', nargs='?', default='models/rf-detr-nano.pth',
                   help='a .pth (RF-DETR) or .pt (YOLO); default models/rf-detr-nano.pth')
    p.add_argument('--format', choices=sorted(FORMATS), default='coreml')
    p.add_argument('--fp16', action='store_true',
                   help='with --format onnx: also <name>_fp16.onnx, half precision inside, '
                        'float32 in and out (from the .onnx, made first if missing)')
    args = p.parse_args(argv)
    model = Path(args.model)
    model = model if model.is_absolute() else ROOT / model
    if importlib.util.find_spec(NEEDS[args.format]) is None:
        how = ('on a Mac, in the ml environment: '
               'pixi run -e ml python3 tools/export_model.py --format coreml'
               if args.format == 'coreml' else
               'the ml environment: pixi run -e ml python3 tools/export_model.py '
               '--format onnx; or let tools/build_engine.py do it on the Orin')
        print(f'--format {args.format} needs {NEEDS[args.format]}: {how}', file=sys.stderr)
        return 2
    if not model.exists():
        print(f'{model} is missing: pixi run get models', file=sys.stderr)
        return 2
    out = model.with_suffix(FORMATS[args.format])
    start = time.monotonic()
    if args.fp16 and args.format == 'onnx' and out.exists():
        half = half_precision(out)
        print(f'{half}: made in {time.monotonic() - start:.0f} s')
        return 0
    if model.suffix == '.pth':
        made = export_rfdetr(model, args.format, out)
    elif model.suffix == '.pt':
        made = export_yolo(model, args.format, out)
    else:
        print(f'{model}: export a .pth (RF-DETR) or a .pt (YOLO)', file=sys.stderr)
        return 2
    print(f'{made}: made in {time.monotonic() - start:.0f} s')
    if args.fp16 and args.format == 'onnx':
        print(f'{half_precision(Path(made))}: made')
    return 0


def half_precision(onnx_file: Path) -> Path:
    """
    Write <name>_fp16.onnx: the weights and the work in half precision, in and out float32.

    ONNX Runtime's own converter (onnxruntime.transformers.float16). For the GPUs that
    work faster in half: TensorRT on the Orin, and Core ML's Neural Engine on a Mac.
    `pixi run measure onnx` checks it against the float32 file.
    """
    import onnx
    from onnxruntime.transformers.float16 import convert_float_to_float16
    out = onnx_file.with_name(onnx_file.stem + '_fp16.onnx')
    model = convert_float_to_float16(onnx.load(str(onnx_file)), keep_io_types=True)
    onnx.save(model, str(out))
    return out


if __name__ == '__main__':
    sys.exit(main())
