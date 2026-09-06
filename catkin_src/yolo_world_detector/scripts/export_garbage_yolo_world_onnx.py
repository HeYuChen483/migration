#!/usr/bin/env python3
"""Export the enhanced YOLO-World vocabulary ONNX model.

This keeps the original ten project classes and appends paper_ball and box so
recognition is strengthened without dropping the previous baseline vocabulary.
The export runs from a temporary weight copy so the baseline
``/home/hyc/robocup_vision/yolov8s-worldv2.onnx`` is never overwritten or moved.
"""

import shutil
import tempfile
from pathlib import Path

from ultralytics import YOLOWorld


CLASSES = [
    "person",
    "cup",
    "mug",
    "bottle",
    "bowl",
    "plate",
    "fork",
    "knife",
    "spoon",
    "chopsticks",
    "paper_ball",
    "box",
]

WEIGHTS = Path("/home/hyc/robocup_vision/yolov8s-worldv2.pt")
OUTPUT = Path("/home/hyc/robocup_vision/yolov8s-worldv2-garbage.onnx")


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="yolo_world_enhanced_export_") as temp_dir:
        temp_weights = Path(temp_dir) / "yolov8s-worldv2-enhanced.pt"
        shutil.copy2(WEIGHTS, temp_weights)
        model = YOLOWorld(str(temp_weights))
        model.set_classes(CLASSES)
        exported = Path(
            model.export(
                format="onnx",
                imgsz=640,
                opset=17,
                simplify=False,
                dynamic=False,
            )
        )
        shutil.copy2(exported, OUTPUT)
    print(f"exported={OUTPUT}")
    print("classes=" + ",".join(CLASSES))


if __name__ == "__main__":
    main()
