# YOLO-World enhanced cascade detector

Purpose: strengthen the original YOLO-World recognition without dropping the
existing baseline vocabulary.

Primary detector:

- Launch node: `yolo_world_detector`
- Model: `02_models/primary_full/yolov8s-worldv2-garbage.onnx`
- Classes: original 10 classes plus `paper_ball`, `box`
- Output topic: `/yolo_world/detections`
- Annotated topic: `/yolo_world/annotated_image`
- Default detector / track threshold: `0.18`

Stable supplemental paper/box detector:

- Launch node: `paper_box_detector`
- Model: `02_models/paper_box_supplement/paper_box_balanced_best.onnx`
- Classes: `paper_ball`, `box`
- Output topic: `/yolo_world/paper_box_detections`
- Annotated topic: `/yolo_world/paper_box_annotated_image`
- Detector candidate threshold: `0.15`
- New-track / visible confidence threshold: `0.35`

Far-distance paper_ball specialist:

- Launch node: `paper_box_secondary_detector`
- Model: `02_models/paper_ball_specialist/paper_ball_farhard_best.onnx`
- Classes: `paper_ball`, `box`
- Output topic: `/yolo_world/paper_box_secondary_detections`
- In the final merge, only its `paper_ball` result is accepted; its `box` result is suppressed.
- This model was fine-tuned from the current paper-ball specialist with 80 real far-distance hard cases.

ByteTrack stabilization layer:

- Launch nodes: `primary_bytetrack`, `paper_box_bytetrack`,
  `paper_box_secondary_bytetrack`
- Primary input/output: `/yolo_world/detections` ->
  `/yolo_world/primary_tracked_detections`
- Supplemental input/output: `/yolo_world/paper_box_detections` ->
  `/yolo_world/paper_box_tracked_detections`
- Far-distance specialist input/output:
  `/yolo_world/paper_box_secondary_detections` ->
  `/yolo_world/paper_box_secondary_tracked_detections`
- Default primary tracked classes:
  `person,cup,mug,bottle,bowl,plate,fork,knife,spoon,chopsticks,paper_ball,box`
- Default behavior: high-confidence detections start tracks, low-confidence
  detections can continue existing tracks when the detector threshold is lowered,
  and recently lost tracks are held briefly to reduce single-frame flicker.
- Disable for A/B testing with `enable_bytetrack:=false`.

Merged enhanced output:

- Launch node: `enhanced_detection_merger`
- Output topic: `/yolo_world/enhanced_detections`
- Visualization node: `detection_overlay_node`
- Annotated output topic: `/yolo_world/enhanced_annotated_image`
- Merge rule: keep all tracked primary detections, then add or replace overlapping
  `paper_ball` / `box` detections from the tracked supplemental stream when
  ByteTrack is enabled; with `enable_bytetrack:=false`, merge the raw
  supplemental stream as before.
- Task-layer recommendation: subscribe to `/yolo_world/enhanced_detections` so
  downstream code does not need to merge the primary and supplemental topics.

Terminal-only run:

```bash
cd /path/to/robocup2026_migration_senior_20260828/04_terminal_scripts
WORKSPACE=/path/to/catkin_ws \
  ./start_robocup_garbage_realtime_dual_paper_box_far_selfpaper_boxstable_test.sh
```

The default real-robot source follows `wpb_home`: `/kinect2/hd/image_color` from
`kinect2_bridge`. For USB fallback, run with `CAMERA_SOURCE=usb` and pass an
explicit `/dev/v4l/by-id/<robot-camera>` path.

Open visualization only when the recognition stack is already running:

```bash
cd /home/hyc
./view_robocup_garbage_detection.sh
```

Raw launch form:

```bash
source /opt/ros/noetic/setup.bash
source /path/to/catkin_ws/devel/setup.bash
roslaunch yolo_world_detector yolo_world_enhanced_cascade_camera.launch
```

Verification status:

- Built targets: `enhanced_detection_merger_node`, `bytetrack_node`,
  `detection_overlay_node`
- Runtime nodes verified: `yolo_world_detector`, `paper_box_detector`,
  `paper_box_secondary_detector`, `primary_bytetrack`, `paper_box_bytetrack`,
  `paper_box_secondary_bytetrack`, `enhanced_detection_merger`,
  `detection_overlay_node`
- Runtime topics verified: `/yolo_world/detections`,
  `/yolo_world/primary_tracked_detections`,
  `/yolo_world/paper_box_detections`, `/yolo_world/paper_box_tracked_detections`,
  `/yolo_world/paper_box_secondary_detections`,
  `/yolo_world/paper_box_secondary_tracked_detections`,
  `/yolo_world/enhanced_detections`, `/yolo_world/enhanced_annotated_image`
- Sample merged backend:
  `yolo_world_onnxruntime+bytetrack+paper_box_bytetrack+far_paper_ball_specialist`
- Sample merged publish rate: about 2.92 Hz on the current CPU/USB-camera setup

Primary tracking tuning examples:

```bash
# Track only the pick-up/object classes and leave person untracked.
PRIMARY_BYTETRACK_CLASSES=cup,mug,bottle,bowl,plate,fork,knife,spoon,chopsticks,paper_ball,box \
  ./start_robocup_garbage_realtime.sh

# Let weaker primary detections continue existing tracks without creating new ones.
PRIMARY_DETECTOR_CONFIDENCE_THRESHOLD=0.12 \
PRIMARY_TRACK_CONFIDENCE_THRESHOLD=0.18 \
PRIMARY_BYTETRACK_LOW_CONFIDENCE_THRESHOLD=0.12 \
  ./start_robocup_garbage_realtime.sh
```

Notes:

- Do not overwrite `/home/hyc/robocup_vision/yolov8s-worldv2.onnx`.
- Do not replace the primary enhanced ONNX with a two-class ONNX.
- The direct 12-class fine-tuned run exists at
  `/home/hyc/robocup_vision/garbage_training/runs/paper_box_enhanced12_yoloworld_s`,
  but sample testing showed poor paper_ball recall, so it is not the recommended
  runtime model.
- The cascade launch keeps the original vocabulary available while using the
  stable two-class model for paper_ball/box and the far-distance two-class model
  only as a paper_ball signal.
- The weak paper/box model over-detected `paper_ball`; the cleaned model reduced
  false positives but missed many paper balls. The current balanced paper/box
  ONNX is the default compromise for live testing.
