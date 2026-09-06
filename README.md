# RoboCup Paper Ball G7 Vision To Localization

This release contains the tested 2026-09-06 generation 7 paper_ball route from camera detection to map-frame object localization.

Only one vision model is included:

- `models/paper_ball_robotview_continue7_20260906_150818_best.onnx`

Generation 8 A/B models, training datasets, screenshots, and raw labels are intentionally not included.

## Contents

- `catkin_src/vision_msgs`: custom detection messages.
- `catkin_src/yolo_world_detector`: YOLO-World cascade, ByteTrack, enhanced merger, overlay.
- `catkin_src/garbage_localization`: RGB-D 2D-to-3D localization and map-frame paper_ball stabilizer.
- `catkin_src/nav_pkg`: localization-only `map_server + AMCL` launch, with no `move_base` start.
- `scripts/start_robocup_garbage_realtime_robotview_paper_recall_test.sh`: confirmed G7 paper_ball route, with paper/box stable supplement disabled.
- `scripts/start_robocup_garbage_localization_robotview_paper_test.sh`: map-frame object localization.
- `scripts/start_robocup_map_localization_only.sh`: safe map/AMCL only startup.
- `scripts/check_*.sh`: passive status and precision checks.

## Assumptions

- ROS Noetic and `/home/hyc/catkin_wa` are available, or set `WORKSPACE=/path/to/catkin_ws`.
- Kinect2 topics are available: `/kinect2/hd/image_color`, `/kinect2/hd/points`, `/kinect2/hd/camera_info`.
- The existing primary garbage model is available at `/home/hyc/robocup_vision/yolov8s-worldv2-garbage.onnx`, or set `PRIMARY_MODEL_PATH`.

## Install

From the cloned release directory:

```bash
cd /home/hyc/robocup_paper_ball_g7_vision_localization_release
./install_release.sh
```

If the workspace path is different:

```bash
WORKSPACE=/path/to/catkin_ws ./install_release.sh
```

## Run Order

Terminal 1, start G7 vision:

```bash
cd /home/hyc/robocup_paper_ball_g7_vision_localization_release
./scripts/start_robocup_garbage_realtime_robotview_paper_recall_test.sh
```

Terminal 2, start safe map/AMCL localization if `map` is missing:

```bash
cd /home/hyc/robocup_paper_ball_g7_vision_localization_release
./scripts/start_robocup_map_localization_only.sh
```

Terminal 3, start RGB-D object localization:

```bash
cd /home/hyc/robocup_paper_ball_g7_vision_localization_release
./scripts/start_robocup_garbage_localization_robotview_paper_test.sh
```

Terminal 4, check the chain:

```bash
cd /home/hyc/robocup_paper_ball_g7_vision_localization_release
./scripts/check_paper_ball_three_stage_topics.sh
./scripts/check_garbage_localization_status.sh
SECONDS_TO_SAMPLE=10 ./scripts/check_far_paper_localization_precision.sh
```

## Confirmed Route

- Detection topic: `/yolo_world/enhanced_detections`.
- Raw map-frame localization: `/garbage_localization/objects`.
- Stable paper_ball output for future navigation: `/garbage_localization/stable_objects`.
- G7 secondary model threshold: `PAPER_BOX_SECONDARY_CONFIDENCE_THRESHOLD=0.12`.
- Paper/box stable supplement: disabled by default with `ENABLE_PAPER_BOX_SUPPLEMENT=false`.
- RGB-D vote: disabled by default with `ENABLE_PAPER_BALL_RGBD_VOTE=false`.
- Localization frame: `map`, not `base_link`, so moving-robot observations can be fused.

For far-distance navigation, use `/garbage_localization/stable_objects`, not the single-frame raw output. If `xy_jitter_m.p95 <= 0.25-0.35m`, the target is usually usable for navigation. If it is above `0.5m`, debug TF, AMCL initial pose, or Kinect depth sync before sending navigation goals.

## Safety

The localization-only launch and check scripts do not publish `/cmd_vel` and do not send navigation goals. Navigation integration should consume `/garbage_localization/stable_objects` later.
