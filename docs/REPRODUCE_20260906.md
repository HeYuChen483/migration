# 2026-09-06 Real-Robot Reproduction Notes

这份说明只对应 2026-09-06 已确认的真实机器人 G7 `paper_ball` 视觉到定位链路。

不包含 2026-09-07 之后的 prompt A/B、console、Adapter、day-of training 尝试；也不包含仿真包和最终地图，因为真实场地地图还没有建好。

## Included Files

- 主检测模型：`models/yolov8s-worldv2-garbage.onnx`
- G7 `paper_ball` 专家模型：`models/paper_ball_robotview_continue7_20260906_150818_best.onnx`
- ONNX Runtime C++ 运行库：`third_party/onnxruntime`
- freenect2 SDK 快照：`third_party/freenect2`
- ROS 源码：`catkin_src/vision_msgs`、`catkin_src/yolo_world_detector`、`catkin_src/garbage_localization`、`catkin_src/nav_pkg`
- Kinect2 源码副本：`catkin_src/iai_kinect2`、`catkin_src/libfreenect2`

## Not Included

- 最终真实场地地图：尚未建图，不能上传旧 `map.yaml` / `map.pgm` 冒充最终地图
- `wpr_simulation`、`wpb_home` 等仿真 / 教程包
- 2026-09-07 之后的新尝试：prompt A/B、Trash Vision Console、Adapter/domain training、day-of dataset/training
- 训练数据、截图、原始 labels、G8/A-B 模型和旧模型变体

## Vision Architecture

视觉识别是双模型链路：

1. Primary garbage model
   - 文件：`models/yolov8s-worldv2-garbage.onnx`
   - class 文件：`catkin_src/yolo_world_detector/config/garbage_classes.txt`
   - 默认阈值：`PRIMARY_CONFIDENCE_THRESHOLD=0.18`
   - 作用：保留通用垃圾 / 常见物体检测主路。

2. Secondary G7 paper_ball expert
   - 文件：`models/paper_ball_robotview_continue7_20260906_150818_best.onnx`
   - class 文件：`catkin_src/yolo_world_detector/config/paper_box_classes.txt`
   - 开关：`ENABLE_PAPER_BOX_SECONDARY=true`
   - detector entry 阈值：`PAPER_BOX_SECONDARY_DETECTOR_CONFIDENCE_THRESHOLD=0.06`
   - 最终接收阈值：`PAPER_BOX_SECONDARY_CONFIDENCE_THRESHOLD=0.12`
   - crop：`PAPER_BOX_SECONDARY_FALLBACK_CENTER_CROP_SCALE=1.8`，`PAPER_BOX_SECONDARY_FALLBACK_CROP_MODE=center_bottom`
   - 作用：专门提高远距离 / 小 `paper_ball` 召回。

3. Disabled in the 2026-09-06 handoff profile
   - `ENABLE_PAPER_BOX_SUPPLEMENT=false`
   - `ENABLE_PAPER_BALL_RGBD_VOTE=false`
   - 不使用 boxpaper stable supplement，不使用 G8，也不使用 9 月 7 日之后的 prompt/adapter 方案。

最终视觉输出 topic：`/yolo_world/enhanced_detections`。

## Install

在学长机器上先准备 ROS Noetic 和 catkin workspace，例如：

```bash
mkdir -p /home/hyc/catkin_wa/src
cd /home/hyc/robocup_paper_ball_g7_vision_localization_release
WORKSPACE=/home/hyc/catkin_wa ./install_release.sh
```

如果 workspace 不是 `/home/hyc/catkin_wa`，把 `WORKSPACE` 换成实际路径。

如果机器上已经单独安装了 ONNX Runtime，可以覆盖：

```bash
ONNXRUNTIME_ROOT=/path/to/onnxruntime WORKSPACE=/path/to/catkin_ws ./install_release.sh
```

## Kinect2 Requirement

视觉默认使用 Kinect2 HD 彩色图：

- `/kinect2/hd/image_color`
- `/kinect2/hd/points`
- `/kinect2/hd/camera_info`

仓库里放了 `iai_kinect2` / `libfreenect2` 源码副本，但不同机器的 USB 权限、udev、GPU/OpenCL、libusb 环境可能不同。若 `kinect2_bridge` 启动失败，先按 Kinect2 驱动常规方式在学长机器上把 Kinect2 跑通，再回到本流程。

如果需要重编 `kinect2_bridge`，优先让 CMake 找到仓库内的 freenect2 SDK：

```bash
cd /home/hyc/catkin_wa
catkin_make --pkg kinect2_registration kinect2_bridge \
  -Dfreenect2_DIR=/home/hyc/robocup_paper_ball_g7_vision_localization_release/third_party/freenect2/lib/cmake/freenect2
```

## Run Order

Terminal 1，启动双模型视觉识别：

```bash
cd /home/hyc/robocup_paper_ball_g7_vision_localization_release
./scripts/start_robocup_garbage_realtime_robotview_paper_recall_test.sh
```

确认视觉 topic：

```bash
rostopic echo /yolo_world/enhanced_detections
rostopic hz /yolo_world/detections
```

Terminal 2，建图完成后再启动 map/AMCL。当前仓库不带最终地图，所以必须手动给 `MAP_FILE`：

```bash
cd /home/hyc/robocup_paper_ball_g7_vision_localization_release
MAP_FILE=/path/to/real_map.yaml ./scripts/start_robocup_map_localization_only.sh
```

Terminal 3，启动 RGB-D 到 map-frame 的物体定位：

```bash
cd /home/hyc/robocup_paper_ball_g7_vision_localization_release
./scripts/start_robocup_garbage_localization_robotview_paper_test.sh
```

Terminal 4，检查链路：

```bash
cd /home/hyc/robocup_paper_ball_g7_vision_localization_release
./scripts/check_paper_ball_three_stage_topics.sh
./scripts/check_garbage_localization_status.sh
SECONDS_TO_SAMPLE=10 ./scripts/check_far_paper_localization_precision.sh
```

## Expected Topics

- raw primary detections：`/yolo_world/detections`
- raw G7 secondary detections：`/yolo_world/paper_box_secondary_detections`
- tracked G7 secondary detections：`/yolo_world/paper_box_secondary_tracked_detections`
- merged final detections：`/yolo_world/enhanced_detections`
- raw localized objects：`/garbage_localization/objects`
- stable localized objects：`/garbage_localization/stable_objects`

## Localization Gate

后续 navigation 不要直接吃单帧 `/garbage_localization/objects`，应该吃稳定后的 `/garbage_localization/stable_objects`。

判断规则：

- `xy_jitter_m.p95 <= 0.25-0.35m`：通常可以给 navigation 继续调
- `0.35m < xy_jitter_m.p95 <= 0.50m`：边界，需要复查 TF / AMCL / 深度同步
- `xy_jitter_m.p95 > 0.50m`：不要发 navigation goal

如果定位没有 `map` frame，说明还没启动建图后的 map/AMCL，或者 AMCL 初始位姿 / TF 没对齐。
