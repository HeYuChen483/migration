# yolo_world_detector

ROS1 C++ detector package for the project's YOLO-World ONNX runtimes.

The basic launch keeps the original 10-class vocabulary. The current RoboCup
garbage live-camera entry uses the enhanced cascade launch documented in
`README_enhanced_cascade.md`, so `person` and common objects stay available
while the stable paper/box model and far-distance paper_ball specialist add
trained signals.

Basic model contract:

- input `images`, float32 `[1,3,640,640]`
- output `output0`, float32 `[1,14,8400]` at runtime; exported ONNX metadata may show dynamic batch/channel dims as `[-1,-1,8400]`
- ten classes in `config/classes.txt`, in model order
- raw center-width-height boxes and class scores; this node performs NMS

Build:

```bash
catkin_make -C <catkin_ws> --pkg yolo_world_detector \
  -DONNXRUNTIME_ROOT=/path/to/onnxruntime
source <catkin_ws>/devel/setup.bash
```

Run the basic detector:

```bash
roslaunch yolo_world_detector yolo_world_detector.launch
```

Detections publish on `/yolo_world/detections`; annotated images publish on
`/yolo_world/annotated_image`. The existing `yolo_detector` package and
`/yolo/detections` remain independent.

Run the current RoboCup live garbage/person visual pipeline from a normal local
terminal. The default image source follows the original `wpb_home` robot stack:
`/kinect2/hd/image_color` from `kinect2_bridge`.

```bash
cd /path/to/robocup2026_migration_senior_20260828/04_terminal_scripts
WORKSPACE=<catkin_ws> \
  ./start_robocup_garbage_realtime_dual_paper_box_far_selfpaper_boxstable_test.sh
```

USB fallback remains available with `CAMERA_SOURCE=usb` and an explicit device.

Open only the current combined visualization window:

```bash
cd /path/to/robocup2026_migration_senior_20260828/04_terminal_scripts
./view_robocup_garbage_detection.sh
```
