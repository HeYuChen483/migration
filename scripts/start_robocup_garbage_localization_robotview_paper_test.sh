#!/usr/bin/env bash
set -euo pipefail

# Start only RGB-D localization for the current enhanced paper_ball path.
# Safe: no navigation goal and no cmd_vel. Default output is map-frame for
# moving-robot far-target localization.

WORKSPACE="${WORKSPACE:-/home/hyc/catkin_wa}"
SETUP="$WORKSPACE/devel/setup.bash"
if [[ ! -f "$SETUP" ]]; then
  echo "ERROR: catkin setup not found: $SETUP" >&2
  echo "Build first: cd $WORKSPACE && catkin_make --pkg garbage_localization" >&2
  exit 1
fi

source "$SETUP"

DETECTION_TOPIC="${DETECTION_TOPIC:-/yolo_world/enhanced_detections}"
POINT_CLOUD_TOPIC="${POINT_CLOUD_TOPIC:-/kinect2/hd/points}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/kinect2/hd/camera_info}"
OUTPUT_FRAME="${OUTPUT_FRAME:-map}"
ENABLE_STABILIZER="${ENABLE_STABILIZER:-true}"

exec roslaunch garbage_localization garbage_localization_robotview_paper.launch \
  detection_topic:="$DETECTION_TOPIC" \
  point_cloud_topic:="$POINT_CLOUD_TOPIC" \
  camera_info_topic:="$CAMERA_INFO_TOPIC" \
  output_frame:="$OUTPUT_FRAME" \
  enable_stabilizer:="$ENABLE_STABILIZER" \
  "$@"
