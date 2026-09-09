#!/usr/bin/env bash
set -euo pipefail

# Install ROS source overlay for the G7 paper_ball vision -> localization route.
# This script copies source packages only; it does not publish cmd_vel or start ROS nodes.

RELEASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-/home/hyc/catkin_wa}"
SRC_DIR="$WORKSPACE/src"

if [[ ! -d "$SRC_DIR" ]]; then
  echo "ERROR: catkin workspace src directory not found: $SRC_DIR" >&2
  echo "Set WORKSPACE=/path/to/catkin_ws and run again." >&2
  exit 1
fi

CORE_PACKAGES=(vision_msgs yolo_world_detector garbage_localization nav_pkg)
SUPPORT_DIRS=(iai_kinect2 libfreenect2)

for pkg in "${CORE_PACKAGES[@]}"; do
  echo "Installing $pkg -> $SRC_DIR/$pkg"
  mkdir -p "$SRC_DIR/$pkg"
  rsync -a --delete \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    "$RELEASE_ROOT/catkin_src/$pkg/" "$SRC_DIR/$pkg/"
done

for pkg in "${SUPPORT_DIRS[@]}"; do
  if [[ -d "$RELEASE_ROOT/catkin_src/$pkg" ]]; then
    echo "Installing support source $pkg -> $SRC_DIR/$pkg"
    mkdir -p "$SRC_DIR/$pkg"
    rsync -a --delete \
      --exclude='.git/' \
      --exclude='build/' \
      --exclude='devel/' \
      --exclude='install/' \
      --exclude='logs/' \
      --exclude='__pycache__/' \
      --exclude='*.pyc' \
      "$RELEASE_ROOT/catkin_src/$pkg/" "$SRC_DIR/$pkg/"
  fi
done

ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-$RELEASE_ROOT/third_party/onnxruntime}"
FREENECT2_ROOT="${FREENECT2_ROOT:-$RELEASE_ROOT/third_party/freenect2}"
export freenect2_DIR="${freenect2_DIR:-$FREENECT2_ROOT/lib/cmake/freenect2}"

echo "Building ROS packages..."
cd "$WORKSPACE"
catkin_make --force-cmake \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT" \
  -DCATKIN_WHITELIST_PACKAGES='vision_msgs;yolo_world_detector;garbage_localization;nav_pkg' \
  --pkg vision_msgs yolo_world_detector garbage_localization nav_pkg

echo
echo "Install complete. Source the workspace before running:"
echo "source $WORKSPACE/devel/setup.bash"
echo "ONNX Runtime root used for build: $ONNXRUNTIME_ROOT"
echo "Kinect2 freenect2 SDK path available at: $FREENECT2_ROOT"
echo "If rebuilding kinect2_bridge, export freenect2_DIR=$freenect2_DIR"
