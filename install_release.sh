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

for pkg in vision_msgs yolo_world_detector garbage_localization nav_pkg; do
  echo "Installing $pkg -> $SRC_DIR/$pkg"
  mkdir -p "$SRC_DIR/$pkg"
  rsync -a --delete \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    "$RELEASE_ROOT/catkin_src/$pkg/" "$SRC_DIR/$pkg/"
done

echo "Building ROS packages..."
cd "$WORKSPACE"
catkin_make --force-cmake \
  -DCATKIN_WHITELIST_PACKAGES='vision_msgs;yolo_world_detector;garbage_localization;nav_pkg' \
  --pkg vision_msgs yolo_world_detector garbage_localization nav_pkg

echo
echo "Install complete. Source the workspace before running:"
echo "source $WORKSPACE/devel/setup.bash"
