#!/usr/bin/env bash
set -euo pipefail

# Start only map + AMCL for map-frame localization.
# Safe: this does not start move_base and never publishes /cmd_vel.

WORKSPACE="${WORKSPACE:-/home/hyc/catkin_wa}"
SETUP="$WORKSPACE/devel/setup.bash"
if [[ ! -f "$SETUP" ]]; then
  echo "ERROR: catkin setup not found: $SETUP" >&2
  exit 1
fi

source "$SETUP"

MAP_FILE="${MAP_FILE:-}"
if [[ -z "$MAP_FILE" ]]; then
  echo "ERROR: MAP_FILE is not set." >&2
  echo "This handoff does not include a final real-robot map yet." >&2
  echo "After mapping, run: MAP_FILE=/path/to/map.yaml $0" >&2
  exit 1
fi
if [[ ! -f "$MAP_FILE" ]]; then
  echo "ERROR: map file not found: $MAP_FILE" >&2
  exit 1
fi
INITIAL_POSE_X="${INITIAL_POSE_X:-6.75}"
INITIAL_POSE_Y="${INITIAL_POSE_Y:-0.35}"
INITIAL_POSE_A="${INITIAL_POSE_A:-1.5708}"
OPEN_RVIZ="${OPEN_RVIZ:-true}"

exec roslaunch nav_pkg localization_only.launch \
  map_file:="$MAP_FILE" \
  initial_pose_x:="$INITIAL_POSE_X" \
  initial_pose_y:="$INITIAL_POSE_Y" \
  initial_pose_a:="$INITIAL_POSE_A" \
  open_rviz:="$OPEN_RVIZ" \
  "$@"
