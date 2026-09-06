#!/usr/bin/env bash
set -euo pipefail

# Read-only precision sampler for far paper_ball map localization.

WORKSPACE="${WORKSPACE:-/home/hyc/catkin_wa}"
SETUP="$WORKSPACE/devel/setup.bash"
if [[ -f "$SETUP" ]]; then
  source "$SETUP"
fi

SECONDS_TO_SAMPLE="${SECONDS_TO_SAMPLE:-8}"
TOPIC="${TOPIC:-/garbage_localization/stable_objects}"

exec rosrun garbage_localization sample_localization_stability.py \
  --topic "$TOPIC" \
  --class-name paper_ball \
  --frame map \
  --seconds "$SECONDS_TO_SAMPLE"
