#!/usr/bin/env bash
set -euo pipefail

# Passive localization health check. It never publishes /cmd_vel or navigation goals.

WORKSPACE="${WORKSPACE:-/home/hyc/catkin_wa}"
SETUP="$WORKSPACE/devel/setup.bash"
if [[ -f "$SETUP" ]]; then
  source "$SETUP"
fi

echo "== ROS master =="
if ! timeout 3s rosnode list >/tmp/garbage_localization_rosnodes.txt 2>/tmp/garbage_localization_rosnodes.err; then
  echo "ROS master not reachable. Start the robot/vision stack first."
  cat /tmp/garbage_localization_rosnodes.err 2>/dev/null || true
  exit 1
fi
cat /tmp/garbage_localization_rosnodes.txt

echo
echo "== Required topics =="
topics=$(rostopic list)
for topic in \
  /map \
  /amcl_pose \
  /scan \
  /odom \
  /yolo_world/enhanced_detections \
  /garbage_localization/objects \
  /garbage_localization/stable_objects \
  /kinect2/hd/camera_info \
  /kinect2/hd/points \
  /kinect2/sd/points \
  /tf \
  /tf_static; do
  if grep -qx "$topic" <<<"$topics"; then
    echo "OK   $topic"
  else
    echo "MISS $topic"
  fi
done

echo
echo "== TF checks =="
echo "map -> base_footprint"
timeout 4s rosrun tf tf_echo map base_footprint || true
echo
echo "map -> base_link"
timeout 4s rosrun tf tf_echo map base_link || true
echo
echo "base_link -> kinect2_rgb_optical_frame"
timeout 4s rosrun tf tf_echo base_link kinect2_rgb_optical_frame || true

echo
echo "== If map frame is missing =="
echo "Run: cd /home/hyc && ./start_robocup_map_localization_only.sh"

echo
echo "== Latest localization output =="
timeout 4s rostopic echo -n 1 /garbage_localization/objects || true

echo
echo "== Latest stable map-frame paper target =="
timeout 4s rostopic echo -n 1 /garbage_localization/stable_objects || true

echo
echo "== Latest localization diagnostic =="
timeout 4s rostopic echo -n 1 /diagnostics | sed -n '/garbage_localization\/status/,+35p' || true
