#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MIGRATION_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
if [[ -f "$MIGRATION_ROOT/models/yolov8s-worldv2-garbage.onnx" ]]; then
  DEFAULT_PRIMARY_MODEL_PATH="$MIGRATION_ROOT/models/yolov8s-worldv2-garbage.onnx"
else
  DEFAULT_PRIMARY_MODEL_PATH="/home/hyc/robocup_vision/yolov8s-worldv2-garbage.onnx"
fi
if [[ -f "$MIGRATION_ROOT/models/paper_ball_robotview_continue7_20260906_150818_best.onnx" ]]; then
  DEFAULT_PAPER_BALL_SPECIALIST_MODEL_PATH="$MIGRATION_ROOT/models/paper_ball_robotview_continue7_20260906_150818_best.onnx"
else
  DEFAULT_PAPER_BALL_SPECIALIST_MODEL_PATH="/home/hyc/robocup_vision/garbage_training/runs/paper_box_robotview_threefusion_specialist_continue7_20260906_150818_yoloworld_s/weights/best.onnx"
fi
if [[ -d "$MIGRATION_ROOT/third_party/onnxruntime/lib" ]]; then
  DEFAULT_ONNXRUNTIME_LIB_DIR="$MIGRATION_ROOT/third_party/onnxruntime/lib"
else
  DEFAULT_ONNXRUNTIME_LIB_DIR="/home/hyc/robocup_vision/onnxruntime/lib"
fi

if [[ -d "$MIGRATION_ROOT/02_models" ]]; then
  DEFAULT_PRIMARY_MODEL_PATH="$MIGRATION_ROOT/02_models/primary_full/yolov8s-worldv2-garbage.onnx"
  DEFAULT_PAPER_BOX_MODEL_PATH="$MIGRATION_ROOT/02_models/paper_box_supplement/paper_box_balanced_best.onnx"
  DEFAULT_PAPER_BALL_SPECIALIST_MODEL_PATH="$MIGRATION_ROOT/02_models/paper_ball_specialist/paper_ball_farhard_best.onnx"
  DEFAULT_ONNXRUNTIME_LIB_DIR="$MIGRATION_ROOT/03_runtime/onnxruntime/lib"
else
  DEFAULT_PAPER_BOX_MODEL_PATH="/home/hyc/robocup_vision/garbage_training/runs/paper_box_balanced_user_paper_wallneg_finetune_from_userbest_20260829_yoloworld_s/weights/best.onnx"
fi

WORKSPACE=${WORKSPACE:-/home/hyc/catkin_wa}
CAMERA_SOURCE=${CAMERA_SOURCE:-}
if [[ -z "$CAMERA_SOURCE" ]]; then
  if [[ $# -gt 0 || -n "${VIDEO_DEVICE:-}" ]]; then
    CAMERA_SOURCE=usb
  else
    CAMERA_SOURCE=kinect2
  fi
fi

case "$CAMERA_SOURCE" in
  kinect|kinect2)
    CAMERA_SOURCE=kinect2
    CAMERA_IMAGE_TOPIC=${CAMERA_IMAGE_TOPIC:-/kinect2/hd/image_color}
    ;;
  usb|usb_cam)
    CAMERA_SOURCE=usb
    CAMERA_IMAGE_TOPIC=${CAMERA_IMAGE_TOPIC:-/usb_cam/image_raw}
    if [[ $# -gt 0 ]]; then
      VIDEO_DEVICE="$1"
    elif [[ -n "${VIDEO_DEVICE:-}" ]]; then
      VIDEO_DEVICE="$VIDEO_DEVICE"
    else
      echo "ERROR: CAMERA_SOURCE=usb requires a camera device."
      echo "Run: $SCRIPT_DIR/list_video_devices.sh"
      echo "Then pass the robot camera path, preferably /dev/v4l/by-id/<robot-camera>."
      exit 1
    fi
    ;;
  *)
    echo "ERROR: unknown CAMERA_SOURCE=$CAMERA_SOURCE. Use kinect2 or usb."
    exit 1
    ;;
esac
CAMERA_IMAGE_WIDTH=${CAMERA_IMAGE_WIDTH:-640}
CAMERA_IMAGE_HEIGHT=${CAMERA_IMAGE_HEIGHT:-480}
CAMERA_PIXEL_FORMAT=${CAMERA_PIXEL_FORMAT:-yuyv}
CAMERA_COLOR_FORMAT=${CAMERA_COLOR_FORMAT:-yuv422p}
if [[ -z "${CAMERA_FRAME_ID:-}" ]]; then
  if [[ "$CAMERA_SOURCE" == "kinect2" ]]; then
    CAMERA_FRAME_ID=kinect2_rgb_optical_frame
  else
    CAMERA_FRAME_ID=usb_cam
  fi
fi
CAMERA_IO_METHOD=${CAMERA_IO_METHOD:-mmap}
CAMERA_DISABLE_FALLBACK=${CAMERA_DISABLE_FALLBACK:-false}
CAMERA_DISABLE_DEVICE_FALLBACK=${CAMERA_DISABLE_DEVICE_FALLBACK:-true}
KINECT2_START_BRIDGE=${KINECT2_START_BRIDGE:-true}
ONNXRUNTIME_LIB_DIR=${ONNXRUNTIME_LIB_DIR:-$DEFAULT_ONNXRUNTIME_LIB_DIR}
YOLO_EXECUTION_PROVIDER=${YOLO_EXECUTION_PROVIDER:-cpu}
YOLO_CUDA_DEVICE_ID=${YOLO_CUDA_DEVICE_ID:-0}
MODE=${MODE:-enhanced}
VIEW_TOPIC=${VIEW_TOPIC:-/yolo_world/enhanced_annotated_image}
LOG_DIR=${LOG_DIR:-/tmp/robocup_garbage_realtime_$(date +%Y%m%d_%H%M%S)}
PRIMARY_MODEL_PATH=${PRIMARY_MODEL_PATH:-$DEFAULT_PRIMARY_MODEL_PATH}
PRIMARY_CONFIDENCE_THRESHOLD=${PRIMARY_CONFIDENCE_THRESHOLD:-0.18}
PRIMARY_DETECTOR_CONFIDENCE_THRESHOLD=${PRIMARY_DETECTOR_CONFIDENCE_THRESHOLD:-$PRIMARY_CONFIDENCE_THRESHOLD}
PRIMARY_TRACK_CONFIDENCE_THRESHOLD=${PRIMARY_TRACK_CONFIDENCE_THRESHOLD:-$PRIMARY_CONFIDENCE_THRESHOLD}
ENABLE_PAPER_BOX_SUPPLEMENT=${ENABLE_PAPER_BOX_SUPPLEMENT:-true}
PAPER_BOX_DETECTOR_CONFIDENCE_THRESHOLD=${PAPER_BOX_DETECTOR_CONFIDENCE_THRESHOLD:-0.15}
PAPER_BOX_MODEL_PATH=${PAPER_BOX_MODEL_PATH:-$DEFAULT_PAPER_BOX_MODEL_PATH}
PAPER_BOX_CONFIDENCE_THRESHOLD=${PAPER_BOX_CONFIDENCE_THRESHOLD:-0.35}
ENABLE_PAPER_BOX_SECONDARY=${ENABLE_PAPER_BOX_SECONDARY:-false}
PAPER_BOX_SECONDARY_MODEL_PATH=${PAPER_BOX_SECONDARY_MODEL_PATH:-$DEFAULT_PAPER_BALL_SPECIALIST_MODEL_PATH}
PAPER_BOX_SECONDARY_DETECTOR_CONFIDENCE_THRESHOLD=${PAPER_BOX_SECONDARY_DETECTOR_CONFIDENCE_THRESHOLD:-0.12}
PAPER_BOX_SECONDARY_CONFIDENCE_THRESHOLD=${PAPER_BOX_SECONDARY_CONFIDENCE_THRESHOLD:-0.40}
PAPER_BOX_TEMPORAL_HOLD_MINIMUM_COUNT=${PAPER_BOX_TEMPORAL_HOLD_MINIMUM_COUNT:-0}
PAPER_BOX_TEMPORAL_HOLD_SECONDS=${PAPER_BOX_TEMPORAL_HOLD_SECONDS:-0.0}
PAPER_BOX_FALLBACK_CENTER_CROP_SCALE=${PAPER_BOX_FALLBACK_CENTER_CROP_SCALE:-1.0}
PAPER_BOX_FALLBACK_CROP_MODE=${PAPER_BOX_FALLBACK_CROP_MODE:-wide}
PAPER_BOX_SECONDARY_TEMPORAL_HOLD_MINIMUM_COUNT=${PAPER_BOX_SECONDARY_TEMPORAL_HOLD_MINIMUM_COUNT:-0}
PAPER_BOX_SECONDARY_TEMPORAL_HOLD_SECONDS=${PAPER_BOX_SECONDARY_TEMPORAL_HOLD_SECONDS:-0.0}
PAPER_BOX_SECONDARY_FALLBACK_CENTER_CROP_SCALE=${PAPER_BOX_SECONDARY_FALLBACK_CENTER_CROP_SCALE:-1.0}
PAPER_BOX_SECONDARY_FALLBACK_CROP_MODE=${PAPER_BOX_SECONDARY_FALLBACK_CROP_MODE:-wide}
ENABLE_PAPER_BALL_RGBD_VOTE=${ENABLE_PAPER_BALL_RGBD_VOTE:-false}
PAPER_BALL_RGBD_VOTE_DEPTH_TOPIC=${PAPER_BALL_RGBD_VOTE_DEPTH_TOPIC:-/kinect2/hd/image_depth_rect}
PAPER_BALL_RGBD_VOTE_POINT_CLOUD_TOPIC=${PAPER_BALL_RGBD_VOTE_POINT_CLOUD_TOPIC:-/kinect2/hd/points}
PAPER_BALL_RGBD_VOTE_CAMERA_INFO_TOPIC=${PAPER_BALL_RGBD_VOTE_CAMERA_INFO_TOPIC:-/kinect2/hd/camera_info}
PAPER_BALL_RGBD_VOTE_OUTPUT_FRAME=${PAPER_BALL_RGBD_VOTE_OUTPUT_FRAME:-base_link}
PAPER_BALL_RGBD_VOTE_INPUT_TOPIC=${PAPER_BALL_RGBD_VOTE_INPUT_TOPIC:-/yolo_world/paper_box_secondary_detections}
PAPER_BALL_RGBD_VOTE_OUTPUT_TOPIC=${PAPER_BALL_RGBD_VOTE_OUTPUT_TOPIC:-/yolo_world/paper_box_secondary_rgbd_voted_detections}
PAPER_BALL_RGBD_VOTE_LOW_CONFIDENCE_THRESHOLD=${PAPER_BALL_RGBD_VOTE_LOW_CONFIDENCE_THRESHOLD:-$PAPER_BOX_SECONDARY_DETECTOR_CONFIDENCE_THRESHOLD}
PAPER_BALL_RGBD_VOTE_IMMEDIATE_CONFIDENCE_THRESHOLD=${PAPER_BALL_RGBD_VOTE_IMMEDIATE_CONFIDENCE_THRESHOLD:-$PAPER_BOX_SECONDARY_CONFIDENCE_THRESHOLD}
PAPER_BALL_RGBD_VOTE_PUBLISH_CONFIDENCE_FLOOR=${PAPER_BALL_RGBD_VOTE_PUBLISH_CONFIDENCE_FLOOR:-$PAPER_BOX_SECONDARY_CONFIDENCE_THRESHOLD}
PAPER_BALL_RGBD_VOTE_MINIMUM_VOTES=${PAPER_BALL_RGBD_VOTE_MINIMUM_VOTES:-2}
PAPER_BALL_RGBD_VOTE_WINDOW_SECONDS=${PAPER_BALL_RGBD_VOTE_WINDOW_SECONDS:-0.90}
PAPER_BALL_RGBD_VOTE_HOLD_SECONDS=${PAPER_BALL_RGBD_VOTE_HOLD_SECONDS:-0.35}
PAPER_BALL_RGBD_VOTE_MAX_3D_MATCH_DISTANCE=${PAPER_BALL_RGBD_VOTE_MAX_3D_MATCH_DISTANCE:-0.18}
PAPER_BALL_RGBD_VOTE_MAX_IMAGE_MATCH_DISTANCE=${PAPER_BALL_RGBD_VOTE_MAX_IMAGE_MATCH_DISTANCE:-28.0}
PAPER_BALL_RGBD_VOTE_MIN_CENTER_Y_RATIO=${PAPER_BALL_RGBD_VOTE_MIN_CENTER_Y_RATIO:-0.20}
PAPER_BALL_RGBD_VOTE_MAX_CLOUD_TIME_DELTA=${PAPER_BALL_RGBD_VOTE_MAX_CLOUD_TIME_DELTA:-0.15}
PAPER_BALL_RGBD_VOTE_TF_TIMEOUT=${PAPER_BALL_RGBD_VOTE_TF_TIMEOUT:-0.05}
PAPER_BALL_RGBD_VOTE_MIN_GROUND_Z=${PAPER_BALL_RGBD_VOTE_MIN_GROUND_Z:--0.60}
PAPER_BALL_RGBD_VOTE_MAX_GROUND_Z=${PAPER_BALL_RGBD_VOTE_MAX_GROUND_Z:-0.25}
PAPER_BALL_RGBD_VOTE_USE_POINT_CLOUD=${PAPER_BALL_RGBD_VOTE_USE_POINT_CLOUD:-true}
PAPER_BALL_RGBD_VOTE_ENABLE_GROUND_Z_FILTER=${PAPER_BALL_RGBD_VOTE_ENABLE_GROUND_Z_FILTER:-true}
PAPER_BALL_RGBD_VOTE_REQUIRE_OUTPUT_FRAME_FOR_VOTE=${PAPER_BALL_RGBD_VOTE_REQUIRE_OUTPUT_FRAME_FOR_VOTE:-false}
PAPER_BALL_RGBD_VOTE_REQUIRE_DEPTH_FOR_VOTE=${PAPER_BALL_RGBD_VOTE_REQUIRE_DEPTH_FOR_VOTE:-true}
PAPER_BALL_RGBD_VOTE_FILTER_HIGH_CONFIDENCE_WITH_DEPTH=${PAPER_BALL_RGBD_VOTE_FILTER_HIGH_CONFIDENCE_WITH_DEPTH:-true}
PAPER_BALL_RGBD_VOTE_ALLOW_HIGH_CONFIDENCE_WITHOUT_DEPTH=${PAPER_BALL_RGBD_VOTE_ALLOW_HIGH_CONFIDENCE_WITHOUT_DEPTH:-true}
PAPER_BALL_RGBD_VOTE_ENABLE_HSV_GROUND_FILTER=${PAPER_BALL_RGBD_VOTE_ENABLE_HSV_GROUND_FILTER:-true}
PAPER_BALL_RGBD_VOTE_REQUIRE_HSV_GROUND_FILTER=${PAPER_BALL_RGBD_VOTE_REQUIRE_HSV_GROUND_FILTER:-true}
PAPER_BALL_RGBD_VOTE_MAX_HSV_IMAGE_TIME_DELTA=${PAPER_BALL_RGBD_VOTE_MAX_HSV_IMAGE_TIME_DELTA:-0.50}
PAPER_BALL_RGBD_VOTE_HSV_GROUND_MAX_SATURATION=${PAPER_BALL_RGBD_VOTE_HSV_GROUND_MAX_SATURATION:-70}
PAPER_BALL_RGBD_VOTE_HSV_GROUND_MIN_VALUE=${PAPER_BALL_RGBD_VOTE_HSV_GROUND_MIN_VALUE:-20}
PAPER_BALL_RGBD_VOTE_HSV_GROUND_MAX_VALUE=${PAPER_BALL_RGBD_VOTE_HSV_GROUND_MAX_VALUE:-255}
PAPER_BALL_RGBD_VOTE_ENABLE_HSV_CLAHE=${PAPER_BALL_RGBD_VOTE_ENABLE_HSV_CLAHE:-true}
PAPER_BALL_RGBD_VOTE_HSV_CLAHE_CLIP_LIMIT=${PAPER_BALL_RGBD_VOTE_HSV_CLAHE_CLIP_LIMIT:-2.0}
PAPER_BALL_RGBD_VOTE_HSV_CLAHE_TILE_GRID_SIZE=${PAPER_BALL_RGBD_VOTE_HSV_CLAHE_TILE_GRID_SIZE:-8}
PAPER_BALL_RGBD_VOTE_GROUND_MASK_MORPH_KERNEL=${PAPER_BALL_RGBD_VOTE_GROUND_MASK_MORPH_KERNEL:-5}
PAPER_BALL_RGBD_VOTE_GROUND_MASK_BOTTOM_BAND_RATIO=${PAPER_BALL_RGBD_VOTE_GROUND_MASK_BOTTOM_BAND_RATIO:-0.10}
PAPER_BALL_RGBD_VOTE_GROUND_MASK_MIN_BOTTOM_PIXELS=${PAPER_BALL_RGBD_VOTE_GROUND_MASK_MIN_BOTTOM_PIXELS:-10}
PAPER_BALL_RGBD_VOTE_GROUND_CONTACT_BAND_RATIO=${PAPER_BALL_RGBD_VOTE_GROUND_CONTACT_BAND_RATIO:-0.30}
PAPER_BALL_RGBD_VOTE_GROUND_CONTACT_MIN_RATIO=${PAPER_BALL_RGBD_VOTE_GROUND_CONTACT_MIN_RATIO:-0.03}
PAPER_BALL_RGBD_VOTE_GROUND_BELOW_BAND_RATIO=${PAPER_BALL_RGBD_VOTE_GROUND_BELOW_BAND_RATIO:-0.18}
PAPER_BALL_RGBD_VOTE_GROUND_BELOW_WIDTH_RATIO=${PAPER_BALL_RGBD_VOTE_GROUND_BELOW_WIDTH_RATIO:-0.70}
PAPER_BALL_RGBD_VOTE_GROUND_BELOW_MIN_RATIO=${PAPER_BALL_RGBD_VOTE_GROUND_BELOW_MIN_RATIO:-0.10}
PAPER_BALL_RGBD_VOTE_GROUND_MASK_DEBUG_TOPIC=${PAPER_BALL_RGBD_VOTE_GROUND_MASK_DEBUG_TOPIC:-/yolo_world/paper_ball_ground_mask}
ENABLE_BYTETRACK=${ENABLE_BYTETRACK:-true}
PRIMARY_BYTETRACK_CLASSES=${PRIMARY_BYTETRACK_CLASSES:-person,cup,mug,bottle,bowl,plate,fork,knife,spoon,chopsticks,paper_ball,box}
PRIMARY_BYTETRACK_LOW_CONFIDENCE_THRESHOLD=${PRIMARY_BYTETRACK_LOW_CONFIDENCE_THRESHOLD:-$PRIMARY_DETECTOR_CONFIDENCE_THRESHOLD}
PRIMARY_BYTETRACK_MATCH_IOU_THRESHOLD=${PRIMARY_BYTETRACK_MATCH_IOU_THRESHOLD:-0.30}
PRIMARY_BYTETRACK_MAX_LOST_SECONDS=${PRIMARY_BYTETRACK_MAX_LOST_SECONDS:-0.50}
PRIMARY_BYTETRACK_PUBLISH_LOST_TRACKS=${PRIMARY_BYTETRACK_PUBLISH_LOST_TRACKS:-true}
PRIMARY_BYTETRACK_SUPPRESS_LOST_TRACK_CLASSES=${PRIMARY_BYTETRACK_SUPPRESS_LOST_TRACK_CLASSES:-}
PRIMARY_BYTETRACK_MIN_TRACK_HITS=${PRIMARY_BYTETRACK_MIN_TRACK_HITS:-1}
PRIMARY_BYTETRACK_DELAY_OUTPUT_CLASSES=${PRIMARY_BYTETRACK_DELAY_OUTPUT_CLASSES:-}
BYTETRACK_LOW_CONFIDENCE_THRESHOLD=${BYTETRACK_LOW_CONFIDENCE_THRESHOLD:-0.15}
BYTETRACK_MATCH_IOU_THRESHOLD=${BYTETRACK_MATCH_IOU_THRESHOLD:-0.30}
BYTETRACK_MAX_LOST_SECONDS=${BYTETRACK_MAX_LOST_SECONDS:-0.70}
BYTETRACK_PUBLISH_LOST_TRACKS=${BYTETRACK_PUBLISH_LOST_TRACKS:-true}
PAPER_BOX_BYTETRACK_SUPPRESS_LOST_TRACK_CLASSES=${PAPER_BOX_BYTETRACK_SUPPRESS_LOST_TRACK_CLASSES:-}
PAPER_BOX_SECONDARY_BYTETRACK_SUPPRESS_LOST_TRACK_CLASSES=${PAPER_BOX_SECONDARY_BYTETRACK_SUPPRESS_LOST_TRACK_CLASSES:-}
PAPER_BOX_BYTETRACK_MIN_TRACK_HITS=${PAPER_BOX_BYTETRACK_MIN_TRACK_HITS:-1}
PAPER_BOX_SECONDARY_BYTETRACK_MIN_TRACK_HITS=${PAPER_BOX_SECONDARY_BYTETRACK_MIN_TRACK_HITS:-1}
PAPER_BOX_BYTETRACK_DELAY_OUTPUT_CLASSES=${PAPER_BOX_BYTETRACK_DELAY_OUTPUT_CLASSES:-}
PAPER_BOX_SECONDARY_BYTETRACK_DELAY_OUTPUT_CLASSES=${PAPER_BOX_SECONDARY_BYTETRACK_DELAY_OUTPUT_CLASSES:-}
PAPER_BOX_SECONDARY_TRACKED_TOPIC=${PAPER_BOX_SECONDARY_TRACKED_TOPIC:-/yolo_world/paper_box_secondary_tracked_detections}
if [[ "$ENABLE_PAPER_BOX_SUPPLEMENT" == "true" ]]; then
  PAPER_BOX_MERGE_TOPIC=${PAPER_BOX_MERGE_TOPIC:-/yolo_world/paper_box_tracked_detections}
  PAPER_BOX_UNTRACKED_MERGE_TOPIC=${PAPER_BOX_UNTRACKED_MERGE_TOPIC:-/yolo_world/paper_box_detections}
else
  PAPER_BOX_MERGE_TOPIC=${PAPER_BOX_MERGE_TOPIC:-/yolo_world/disabled_paper_box_detections}
  PAPER_BOX_UNTRACKED_MERGE_TOPIC=${PAPER_BOX_UNTRACKED_MERGE_TOPIC:-/yolo_world/disabled_paper_box_detections}
fi
if [[ "$ENABLE_PAPER_BALL_RGBD_VOTE" == "true" ]]; then
  PAPER_BOX_SECONDARY_MERGE_TOPIC=${PAPER_BOX_SECONDARY_MERGE_TOPIC:-$PAPER_BALL_RGBD_VOTE_OUTPUT_TOPIC}
  PAPER_BOX_SECONDARY_UNTRACKED_MERGE_TOPIC=${PAPER_BOX_SECONDARY_UNTRACKED_MERGE_TOPIC:-$PAPER_BALL_RGBD_VOTE_OUTPUT_TOPIC}
else
  PAPER_BOX_SECONDARY_MERGE_TOPIC=${PAPER_BOX_SECONDARY_MERGE_TOPIC:-$PAPER_BOX_SECONDARY_TRACKED_TOPIC}
  PAPER_BOX_SECONDARY_UNTRACKED_MERGE_TOPIC=${PAPER_BOX_SECONDARY_UNTRACKED_MERGE_TOPIC:-/yolo_world/paper_box_secondary_detections}
fi
PRIMARY_BOX_MIN_CONFIDENCE=${PRIMARY_BOX_MIN_CONFIDENCE:-0.0}
SUPPLEMENTAL_BOX_MIN_CONFIDENCE=${SUPPLEMENTAL_BOX_MIN_CONFIDENCE:-0.0}
BOX_MERGE_IOU_THRESHOLD=${BOX_MERGE_IOU_THRESHOLD:-0.45}
BOX_PERSON_OVERLAP_IOU_THRESHOLD=${BOX_PERSON_OVERLAP_IOU_THRESHOLD:-1.0}
BOX_MAX_ASPECT_RATIO=${BOX_MAX_ASPECT_RATIO:-100.0}

mkdir -p "$LOG_DIR"

if [[ -d "$ONNXRUNTIME_LIB_DIR" ]]; then
  export LD_LIBRARY_PATH="$ONNXRUNTIME_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

if [[ "$CAMERA_SOURCE" == "usb" ]]; then
  if [[ ! -e "$VIDEO_DEVICE" ]]; then
    echo "ERROR: camera device not found: $VIDEO_DEVICE"
    echo "Check available devices with: ls /dev/video*"
    exit 1
  fi

  if [[ ! -r "$VIDEO_DEVICE" ]]; then
    echo "ERROR: current user cannot read $VIDEO_DEVICE"
    echo "Add the user to the video group, then log out and back in: sudo usermod -aG video \"$USER\""
    exit 1
  fi
fi

MODEL_PATHS=("$PRIMARY_MODEL_PATH")
if [[ "$ENABLE_PAPER_BOX_SUPPLEMENT" == "true" ]]; then
  MODEL_PATHS+=("$PAPER_BOX_MODEL_PATH")
fi
for model_path in "${MODEL_PATHS[@]}"; do
  if [[ ! -f "$model_path" ]]; then
    echo "ERROR: model file not found: $model_path"
    exit 1
  fi
done
if [[ "$ENABLE_PAPER_BOX_SECONDARY" == "true" && ! -f "$PAPER_BOX_SECONDARY_MODEL_PATH" ]]; then
  echo "ERROR: secondary model file not found: $PAPER_BOX_SECONDARY_MODEL_PATH"
  exit 1
fi

PIDS=()

cleanup() {
  echo
  echo "Stopping RoboCup garbage realtime recognition..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

remove_pid() {
  local remove="$1"
  local kept=()
  local pid
  for pid in "${PIDS[@]:-}"; do
    if [[ "$pid" != "$remove" ]]; then
      kept+=("$pid")
    fi
  done
  PIDS=("${kept[@]}")
}

wait_for_image_frame() {
  local topic="$1"
  local attempts="${2:-16}"
  local timeout_seconds="${3:-3s}"
  local attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    # Avoid serializing the full image data array; large raw frames can make
    # plain rostopic echo exceed the timeout even when the camera is publishing.
    if timeout "$timeout_seconds" rostopic echo --noarr -n 1 "$topic" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

try_start_usb_camera() {
  local width="$1"
  local height="$2"
  local pixel_format="$3"
  local color_format="$4"
  local safe_pixel_format="${pixel_format//[^A-Za-z0-9_]/_}"
  local log_file="$LOG_DIR/usb_cam_${width}x${height}_${safe_pixel_format}.log"
  local pid

  echo "Starting USB camera on $VIDEO_DEVICE at ${width}x${height} (${pixel_format})..."
  rosrun usb_cam usb_cam_node \
    _video_device:="$VIDEO_DEVICE" \
    _image_width:="$width" \
    _image_height:="$height" \
    _pixel_format:="$pixel_format" \
    _color_format:="$color_format" \
    _camera_frame_id:="$CAMERA_FRAME_ID" \
    _io_method:="$CAMERA_IO_METHOD" \
    >"$log_file" 2>&1 &
  pid="$!"
  PIDS+=("$pid")

  echo "Waiting for $CAMERA_IMAGE_TOPIC..."
  if wait_for_image_frame "$CAMERA_IMAGE_TOPIC" 16 3s; then
    CAMERA_IMAGE_WIDTH="$width"
    CAMERA_IMAGE_HEIGHT="$height"
    CAMERA_PIXEL_FORMAT="$pixel_format"
    CAMERA_COLOR_FORMAT="$color_format"
    USB_CAM_LOG="$log_file"
    return 0
  fi

  kill "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  remove_pid "$pid"
  rosnode kill /usb_cam >/dev/null 2>&1 || true

  echo "Camera mode failed: ${width}x${height} ${pixel_format}/${color_format}"
  echo "Log tail: $log_file"
  tail -n 20 "$log_file" 2>/dev/null || true
  return 1
}

try_start_kinect2_camera() {
  local log_file="$LOG_DIR/kinect2_bridge.log"
  local pid

  if wait_for_image_frame "$CAMERA_IMAGE_TOPIC" 2 1s; then
    echo "Using existing WPB Kinect2 image topic: $CAMERA_IMAGE_TOPIC"
    return 0
  fi

  if [[ "$KINECT2_START_BRIDGE" != "true" ]]; then
    echo "Kinect2 bridge autostart disabled; waiting topic is missing: $CAMERA_IMAGE_TOPIC"
    return 1
  fi

  echo "Starting WPB Kinect2 bridge..."
  roslaunch kinect2_bridge kinect2_bridge.launch >"$log_file" 2>&1 &
  pid="$!"
  PIDS+=("$pid")

  echo "Waiting for $CAMERA_IMAGE_TOPIC..."
  if wait_for_image_frame "$CAMERA_IMAGE_TOPIC" 24 2s; then
    KINECT2_LOG="$log_file"
    return 0
  fi

  kill "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  remove_pid "$pid"

  echo "ERROR: Kinect2 bridge did not publish $CAMERA_IMAGE_TOPIC"
  echo "Log tail: $log_file"
  tail -n 30 "$log_file" 2>/dev/null || true
  return 1
}

if rostopic list >/dev/null 2>&1; then
  echo "Using existing ROS master."
else
  echo "Starting roscore..."
  roscore >"$LOG_DIR/roscore.log" 2>&1 &
  PIDS+=("$!")
  for _ in {1..30}; do
    if rostopic list >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done
fi

CAMERA_STARTED=false
REQUESTED_VIDEO_DEVICE="${VIDEO_DEVICE:-}"

try_camera_fallback_modes() {
  try_start_usb_camera 640 480 yuyv yuv422p || \
    try_start_usb_camera 640 480 mjpeg yuv422p
}

if [[ "$CAMERA_SOURCE" == "kinect2" ]]; then
  if try_start_kinect2_camera; then
    CAMERA_STARTED=true
  fi
else
  if try_start_usb_camera "$CAMERA_IMAGE_WIDTH" "$CAMERA_IMAGE_HEIGHT" "$CAMERA_PIXEL_FORMAT" "$CAMERA_COLOR_FORMAT"; then
    CAMERA_STARTED=true
  elif [[ "$CAMERA_DISABLE_FALLBACK" != "true" ]]; then
    echo "Retrying camera with conservative fallback modes on $VIDEO_DEVICE..."
    if try_camera_fallback_modes; then
      CAMERA_STARTED=true
    elif [[ "$CAMERA_DISABLE_DEVICE_FALLBACK" != "true" ]]; then
      for candidate_device in /dev/video*; do
        if [[ ! -e "$candidate_device" || "$candidate_device" == "$REQUESTED_VIDEO_DEVICE" || ! -r "$candidate_device" ]]; then
          continue
        fi
        VIDEO_DEVICE="$candidate_device"
        echo "Retrying camera on alternate device $VIDEO_DEVICE..."
        if try_start_usb_camera "$CAMERA_IMAGE_WIDTH" "$CAMERA_IMAGE_HEIGHT" "$CAMERA_PIXEL_FORMAT" "$CAMERA_COLOR_FORMAT" || \
           try_camera_fallback_modes; then
          CAMERA_STARTED=true
          break
        fi
      done
    fi
  fi
fi

if [[ "$CAMERA_STARTED" != "true" ]]; then
  echo "ERROR: camera did not publish $CAMERA_IMAGE_TOPIC"
  if [[ "$CAMERA_SOURCE" == "usb" ]]; then
    echo "Requested device: $REQUESTED_VIDEO_DEVICE"
    echo "Check supported modes with: v4l2-ctl -d $REQUESTED_VIDEO_DEVICE --list-formats-ext"
  else
    echo "This WPB profile expects Kinect2 topics from kinect2_bridge."
  fi
  echo "Logs: $LOG_DIR"
  exit 1
fi

case "$MODE" in
  enhanced)
    echo "Starting enhanced garbage detector..."
    roslaunch yolo_world_detector yolo_world_enhanced_cascade_camera.launch \
      image_topic:="$CAMERA_IMAGE_TOPIC" \
      publish_annotated_image:=true \
      primary_model_path:="$PRIMARY_MODEL_PATH" \
      primary_confidence_threshold:="$PRIMARY_CONFIDENCE_THRESHOLD" \
      primary_detector_confidence_threshold:="$PRIMARY_DETECTOR_CONFIDENCE_THRESHOLD" \
      primary_track_confidence_threshold:="$PRIMARY_TRACK_CONFIDENCE_THRESHOLD" \
      primary_bytetrack_classes:="$PRIMARY_BYTETRACK_CLASSES" \
      primary_bytetrack_low_confidence_threshold:="$PRIMARY_BYTETRACK_LOW_CONFIDENCE_THRESHOLD" \
      primary_bytetrack_match_iou_threshold:="$PRIMARY_BYTETRACK_MATCH_IOU_THRESHOLD" \
      primary_bytetrack_max_lost_seconds:="$PRIMARY_BYTETRACK_MAX_LOST_SECONDS" \
      primary_bytetrack_publish_lost_tracks:="$PRIMARY_BYTETRACK_PUBLISH_LOST_TRACKS" \
      primary_bytetrack_suppress_lost_track_classes:="$PRIMARY_BYTETRACK_SUPPRESS_LOST_TRACK_CLASSES" \
      primary_bytetrack_min_track_hits:="$PRIMARY_BYTETRACK_MIN_TRACK_HITS" \
      primary_bytetrack_delay_output_classes:="$PRIMARY_BYTETRACK_DELAY_OUTPUT_CLASSES" \
      enable_paper_box_supplement:="$ENABLE_PAPER_BOX_SUPPLEMENT" \
      paper_box_detector_confidence_threshold:="$PAPER_BOX_DETECTOR_CONFIDENCE_THRESHOLD" \
      paper_box_model_path:="$PAPER_BOX_MODEL_PATH" \
      paper_box_confidence_threshold:="$PAPER_BOX_CONFIDENCE_THRESHOLD" \
      enable_paper_box_secondary:="$ENABLE_PAPER_BOX_SECONDARY" \
      paper_box_secondary_model_path:="$PAPER_BOX_SECONDARY_MODEL_PATH" \
      paper_box_secondary_detector_confidence_threshold:="$PAPER_BOX_SECONDARY_DETECTOR_CONFIDENCE_THRESHOLD" \
      paper_box_secondary_confidence_threshold:="$PAPER_BOX_SECONDARY_CONFIDENCE_THRESHOLD" \
      paper_box_temporal_hold_minimum_count:="$PAPER_BOX_TEMPORAL_HOLD_MINIMUM_COUNT" \
      paper_box_temporal_hold_seconds:="$PAPER_BOX_TEMPORAL_HOLD_SECONDS" \
      paper_box_fallback_center_crop_scale:="$PAPER_BOX_FALLBACK_CENTER_CROP_SCALE" \
      paper_box_fallback_crop_mode:="$PAPER_BOX_FALLBACK_CROP_MODE" \
      paper_box_secondary_temporal_hold_minimum_count:="$PAPER_BOX_SECONDARY_TEMPORAL_HOLD_MINIMUM_COUNT" \
      paper_box_secondary_temporal_hold_seconds:="$PAPER_BOX_SECONDARY_TEMPORAL_HOLD_SECONDS" \
      paper_box_secondary_fallback_center_crop_scale:="$PAPER_BOX_SECONDARY_FALLBACK_CENTER_CROP_SCALE" \
      paper_box_secondary_fallback_crop_mode:="$PAPER_BOX_SECONDARY_FALLBACK_CROP_MODE" \
      enable_paper_ball_rgbd_vote:="$ENABLE_PAPER_BALL_RGBD_VOTE" \
      paper_ball_rgbd_vote_depth_topic:="$PAPER_BALL_RGBD_VOTE_DEPTH_TOPIC" \
      paper_ball_rgbd_vote_point_cloud_topic:="$PAPER_BALL_RGBD_VOTE_POINT_CLOUD_TOPIC" \
      paper_ball_rgbd_vote_camera_info_topic:="$PAPER_BALL_RGBD_VOTE_CAMERA_INFO_TOPIC" \
      paper_ball_rgbd_vote_output_frame:="$PAPER_BALL_RGBD_VOTE_OUTPUT_FRAME" \
      paper_ball_rgbd_vote_input_topic:="$PAPER_BALL_RGBD_VOTE_INPUT_TOPIC" \
      paper_ball_rgbd_vote_output_topic:="$PAPER_BALL_RGBD_VOTE_OUTPUT_TOPIC" \
      paper_box_merge_topic:="$PAPER_BOX_MERGE_TOPIC" \
      paper_box_untracked_merge_topic:="$PAPER_BOX_UNTRACKED_MERGE_TOPIC" \
      paper_box_secondary_merge_topic:="$PAPER_BOX_SECONDARY_MERGE_TOPIC" \
      paper_box_secondary_untracked_merge_topic:="$PAPER_BOX_SECONDARY_UNTRACKED_MERGE_TOPIC" \
      paper_ball_rgbd_vote_low_confidence_threshold:="$PAPER_BALL_RGBD_VOTE_LOW_CONFIDENCE_THRESHOLD" \
      paper_ball_rgbd_vote_immediate_confidence_threshold:="$PAPER_BALL_RGBD_VOTE_IMMEDIATE_CONFIDENCE_THRESHOLD" \
      paper_ball_rgbd_vote_publish_confidence_floor:="$PAPER_BALL_RGBD_VOTE_PUBLISH_CONFIDENCE_FLOOR" \
      paper_ball_rgbd_vote_minimum_votes:="$PAPER_BALL_RGBD_VOTE_MINIMUM_VOTES" \
      paper_ball_rgbd_vote_window_seconds:="$PAPER_BALL_RGBD_VOTE_WINDOW_SECONDS" \
      paper_ball_rgbd_vote_hold_seconds:="$PAPER_BALL_RGBD_VOTE_HOLD_SECONDS" \
      paper_ball_rgbd_vote_max_3d_match_distance:="$PAPER_BALL_RGBD_VOTE_MAX_3D_MATCH_DISTANCE" \
      paper_ball_rgbd_vote_max_image_match_distance:="$PAPER_BALL_RGBD_VOTE_MAX_IMAGE_MATCH_DISTANCE" \
      paper_ball_rgbd_vote_min_center_y_ratio:="$PAPER_BALL_RGBD_VOTE_MIN_CENTER_Y_RATIO" \
      paper_ball_rgbd_vote_max_cloud_time_delta:="$PAPER_BALL_RGBD_VOTE_MAX_CLOUD_TIME_DELTA" \
      paper_ball_rgbd_vote_tf_timeout:="$PAPER_BALL_RGBD_VOTE_TF_TIMEOUT" \
      paper_ball_rgbd_vote_min_ground_z:="$PAPER_BALL_RGBD_VOTE_MIN_GROUND_Z" \
      paper_ball_rgbd_vote_max_ground_z:="$PAPER_BALL_RGBD_VOTE_MAX_GROUND_Z" \
      paper_ball_rgbd_vote_use_point_cloud:="$PAPER_BALL_RGBD_VOTE_USE_POINT_CLOUD" \
      paper_ball_rgbd_vote_enable_ground_z_filter:="$PAPER_BALL_RGBD_VOTE_ENABLE_GROUND_Z_FILTER" \
      paper_ball_rgbd_vote_require_output_frame_for_vote:="$PAPER_BALL_RGBD_VOTE_REQUIRE_OUTPUT_FRAME_FOR_VOTE" \
      paper_ball_rgbd_vote_require_depth_for_vote:="$PAPER_BALL_RGBD_VOTE_REQUIRE_DEPTH_FOR_VOTE" \
      paper_ball_rgbd_vote_filter_high_confidence_with_depth:="$PAPER_BALL_RGBD_VOTE_FILTER_HIGH_CONFIDENCE_WITH_DEPTH" \
      paper_ball_rgbd_vote_allow_high_confidence_without_depth:="$PAPER_BALL_RGBD_VOTE_ALLOW_HIGH_CONFIDENCE_WITHOUT_DEPTH" \
      paper_ball_rgbd_vote_enable_hsv_ground_filter:="$PAPER_BALL_RGBD_VOTE_ENABLE_HSV_GROUND_FILTER" \
      paper_ball_rgbd_vote_require_hsv_ground_filter:="$PAPER_BALL_RGBD_VOTE_REQUIRE_HSV_GROUND_FILTER" \
      paper_ball_rgbd_vote_max_hsv_image_time_delta:="$PAPER_BALL_RGBD_VOTE_MAX_HSV_IMAGE_TIME_DELTA" \
      paper_ball_rgbd_vote_hsv_ground_max_saturation:="$PAPER_BALL_RGBD_VOTE_HSV_GROUND_MAX_SATURATION" \
      paper_ball_rgbd_vote_hsv_ground_min_value:="$PAPER_BALL_RGBD_VOTE_HSV_GROUND_MIN_VALUE" \
      paper_ball_rgbd_vote_hsv_ground_max_value:="$PAPER_BALL_RGBD_VOTE_HSV_GROUND_MAX_VALUE" \
      paper_ball_rgbd_vote_enable_hsv_clahe:="$PAPER_BALL_RGBD_VOTE_ENABLE_HSV_CLAHE" \
      paper_ball_rgbd_vote_hsv_clahe_clip_limit:="$PAPER_BALL_RGBD_VOTE_HSV_CLAHE_CLIP_LIMIT" \
      paper_ball_rgbd_vote_hsv_clahe_tile_grid_size:="$PAPER_BALL_RGBD_VOTE_HSV_CLAHE_TILE_GRID_SIZE" \
      paper_ball_rgbd_vote_ground_mask_morph_kernel:="$PAPER_BALL_RGBD_VOTE_GROUND_MASK_MORPH_KERNEL" \
      paper_ball_rgbd_vote_ground_mask_bottom_band_ratio:="$PAPER_BALL_RGBD_VOTE_GROUND_MASK_BOTTOM_BAND_RATIO" \
      paper_ball_rgbd_vote_ground_mask_min_bottom_pixels:="$PAPER_BALL_RGBD_VOTE_GROUND_MASK_MIN_BOTTOM_PIXELS" \
      paper_ball_rgbd_vote_ground_contact_band_ratio:="$PAPER_BALL_RGBD_VOTE_GROUND_CONTACT_BAND_RATIO" \
      paper_ball_rgbd_vote_ground_contact_min_ratio:="$PAPER_BALL_RGBD_VOTE_GROUND_CONTACT_MIN_RATIO" \
      paper_ball_rgbd_vote_ground_below_band_ratio:="$PAPER_BALL_RGBD_VOTE_GROUND_BELOW_BAND_RATIO" \
      paper_ball_rgbd_vote_ground_below_width_ratio:="$PAPER_BALL_RGBD_VOTE_GROUND_BELOW_WIDTH_RATIO" \
      paper_ball_rgbd_vote_ground_below_min_ratio:="$PAPER_BALL_RGBD_VOTE_GROUND_BELOW_MIN_RATIO" \
      paper_ball_rgbd_vote_ground_mask_debug_topic:="$PAPER_BALL_RGBD_VOTE_GROUND_MASK_DEBUG_TOPIC" \
      primary_box_min_confidence:="$PRIMARY_BOX_MIN_CONFIDENCE" \
      supplemental_box_min_confidence:="$SUPPLEMENTAL_BOX_MIN_CONFIDENCE" \
      box_merge_iou_threshold:="$BOX_MERGE_IOU_THRESHOLD" \
      box_person_overlap_iou_threshold:="$BOX_PERSON_OVERLAP_IOU_THRESHOLD" \
      box_max_aspect_ratio:="$BOX_MAX_ASPECT_RATIO" \
      execution_provider:="$YOLO_EXECUTION_PROVIDER" \
      cuda_device_id:="$YOLO_CUDA_DEVICE_ID" \
      enable_bytetrack:="$ENABLE_BYTETRACK" \
      bytetrack_low_confidence_threshold:="$BYTETRACK_LOW_CONFIDENCE_THRESHOLD" \
      bytetrack_match_iou_threshold:="$BYTETRACK_MATCH_IOU_THRESHOLD" \
      bytetrack_max_lost_seconds:="$BYTETRACK_MAX_LOST_SECONDS" \
      bytetrack_publish_lost_tracks:="$BYTETRACK_PUBLISH_LOST_TRACKS" \
      paper_box_bytetrack_suppress_lost_track_classes:="$PAPER_BOX_BYTETRACK_SUPPRESS_LOST_TRACK_CLASSES" \
      paper_box_secondary_bytetrack_suppress_lost_track_classes:="$PAPER_BOX_SECONDARY_BYTETRACK_SUPPRESS_LOST_TRACK_CLASSES" \
      paper_box_bytetrack_min_track_hits:="$PAPER_BOX_BYTETRACK_MIN_TRACK_HITS" \
      paper_box_secondary_bytetrack_min_track_hits:="$PAPER_BOX_SECONDARY_BYTETRACK_MIN_TRACK_HITS" \
      paper_box_bytetrack_delay_output_classes:="$PAPER_BOX_BYTETRACK_DELAY_OUTPUT_CLASSES" \
      paper_box_secondary_bytetrack_delay_output_classes:="$PAPER_BOX_SECONDARY_BYTETRACK_DELAY_OUTPUT_CLASSES" \
      >"$LOG_DIR/detector.log" 2>&1 &
    ;;
  basic)
    echo "Starting basic garbage detector..."
    roslaunch yolo_world_detector yolo_world_garbage_camera.launch \
      image_topic:="$CAMERA_IMAGE_TOPIC" \
      model_path:="$PRIMARY_MODEL_PATH" \
      publish_annotated_image:=true \
      >"$LOG_DIR/detector.log" 2>&1 &
    ;;
  *)
    echo "ERROR: unknown MODE=$MODE. Use MODE=enhanced or MODE=basic."
    exit 1
    ;;
esac
PIDS+=("$!")

echo "Waiting for annotated detections on $VIEW_TOPIC..."
for _ in {1..40}; do
  if timeout 1s rostopic echo -n 1 "$VIEW_TOPIC" >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

echo "Opening annotated image viewer..."
rosrun image_view image_view image:="$VIEW_TOPIC" _autosize:=true \
  >"$LOG_DIR/viewer.log" 2>&1 &
PIDS+=("$!")

echo "RoboCup garbage realtime recognition is running."
echo "Camera source: $CAMERA_SOURCE"
echo "Image topic:   $CAMERA_IMAGE_TOPIC"
if [[ "$CAMERA_SOURCE" == "usb" ]]; then
  echo "Camera mode:   ${CAMERA_IMAGE_WIDTH}x${CAMERA_IMAGE_HEIGHT} ${CAMERA_PIXEL_FORMAT}/${CAMERA_COLOR_FORMAT}"
fi
echo "ONNX EP:     ${YOLO_EXECUTION_PROVIDER}"
echo "Detections: rostopic echo /yolo_world/enhanced_detections"
echo "Tracked primary: rostopic echo /yolo_world/primary_tracked_detections"
if [[ "$ENABLE_PAPER_BOX_SUPPLEMENT" == "true" ]]; then
  echo "Tracked paper/box supplement: rostopic echo /yolo_world/paper_box_tracked_detections"
else
  echo "Stable paper/box supplement: disabled"
fi
if [[ "$ENABLE_PAPER_BOX_SECONDARY" == "true" ]]; then
  echo "Tracked secondary paper supplement: rostopic echo /yolo_world/paper_box_secondary_tracked_detections"
fi
if [[ "$ENABLE_PAPER_BALL_RGBD_VOTE" == "true" ]]; then
  echo "RGB-D voted paper supplement: rostopic echo $PAPER_BALL_RGBD_VOTE_OUTPUT_TOPIC"
fi
echo "Frame rate:  rostopic hz /yolo_world/detections"
echo "Logs:        $LOG_DIR"
echo "Press Ctrl-C here to stop all started processes."

wait
