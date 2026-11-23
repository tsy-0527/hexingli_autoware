#!/usr/bin/env bash
# ROS 2 setup scripts reference optional variables that may be unset.
set -eo pipefail

WS_ROOT="/home/tang/autoware/vision_detection_ws"
D455_WS="${D455_WS:-/home/tang/tsy/D455_ROS2_Humble}"
MEDIAMTX_BIN="${WS_ROOT}/third_party/mediamtx/mediamtx"
MEDIAMTX_CONFIG="${WS_ROOT}/config/mediamtx.yml"
JPEG_PIPE="${WS_ROOT}/scripts/ros_compressed_to_stdout.py"
D455_DETECT_WRAPPER="${WS_ROOT}/scripts/d455_detect_with_sync_slop.py"
D455_V3_MODEL="${WS_ROOT}/models/d455_v3_best.pt"

CAMERA_STREAM_ENCODER="${CAMERA_STREAM_ENCODER:-libx264}"
CAMERA_STREAM_BITRATE="${CAMERA_STREAM_BITRATE:-600k}"
CAMERA_STREAM_BUFSIZE="${CAMERA_STREAM_BUFSIZE:-600k}"
CAMERA_STREAM_FPS="${CAMERA_STREAM_FPS:-15}"
CAMERA_STREAM_INPUT_FPS="${CAMERA_STREAM_INPUT_FPS:-15}"
CAMERA_STREAM_RTSP_URL="${CAMERA_STREAM_RTSP_URL:-rtsp://127.0.0.1:8554/cam}"
CAMERA_STREAM_LOCK="${CAMERA_STREAM_LOCK:-/tmp/autoware-camera-stream.lock}"
CAMERA_NODE_WAIT_SECONDS="${CAMERA_NODE_WAIT_SECONDS:-30}"
CAMERA_AUTO_START="${CAMERA_AUTO_START:-true}"
CAMERA_DETECTION_ENABLED="${CAMERA_DETECTION_ENABLED:-true}"
CAMERA_DETECTION_IMGSZ="${CAMERA_DETECTION_IMGSZ:-640}"
CAMERA_DETECTION_DEVICE="${CAMERA_DETECTION_DEVICE:-cpu}"
CAMERA_DETECTION_CONFIDENCE="${CAMERA_DETECTION_CONFIDENCE:-0.4}"
CAMERA_DETECTION_SYNC_MODE="${CAMERA_DETECTION_SYNC_MODE:-latest}"
CAMERA_DETECTION_SYNC_SLOP="${CAMERA_DETECTION_SYNC_SLOP:-0.1}"
CAMERA_DETECTION_DISABLE_ALIGN="${CAMERA_DETECTION_DISABLE_ALIGN:-true}"
CAMERA_DETECTION_MODEL="${CAMERA_DETECTION_MODEL:-${D455_V3_MODEL}}"
CAMERA_DETECTION_MODEL_SHA256="${CAMERA_DETECTION_MODEL_SHA256-52c7884b416dd11b0b6b004023e33b09012e6d1d82e8016321e64b28baccc637}"

source /opt/ros/humble/setup.bash
source /home/tang/autoware/install/setup.bash 2>/dev/null || true
source "${WS_ROOT}/install/setup.bash" 2>/dev/null || true
if [[ "${CAMERA_DETECTION_ENABLED}" == "true" ]]; then
  if [[ ! -f "${D455_WS}/install/local_setup.bash" ]]; then
    echo "D455 detection workspace is not built: ${D455_WS}" >&2
    exit 1
  fi
  source "${D455_WS}/install/local_setup.bash"
fi

exec 9>"${CAMERA_STREAM_LOCK}"
if ! flock -n 9; then
  echo "Camera stream is already running."
  echo "WebRTC: http://10.245.188.140:8889/cam"
  exit 0
fi

CAMERA_DRIVER_PID=""
DETECTION_PID=""
MEDIAMTX_PID=""
CAMERA_STREAM_IS_RAW="false"

cleanup() {
  if [[ -n "${MEDIAMTX_PID}" ]]; then
    kill "${MEDIAMTX_PID}" 2>/dev/null || true
    wait "${MEDIAMTX_PID}" 2>/dev/null || true
  fi
  if [[ -n "${DETECTION_PID}" ]]; then
    kill "${DETECTION_PID}" 2>/dev/null || true
    wait "${DETECTION_PID}" 2>/dev/null || true
  fi
  if [[ -n "${CAMERA_DRIVER_PID}" ]]; then
    kill "${CAMERA_DRIVER_PID}" 2>/dev/null || true
    wait "${CAMERA_DRIVER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

if [[ -z "${CAMERA_STREAM_TOPIC:-}" ]]; then
  CAMERA_CANDIDATES=(
    "/sensing/camera/camera0/color/image_raw/compressed"
    "/camera/color/image_raw/compressed"
  )

  CAMERA_TOPICS="$(ros2 topic list 2>/dev/null || true)"
  for candidate in "${CAMERA_CANDIDATES[@]}"; do
    if grep -Fxq "${candidate}" <<<"${CAMERA_TOPICS}" \
      && timeout 2 ros2 topic echo "${candidate}" --once >/dev/null 2>&1; then
      CAMERA_STREAM_TOPIC="${candidate}"
      break
    fi
  done

  if [[ -z "${CAMERA_STREAM_TOPIC:-}" && "${CAMERA_AUTO_START}" == "true" ]]; then
    echo "No live camera owner found; starting the RealSense driver for this stream..."
    ros2 launch livox_sensor_kit_launch camera.launch.xml \
      camera_namespace:=sensing/camera &
    CAMERA_DRIVER_PID=$!
  fi

  echo "Waiting for live RealSense camera frames..."
  CAMERA_WAIT_DEADLINE=$((SECONDS + CAMERA_NODE_WAIT_SECONDS))
  while ((SECONDS < CAMERA_WAIT_DEADLINE)); do
    CAMERA_TOPICS="$(ros2 topic list 2>/dev/null || true)"
    for candidate in "${CAMERA_CANDIDATES[@]}"; do
      # A stale/discovered camera node can remain visible without publishing any
      # frames. Select a source only after receiving a real compressed frame.
      if grep -Fxq "${candidate}" <<<"${CAMERA_TOPICS}" \
        && timeout 2 ros2 topic echo "${candidate}" --once >/dev/null 2>&1; then
        CAMERA_STREAM_TOPIC="${candidate}"
        break 2
      fi
    done
    sleep 1
  done

  if [[ -z "${CAMERA_STREAM_TOPIC:-}" ]]; then
    echo "No live RealSense camera frame appeared within ${CAMERA_NODE_WAIT_SECONDS} seconds." >&2
    if [[ "${CAMERA_AUTO_START}" == "true" ]]; then
      echo "The independently started RealSense driver did not produce a frame; check USB access and whether another process owns the camera." >&2
    else
      echo "CAMERA_AUTO_START=false; start a camera publisher first." >&2
    fi
    exit 3
  fi
fi

CAMERA_SOURCE_TOPIC="${CAMERA_STREAM_TOPIC}"

if [[ "${CAMERA_STREAM_TOPIC}" == /sensing/camera/camera0/* ]]; then
  ros2 param set \
    /sensing/camera/camera0 \
    .camera0.color.image_raw.jpeg_quality \
    70 >/dev/null 2>&1 || true
elif [[ "${CAMERA_STREAM_TOPIC}" == /camera/* ]]; then
  ros2 param set \
    /camera \
    camera.color.image_raw.jpeg_quality \
    70 >/dev/null 2>&1 || true
fi

if [[ "${CAMERA_DETECTION_ENABLED}" == "true" ]]; then
  if [[ "${CAMERA_DETECTION_DISABLE_ALIGN}" == "true" ]]; then
    # The detector aligns raw depth in software. Disabling the camera-side
    # syncer/align/filters avoids the asynchronous_syncer stall that otherwise
    # stops aligned_depth_to_color after a short time. Set
    # CAMERA_DETECTION_DISABLE_ALIGN=false if another consumer needs the
    # camera-side aligned depth topic.
    CAMERA_NODE=""
    case "${CAMERA_SOURCE_TOPIC}" in
      /sensing/camera/camera0/*) CAMERA_NODE="/sensing/camera/camera0" ;;
      /camera/*) CAMERA_NODE="/camera" ;;
    esac
    if [[ -n "${CAMERA_NODE}" ]]; then
      for param in \
        enable_sync \
        align_depth.enable \
        spatial_filter.enable \
        temporal_filter.enable \
        hole_filling_filter.enable
      do
        ros2 param set "${CAMERA_NODE}" "${param}" false >/dev/null 2>&1 || true
      done
    fi
  fi

  case "${CAMERA_SOURCE_TOPIC}" in
    /sensing/camera/camera0/*)
      DETECTION_COLOR_TOPIC="/sensing/camera/camera0/color/image_raw"
      DETECTION_DEPTH_TOPIC="/sensing/camera/camera0/depth/image_rect_raw"
      DETECTION_COLOR_INFO_TOPIC="/sensing/camera/camera0/color/camera_info"
      DETECTION_DEPTH_INFO_TOPIC="/sensing/camera/camera0/depth/camera_info"
      DETECTION_EXTRINSICS_TOPIC="/sensing/camera/camera0/extrinsics/depth_to_color"
      ;;
    /camera/*)
      DETECTION_COLOR_TOPIC="/camera/color/image_raw"
      DETECTION_DEPTH_TOPIC="/camera/depth/image_rect_raw"
      DETECTION_COLOR_INFO_TOPIC="/camera/color/camera_info"
      DETECTION_DEPTH_INFO_TOPIC="/camera/depth/camera_info"
      DETECTION_EXTRINSICS_TOPIC="/camera/extrinsics/depth_to_color"
      ;;
    *)
      echo "Cannot derive D455 RGB-D inputs from camera topic: ${CAMERA_SOURCE_TOPIC}" >&2
      echo "Set CAMERA_DETECTION_ENABLED=false to stream this topic without detection boxes." >&2
      exit 2
      ;;
  esac

  # A missing raw depth stream means the detector cannot produce range even
  # though the color/video path is healthy (e.g. a D455 MIPI/USB fault).
  if ! timeout 12 ros2 topic echo "${DETECTION_DEPTH_TOPIC}" --once --field header >/dev/null 2>&1; then
    echo "No raw depth frame received from ${DETECTION_DEPTH_TOPIC}." >&2
    echo "Power-cycle the D455 and check the USB cable/port; the depth sensor may have failed." >&2
    echo "Set CAMERA_DETECTION_ENABLED=false to stream video without detection boxes." >&2
    exit 5
  fi

  echo "Starting D455 obstacle detection..."
  D455_WS="${D455_WS}" \
    D455_MODEL="${CAMERA_DETECTION_MODEL}" \
    D455_MODEL_SHA256="${CAMERA_DETECTION_MODEL_SHA256}" \
    D455_SYNC_MODE="${CAMERA_DETECTION_SYNC_MODE}" \
    D455_SYNC_SLOP="${CAMERA_DETECTION_SYNC_SLOP}" \
    "${D455_WS}/.venv/bin/python" "${D455_DETECT_WRAPPER}" \
      --ros-args \
      -p imgsz:="${CAMERA_DETECTION_IMGSZ}" \
      -p device:="${CAMERA_DETECTION_DEVICE}" \
      -p confidence:="${CAMERA_DETECTION_CONFIDENCE}" \
      -r "/sensing/camera/camera0/color/image_raw:=${DETECTION_COLOR_TOPIC}" \
      -r "/sensing/camera/camera0/depth/image_rect_raw:=${DETECTION_DEPTH_TOPIC}" \
      -r "/sensing/camera/camera0/color/camera_info:=${DETECTION_COLOR_INFO_TOPIC}" \
      -r "/sensing/camera/camera0/depth/camera_info:=${DETECTION_DEPTH_INFO_TOPIC}" \
      -r "/sensing/camera/camera0/extrinsics/depth_to_color:=${DETECTION_EXTRINSICS_TOPIC}" &
  DETECTION_PID=$!

  # Keep the video path on the camera's native compressed stream. The detector
  # sends only its small JSON result through rosbridge; the browser draws boxes.
  echo "Detection boxes: /d455_perception/detections (JSON overlay in the browser)"
fi

if [[ ! -x "${MEDIAMTX_BIN}" ]]; then
  echo "MediaMTX binary not found: ${MEDIAMTX_BIN}" >&2
  exit 1
fi

echo "Checking camera frames on ${CAMERA_STREAM_TOPIC}..."
if [[ "${CAMERA_STREAM_IS_RAW}" != "true" ]] \
  && ! timeout 8 ros2 topic echo "${CAMERA_STREAM_TOPIC}" --once >/dev/null 2>&1; then
  echo "No camera frame received from ${CAMERA_STREAM_TOPIC}." >&2
  echo "Start or restart the single RealSense owner before starting the stream." >&2
  exit 4
fi
if [[ "${CAMERA_STREAM_IS_RAW}" == "true" ]]; then
  echo "The video pipe will stay subscribed while the CPU model produces detection overlays."
fi

"${MEDIAMTX_BIN}" "${MEDIAMTX_CONFIG}" &
MEDIAMTX_PID=$!
sleep 1
if ! kill -0 "${MEDIAMTX_PID}" 2>/dev/null; then
  wait "${MEDIAMTX_PID}"
fi

FFMPEG_GLOBAL=()
case "${CAMERA_STREAM_ENCODER}" in
  vaapi)
    FFMPEG_GLOBAL=(-vaapi_device /dev/dri/renderD128)
    FFMPEG_VIDEO=(
      -vf "fps=${CAMERA_STREAM_FPS},format=nv12,hwupload"
      -c:v h264_vaapi
      -profile:v constrained_baseline
      -b:v "${CAMERA_STREAM_BITRATE}"
      -maxrate "${CAMERA_STREAM_BITRATE}"
      -bufsize "${CAMERA_STREAM_BUFSIZE}"
      -g "${CAMERA_STREAM_FPS}"
      -bf 0
    )
    ;;
  libx264)
    FFMPEG_VIDEO=(
      -vf "fps=${CAMERA_STREAM_FPS}"
      -c:v libx264
      -preset ultrafast
      -tune zerolatency
      -profile:v baseline
      -b:v "${CAMERA_STREAM_BITRATE}"
      -maxrate "${CAMERA_STREAM_BITRATE}"
      -bufsize "${CAMERA_STREAM_BUFSIZE}"
      -g "${CAMERA_STREAM_FPS}"
      -bf 0
    )
    ;;
  *)
    echo "Unsupported CAMERA_STREAM_ENCODER: ${CAMERA_STREAM_ENCODER}" >&2
    echo "Use 'vaapi' or 'libx264'." >&2
    exit 2
    ;;
esac

echo "Publishing ${CAMERA_STREAM_TOPIC} as H.264 (${CAMERA_STREAM_ENCODER}, ${CAMERA_STREAM_BITRATE}, ${CAMERA_STREAM_FPS} fps)"
echo "WebRTC: http://10.245.188.140:8889/cam"
echo "HLS:    http://10.245.188.140:8888/cam"

JPEG_PIPE_PYTHON="python3"
JPEG_PIPE_ARGS=(--topic "${CAMERA_STREAM_TOPIC}")
FFMPEG_INPUT_ARGS=(
  -fflags nobuffer
  -flags low_delay
  -f mjpeg
  -framerate "${CAMERA_STREAM_INPUT_FPS}"
  -use_wallclock_as_timestamps 1
  -i pipe:0
)
if [[ "${CAMERA_STREAM_IS_RAW}" == "true" ]]; then
  JPEG_PIPE_PYTHON="${D455_WS}/.venv/bin/python"
  JPEG_PIPE_ARGS+=(
    --raw
    --jpeg-quality "${CAMERA_ANNOTATED_JPEG_QUALITY}"
    --repeat-fps "${CAMERA_STREAM_FPS}"
  )
  # The frame holder supplies a stable cadence, so sequential input timestamps
  # are more reliable than wall-clock MJPEG timestamps for the detection stream.
  FFMPEG_INPUT_ARGS=(
    -flags low_delay
    -f mjpeg
    -framerate "${CAMERA_STREAM_FPS}"
    -i pipe:0
  )
fi

"${JPEG_PIPE_PYTHON}" "${JPEG_PIPE}" "${JPEG_PIPE_ARGS[@]}" |
  ffmpeg "${FFMPEG_GLOBAL[@]}" \
    -hide_banner -loglevel warning \
    "${FFMPEG_INPUT_ARGS[@]}" \
    -an "${FFMPEG_VIDEO[@]}" -flush_packets 1 \
    -f rtsp -rtsp_transport tcp "${CAMERA_STREAM_RTSP_URL}"
