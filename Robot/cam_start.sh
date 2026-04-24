#!/usr/bin/env bash
set -euo pipefail

RTSP_PATH="${RTSP_PATH:-robot}"
RTSP_PORT="${RTSP_PORT:-8554}"
VIDEO_SIZE="${VIDEO_SIZE:-640x480}"
FPS="${FPS:-20}"
INPUT_FORMAT="${INPUT_FORMAT:-mjpeg}"
CAMERA_DEVICE="${CAMERA_DEVICE:-}"
FFMPEG_LOGLEVEL="${FFMPEG_LOGLEVEL:-warning}"
MEDIAMTX_CFG="${HOME}/mediamtx.yml"
MEDIAMTX_RUN_BIN="${HOME}/mediamtx_bin"

if ! command -v ffmpeg >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y ffmpeg
fi

find_mediamtx_bin() {
  for p in \
    "./mediamtx" \
    "${HOME}/mediamtx/mediamtx" \
    "${HOME}/mediamtx_bin" \
    "/usr/local/bin/mediamtx" \
    "/usr/bin/mediamtx"
  do
    if [ -x "$p" ]; then
      printf '%s\n' "$p"
      return 0
    fi
  done
  return 1
}

find_mediamtx_src() {
  for p in \
    "./mediamtx" \
    "${HOME}/mediamtx" \
    "${HOME}/bluenviron/mediamtx" \
    "${HOME}/Road-Follower-Robot/Robot/mediamtx" \
    "${HOME}/Road-Follower-Robot/mediamtx"
  do
    if [ -d "$p" ] && [ -f "$p/go.mod" ]; then
      printf '%s\n' "$p"
      return 0
    fi
  done
  return 1
}

MEDIAMTX_BIN=""
if MEDIAMTX_BIN="$(find_mediamtx_bin)"; then
  :
else
  SRC_DIR="$(find_mediamtx_src || true)"
  if [ -z "${SRC_DIR}" ]; then
    echo "mediamtx not found" >&2
    exit 1
  fi
  if ! command -v go >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y golang
  fi
  cd "${SRC_DIR}"
  go build -o "${MEDIAMTX_RUN_BIN}" ./cmd/mediamtx
  MEDIAMTX_BIN="${MEDIAMTX_RUN_BIN}"
fi

cat > "${MEDIAMTX_CFG}" <<EOF
logLevel: warn
rtspAddress: :${RTSP_PORT}
paths:
  ${RTSP_PATH}:
EOF

if [ -z "${CAMERA_DEVICE}" ]; then
  for d in /dev/video0 /dev/video2 /dev/video1 /dev/video3; do
    if [ -e "${d}" ]; then
      CAMERA_DEVICE="${d}"
      break
    fi
  done
fi

if [ -z "${CAMERA_DEVICE}" ]; then
  echo "no camera device found" >&2
  exit 1
fi

pkill -f "${MEDIAMTX_BIN}" >/dev/null 2>&1 || true
pkill -f "ffmpeg.*rtsp://127.0.0.1:${RTSP_PORT}/${RTSP_PATH}" >/dev/null 2>&1 || true
sleep 1

nohup "${MEDIAMTX_BIN}" "${MEDIAMTX_CFG}" >/tmp/mediamtx.log 2>&1 &
sleep 2

exec ffmpeg -hide_banner -loglevel "${FFMPEG_LOGLEVEL}" -f v4l2 -input_format "${INPUT_FORMAT}" -video_size "${VIDEO_SIZE}" -framerate "${FPS}" -i "${CAMERA_DEVICE}" -an -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p -g "${FPS}" -f rtsp "rtsp://127.0.0.1:${RTSP_PORT}/${RTSP_PATH}"
