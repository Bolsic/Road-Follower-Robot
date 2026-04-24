#!/usr/bin/env bash
set -euo pipefail

MEDIAMTX_VERSION="${MEDIAMTX_VERSION:-1.8.4}"
MEDIAMTX_DIR="${HOME}/mediamtx"
MEDIAMTX_BIN="${MEDIAMTX_DIR}/mediamtx"
MEDIAMTX_CFG="${MEDIAMTX_DIR}/mediamtx.yml"
RTSP_PATH="${RTSP_PATH:-robot}"
RTSP_PORT="${RTSP_PORT:-8554}"
VIDEO_SIZE="${VIDEO_SIZE:-640x480}"
FPS="${FPS:-20}"
INPUT_FORMAT="${INPUT_FORMAT:-mjpeg}"
CAMERA_DEVICE="${CAMERA_DEVICE:-}"
FFMPEG_LOGLEVEL="${FFMPEG_LOGLEVEL:-warning}"

mkdir -p "${MEDIAMTX_DIR}"

if ! command -v ffmpeg >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y ffmpeg curl tar
fi

if [ ! -x "${MEDIAMTX_BIN}" ]; then
  ARCH="$(uname -m)"
  case "${ARCH}" in
    aarch64|arm64) PKG_ARCH="arm64" ;;
    armv7l|armv6l|armhf) PKG_ARCH="armv7" ;;
    x86_64|amd64) PKG_ARCH="amd64" ;;
    *) echo "unsupported arch: ${ARCH}" >&2; exit 1 ;;
  esac
  TMPDIR="$(mktemp -d)"
  curl -L "https://github.com/bluenviron/mediamtx/releases/download/v${MEDIAMTX_VERSION}/mediamtx_v${MEDIAMTX_VERSION}_linux_${PKG_ARCH}.tar.gz" -o "${TMPDIR}/mediamtx.tar.gz"
  tar -xzf "${TMPDIR}/mediamtx.tar.gz" -C "${TMPDIR}"
  install -m 755 "${TMPDIR}/mediamtx" "${MEDIAMTX_BIN}"
  rm -rf "${TMPDIR}"
fi

cat > "${MEDIAMTX_CFG}" <<EOF2
logLevel: warn
rtspAddress: :${RTSP_PORT}
paths:
  ${RTSP_PATH}:
EOF2

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
