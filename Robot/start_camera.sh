#!/usr/bin/env bash
set -euo pipefail

MEDIAMTX_BIN="${MEDIAMTX_BIN:-$HOME/mediamtx/mediamtx}"
MEDIAMTX_CFG="${MEDIAMTX_CFG:-$HOME/mediamtx.yml}"
RTSP_PATH="${RTSP_PATH:-robot}"
RTSP_PORT="${RTSP_PORT:-8554}"
WEBRTC_PORT="${WEBRTC_PORT:-8889}"
VIDEO_SIZE="${VIDEO_SIZE:-640x480}"
FPS="${FPS:-20}"
INPUT_FORMAT="${INPUT_FORMAT:-mjpeg}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video2}"
FFMPEG_LOGLEVEL="${FFMPEG_LOGLEVEL:-warning}"

if [ ! -x "${MEDIAMTX_BIN}" ]; then
	echo "mediamtx binary not found: ${MEDIAMTX_BIN}" >&2
	exit 1
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
	echo "ffmpeg not found in PATH" >&2
	exit 1
fi

if [ ! -e "${CAMERA_DEVICE}" ]; then
	for d in /dev/video0 /dev/video2 /dev/video1 /dev/video3; do
		if [ -e "${d}" ]; then
			CAMERA_DEVICE="${d}"
			break
		fi
	done
fi

if [ ! -e "${CAMERA_DEVICE}" ]; then
	echo "no camera device found" >&2
	exit 1
fi

cat > "${MEDIAMTX_CFG}" <<EOF
logLevel: warn
rtspAddress: :${RTSP_PORT}
webrtc: yes
webrtcAddress: :${WEBRTC_PORT}
paths:
	${RTSP_PATH}:
EOF

pkill -f "${MEDIAMTX_BIN}" >/dev/null 2>&1 || true
pkill -f "ffmpeg.*rtsp://127.0.0.1:${RTSP_PORT}/${RTSP_PATH}" >/dev/null 2>&1 || true
sleep 1

nohup "${MEDIAMTX_BIN}" "${MEDIAMTX_CFG}" >/tmp/mediamtx.log 2>&1 &
sleep 2

echo "MediaMTX config: ${MEDIAMTX_CFG}"
echo "Camera device  : ${CAMERA_DEVICE}"
echo "RTSP publish   : rtsp://127.0.0.1:${RTSP_PORT}/${RTSP_PATH}"
echo "WHEP read URL  : http://<robot-ip>:${WEBRTC_PORT}/${RTSP_PATH}/whep"

exec ffmpeg -hide_banner -loglevel "${FFMPEG_LOGLEVEL}" -f v4l2 -input_format "${INPUT_FORMAT}" -video_size "${VIDEO_SIZE}" -framerate "${FPS}" -i "${CAMERA_DEVICE}" -an -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p -g "${FPS}" -f rtsp "rtsp://127.0.0.1:${RTSP_PORT}/${RTSP_PATH}"
