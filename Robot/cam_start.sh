#!/usr/bin/env bash
set -e

cat > ~/mediamtx.yml <<EOF
logLevel: warn

rtspAddress: :8554

webrtc: yes
webrtcAddress: :8889

paths:
  robot:
EOF

~/mediamtx/mediamtx ~/mediamtx.yml &
sleep 2

ffmpeg -f v4l2 -input_format mjpeg -video_size 640x480 -framerate 20 -i /dev/video0 -c:v libx264 -preset ultrafast -tune zerolatency -f rtsp rtsp://127.0.0.1:8554/robot
