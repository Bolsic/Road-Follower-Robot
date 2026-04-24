#!/usr/bin/env bash

sleep 5

~/mediamtx/mediamtx ~/mediamtx.yml &

sleep 3

ffmpeg -f v4l2 -input_format mjpeg -video_size 640x480 -framerate 20 -i /dev/video2 -c:v libx264 -preset ultrafast -tune zerolatency -f rtsp rtsp://127.0.0.1:8554/robot
