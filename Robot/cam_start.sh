#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$APP_DIR/logs"
CAM_SCRIPT="$APP_DIR/cam_server.py"
CAM_HOST="${CAM_HOST:-0.0.0.0}"

mkdir -p "$LOG_DIR"

echo "[INFO] App dir: $APP_DIR"
echo "[INFO] Log dir: $LOG_DIR"

# Stop old camera instance if running
pkill -f "$CAM_SCRIPT" 2>/dev/null || true

echo "[INFO] Starting camera stream..."
nohup python3 "$CAM_SCRIPT" > "$LOG_DIR/camera.log" 2>&1 &
CAM_PID=$!
echo "$CAM_PID" > "$LOG_DIR/camera.pid"

sleep 2

echo
echo "================ CAMERA SERVICE STARTED ================"
echo "Camera PID : $CAM_PID"
echo
echo "Health URLs:"
echo "  Camera health : http://$CAM_HOST:8081/health"
echo "  Snapshot      : http://$CAM_HOST:8081/snapshot.jpg"
echo "  MJPEG stream  : http://$CAM_HOST:8081/stream.mjpg"
echo
echo "Log:"
echo "  $LOG_DIR/camera.log"
echo "========================================================"