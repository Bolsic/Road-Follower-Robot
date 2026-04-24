#!/usr/bin/env python3
import cv2
import threading
import time
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WIDTH = 320
HEIGHT = 240
FPS = 6
JPEG_QUALITY = 45
STREAM_PORT = 8081

# Probe sources in order
CAMERA_SOURCES = [
    "/dev/video2",
    "/dev/video3",
    2,
    3,
]

latest_jpeg = None
latest_lock = threading.Lock()

status = {
    "opened_index": None,
    "frame_ok": False,
    "message": "starting",
    "width": WIDTH,
    "height": HEIGHT,
    "fps": FPS,
}

running = True


def open_working_camera():
    for src in CAMERA_SOURCES:
        print(f"[CAM] Trying source: {src}")
        cap = cv2.VideoCapture(src)

        if not cap.isOpened():
            print(f"[CAM] open failed: {src}")
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FPS)

        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except Exception:
            pass

        # Warmup
        got_frame = False
        frame = None
        for i in range(20):
            ok, test = cap.read()
            if ok and test is not None and test.size > 0:
                got_frame = True
                frame = test
                print(f"[CAM] source {src} produced frame on try {i}")
                break
            time.sleep(0.1)

        if got_frame:
            return cap, src, frame

        print(f"[CAM] no valid frames from: {src}")
        cap.release()

    return None, None, None


def camera_loop():
    global latest_jpeg, status, running

    cap, src, first_frame = open_working_camera()

    if cap is None:
        status["message"] = "no working camera source found"
        print("ERROR:", status["message"])
        return

    status["opened_index"] = str(src)
    status["frame_ok"] = True
    status["message"] = f"camera working on source {src}"
    print(status["message"])

    ok, jpg = cv2.imencode(
        ".jpg",
        first_frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    )
    if ok:
        with latest_lock:
            latest_jpeg = jpg.tobytes()

    while running:
        ok, frame = cap.read()

        if not ok or frame is None or frame.size == 0:
            status["frame_ok"] = False
            time.sleep(0.05)
            continue

        ok, jpg = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        )

        if not ok:
            status["frame_ok"] = False
            time.sleep(0.02)
            continue

        with latest_lock:
            latest_jpeg = jpg.tobytes()

        status["frame_ok"] = True
        time.sleep(1.0 / FPS)

    cap.release()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def do_GET(self):
        if self.path == "/health":
            data = json.dumps(status).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(data)
            self.wfile.flush()
            return

        if self.path == "/snapshot.jpg":
            with latest_lock:
                jpg = latest_jpeg

            if jpg is None:
                self.send_response(503)
                self.end_headers()
                return

            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(jpg)
            self.wfile.flush()
            return

        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            try:
                while True:
                    with latest_lock:
                        jpg = latest_jpeg

                    if jpg is None:
                        time.sleep(0.05)
                        continue

                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("utf-8"))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()

                    time.sleep(1.0 / FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):
        return


def http_server_loop():
    server = ThreadingHTTPServer(("0.0.0.0", STREAM_PORT), Handler)
    print(f"Health:       http://0.0.0.0:{STREAM_PORT}/health")
    print(f"Snapshot:     http://0.0.0.0:{STREAM_PORT}/snapshot.jpg")
    print(f"MJPEG stream: http://0.0.0.0:{STREAM_PORT}/stream.mjpg")
    server.serve_forever()


threading.Thread(target=camera_loop, daemon=True).start()
threading.Thread(target=http_server_loop, daemon=True).start()

print("Robot camera stream app starting...")

while True:
    time.sleep(1)