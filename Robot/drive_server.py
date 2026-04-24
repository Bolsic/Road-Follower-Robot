#!/usr/bin/env python3
import json
import time
import serial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

SERIAL_PORT = "/dev/ttyHS1"
BAUD = 115200
HTTP_PORT = 8090

ser = None
last_cmd = {"l": 0, "r": 0, "ts": 0.0}


def clamp(v, lo, hi):
    return max(lo, min(hi, int(v)))


def open_serial():
    global ser
    print(f"[SERIAL] opening {SERIAL_PORT} @ {BAUD}")
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print("[SERIAL] opened")


def send_drive(left, right):
    global last_cmd
    left = clamp(left, -255, 255)
    right = clamp(right, -255, 255)

    msg = f"L{left} R{right}\n"
    print(f"[TX] {msg.strip()}")

    ser.write(msg.encode("utf-8"))
    ser.flush()

    last_cmd = {"l": left, "r": right, "ts": time.time()}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        print(f"[HTTP] {self.path}")

        if path == "/health":
            self._json(200, {
                "ok": True,
                "serial_port": SERIAL_PORT,
                "last_cmd": last_cmd,
            })
            return

        if path == "/stop":
            try:
                send_drive(0, 0)
                self._json(200, {"ok": True, "stopped": True})
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)})
            return

        if path == "/drive":
            try:
                left = clamp(qs.get("l", ["0"])[0], -255, 255)
                right = clamp(qs.get("r", ["0"])[0], -255, 255)
                send_drive(left, right)
                self._json(200, {"ok": True, "left": left, "right": right})
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)})
            return

        self._json(404, {"ok": False, "error": "not found"})

    def _json(self, code, obj):
        raw = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(raw)

    def log_message(self, format, *args):
        return


def main():
    open_serial()
    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[HTTP] listening on {HTTP_PORT}")
    print(f"Health: http://0.0.0.0:{HTTP_PORT}/health")
    print(f"Drive : http://0.0.0.0:{HTTP_PORT}/drive?l=80&r=80")
    print(f"Stop  : http://0.0.0.0:{HTTP_PORT}/stop")
    server.serve_forever()


if __name__ == "__main__":
    main()