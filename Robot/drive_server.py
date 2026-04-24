#!/usr/bin/env python3
import json
import importlib
import os
import signal
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

try:
    GPIO = importlib.import_module("RPi.GPIO")
except Exception as e:  # pragma: no cover
    GPIO = None
    GPIO_IMPORT_ERROR = e
else:
    GPIO_IMPORT_ERROR = None

HTTP_PORT = int(os.getenv("DRIVE_HTTP_PORT", "8090"))
PWM_FREQ_HZ = int(os.getenv("MOTOR_PWM_FREQ_HZ", "1000"))
GPIO_WARNINGS = os.getenv("GPIO_WARNINGS", "0") == "1"


def _env_int(names: tuple[str, ...], default: int) -> int:
    for name in names:
        raw = os.getenv(name)
        if raw is not None and raw != "":
            return int(raw)
    return default


def _env_bool(names: tuple[str, ...], default: bool = False) -> bool:
    for name in names:
        raw = os.getenv(name)
        if raw is not None and raw != "":
            return raw == "1"
    return default


@dataclass(frozen=True)
class MotorPins:
    in1: int
    in2: int
    pwm: int
    invert: bool = False


LEFT_MOTOR = MotorPins(
    # L298N channel A: IN1/IN2 + ENA(PWM)
    in1=_env_int(("LEFT_IN1_GPIO", "L298N_IN1_GPIO"), 5),
    in2=_env_int(("LEFT_IN2_GPIO", "L298N_IN2_GPIO"), 6),
    pwm=_env_int(("LEFT_PWM_GPIO", "L298N_ENA_GPIO"), 12),
    invert=_env_bool(("LEFT_INVERT", "L298N_LEFT_INVERT"), False),
)
RIGHT_MOTOR = MotorPins(
    # L298N channel B: IN3/IN4 + ENB(PWM)
    in1=_env_int(("RIGHT_IN1_GPIO", "L298N_IN3_GPIO"), 20),
    in2=_env_int(("RIGHT_IN2_GPIO", "L298N_IN4_GPIO"), 21),
    pwm=_env_int(("RIGHT_PWM_GPIO", "L298N_ENB_GPIO"), 13),
    invert=_env_bool(("RIGHT_INVERT", "L298N_RIGHT_INVERT"), False),
)

last_cmd = {"l": 0, "r": 0, "ts": 0.0}
last_cmd_lock = threading.Lock()

drive = None
httpd = None


def clamp(v, lo, hi):
    return max(lo, min(hi, int(v)))


class DifferentialDriveGPIO:
    def __init__(self, left: MotorPins, right: MotorPins, pwm_freq_hz: int):
        self.left = left
        self.right = right
        self.pwm_freq_hz = pwm_freq_hz
        self.lock = threading.Lock()

        GPIO.setwarnings(GPIO_WARNINGS)
        GPIO.setmode(GPIO.BCM)

        for pin in (left.in1, left.in2, left.pwm, right.in1, right.in2, right.pwm):
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        self.left_pwm = GPIO.PWM(left.pwm, pwm_freq_hz)
        self.right_pwm = GPIO.PWM(right.pwm, pwm_freq_hz)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

    def _apply_motor(self, cfg: MotorPins, pwm, speed: int):
        speed = clamp(speed, -255, 255)
        if cfg.invert:
            speed = -speed

        duty = (abs(speed) / 255.0) * 100.0

        if speed > 0:
            GPIO.output(cfg.in1, GPIO.HIGH)
            GPIO.output(cfg.in2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(cfg.in1, GPIO.LOW)
            GPIO.output(cfg.in2, GPIO.HIGH)
        else:
            GPIO.output(cfg.in1, GPIO.LOW)
            GPIO.output(cfg.in2, GPIO.LOW)

        pwm.ChangeDutyCycle(duty)

    def set_speed(self, left_speed: int, right_speed: int):
        with self.lock:
            self._apply_motor(self.left, self.left_pwm, left_speed)
            self._apply_motor(self.right, self.right_pwm, right_speed)

    def stop(self):
        self.set_speed(0, 0)

    def cleanup(self):
        with self.lock:
            try:
                self.left_pwm.ChangeDutyCycle(0)
                self.right_pwm.ChangeDutyCycle(0)
            except Exception:
                pass
            try:
                self.left_pwm.stop()
                self.right_pwm.stop()
            except Exception:
                pass

            GPIO.output(self.left.in1, GPIO.LOW)
            GPIO.output(self.left.in2, GPIO.LOW)
            GPIO.output(self.right.in1, GPIO.LOW)
            GPIO.output(self.right.in2, GPIO.LOW)
            GPIO.cleanup([
                self.left.in1,
                self.left.in2,
                self.left.pwm,
                self.right.in1,
                self.right.in2,
                self.right.pwm,
            ])


def send_drive(left, right):
    global last_cmd

    left = clamp(left, -255, 255)
    right = clamp(right, -255, 255)

    drive.set_speed(left, right)
    print(f"[GPIO] set L={left} R={right}")

    with last_cmd_lock:
        last_cmd = {"l": left, "r": right, "ts": time.time()}


def snapshot_last_cmd():
    with last_cmd_lock:
        return dict(last_cmd)


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
                "backend": "rpi_gpio",
                "http_port": HTTP_PORT,
                "pwm_freq_hz": PWM_FREQ_HZ,
                "left_pins": {
                    "in1": LEFT_MOTOR.in1,
                    "in2": LEFT_MOTOR.in2,
                    "pwm": LEFT_MOTOR.pwm,
                    "invert": LEFT_MOTOR.invert,
                },
                "right_pins": {
                    "in1": RIGHT_MOTOR.in1,
                    "in2": RIGHT_MOTOR.in2,
                    "pwm": RIGHT_MOTOR.pwm,
                    "invert": RIGHT_MOTOR.invert,
                },
                "last_cmd": snapshot_last_cmd(),
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


def stop_everything():
    if drive is not None:
        try:
            drive.stop()
        except Exception:
            pass
        try:
            drive.cleanup()
        except Exception:
            pass


def handle_signal(sig, frame):
    print(f"[SYS] signal {sig}, shutting down")
    if httpd is not None:
        httpd.shutdown()


def main():
    global drive, httpd

    if GPIO is None:
        raise RuntimeError(
            "RPi.GPIO is not available. Install python3-rpi.gpio on Raspberry Pi OS. "
            f"Import error: {GPIO_IMPORT_ERROR}"
        )

    drive = DifferentialDriveGPIO(LEFT_MOTOR, RIGHT_MOTOR, PWM_FREQ_HZ)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    httpd = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"[HTTP] listening on {HTTP_PORT}")
    print(f"Health: http://0.0.0.0:{HTTP_PORT}/health")
    print(f"Drive : http://0.0.0.0:{HTTP_PORT}/drive?l=80&r=80")
    print(f"Stop  : http://0.0.0.0:{HTTP_PORT}/stop")

    try:
        httpd.serve_forever()
    finally:
        stop_everything()


if __name__ == "__main__":
    main()