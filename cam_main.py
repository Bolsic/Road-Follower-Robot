#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import signal
import threading
import time
from dataclasses import dataclass
from typing import Optional

import aiohttp
import av
import cv2
import numpy as np
import requests
from aiortc import RTCPeerConnection, RTCSessionDescription


@dataclass
class Config:
    whep_url: str = "http://10.255.7.42:8889/robot/whep"
    drive_base: str = "http://10.255.7.42:8090"

    width: int = 320
    height: int = 240
    flip: bool = False

    # ROI
    roi_y_start_frac: float = 0.48
    roi_y_end_frac: float = 0.95

    # bottom -> mid -> upper
    scan_row_fracs: tuple[float, float, float] = (0.90, 0.72, 0.52)

    # Otsu preprocess
    blur_kernel: int = 5
    morph_kernel: int = 3
    min_black_run_width: int = 5

    # Value threshold for line detection (only dark pixels can be lines)
    # (we use value-only masking; saturation-based masking disabled)
    value_threshold: int = 60  # regions with value > this are masked out (only dark pixels remain)

    # Lane width estimate for one-side fallback
    lane_width_est_px: float = 220.0
    lane_width_update_alpha: float = 0.18
    min_lane_width_px: float = 110.0
    max_lane_width_px: float = 290.0
    edge_to_center_factor: float = 0.50

    # Control
    base_speed: int = 54
    min_speed: int = 50
    max_speed: int = 60

    kp: float = 0.6
    ki: float = 0.01
    kd: float = 0.5
    deadband_px: int = 8
    max_turn: int = 32

    # Curve anticipation
    preview_gain: float = 1.80
    max_preview_px: float = 45.0

    # Slow in corners
    slow_gain: float = 0.30
    max_slowdown: int = 22

    # Lost handling
    lost_turn_speed: int = 40
    lost_turn_boost: int = 14
    lost_hold_seconds: float = 0.7

    # HTTP
    drive_timeout_s: float = 0.20
    stop_timeout_s: float = 0.20
    drive_resend_interval_s: float = 0.08

    # UI
    show_debug: bool = True
    window_name: str = "Lane Follower Preview"
    binary_window_name: str = "Binary"


CFG = Config()


def clamp(v: float, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(round(v))))


class RobotClient:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.session = requests.Session()

    def drive(self, left: int, right: int) -> bool:
        left = clamp(left, -255, 255)
        right = clamp(right, -255, 255)
        try:
            r = self.session.get(
                f"{self.cfg.drive_base}/drive",
                params={"l": left, "r": right},
                timeout=self.cfg.drive_timeout_s,
            )
            return r.ok
        except requests.RequestException:
            return False

    def stop(self) -> bool:
        try:
            r = self.session.get(
                f"{self.cfg.drive_base}/stop",
                timeout=self.cfg.stop_timeout_s,
            )
            return r.ok
        except requests.RequestException:
            return False


class WHEPReceiver:
    def __init__(self, whep_url: str, width: int, height: int, flip: bool = False) -> None:
        self.whep_url = whep_url
        self.width = width
        self.height = height
        self.flip = flip

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._status = "stopped"
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()

    def get_status(self) -> str:
        with self._lock:
            return self._status

    def _set_status(self, txt: str) -> None:
        with self._lock:
            self._status = txt

    def _store_frame(self, img: np.ndarray) -> None:
        img = cv2.resize(img, (self.width, self.height), interpolation=cv2.INTER_AREA)
        if self.flip:
            img = cv2.flip(img, -1)
        with self._lock:
            self._latest_frame = img

    def _thread_main(self) -> None:
        asyncio.run(self._run_forever())

    async def _wait_ice_complete(self, pc: RTCPeerConnection, timeout: float = 5.0) -> None:
        if pc.iceGatheringState == "complete":
            return

        done = asyncio.Event()

        @pc.on("icegatheringstatechange")
        async def _on_change() -> None:
            if pc.iceGatheringState == "complete":
                done.set()

        try:
            await asyncio.wait_for(done.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            pass

    async def _create_session(self, session: aiohttp.ClientSession, pc: RTCPeerConnection) -> str:
        pc.addTransceiver("video", direction="recvonly")

        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await self._wait_ice_complete(pc)

        assert pc.localDescription is not None

        headers = {
            "Content-Type": "application/sdp",
            "Accept": "application/sdp",
        }

        async with session.post(
            self.whep_url,
            data=pc.localDescription.sdp.encode("utf-8"),
            headers=headers,
        ) as resp:
            answer_sdp = await resp.text()
            if resp.status not in (200, 201):
                raise RuntimeError(f"WHEP POST failed: {resp.status} {answer_sdp}")

            location = resp.headers.get("Location", "")
            if not location:
                session_url = self.whep_url
            elif location.startswith("http://") or location.startswith("https://"):
                session_url = location
            else:
                base = self.whep_url.rsplit("/", 1)[0]
                session_url = base + "/" + location.lstrip("/")

        await pc.setRemoteDescription(RTCSessionDescription(sdp=answer_sdp, type="answer"))
        return session_url

    async def _delete_session(self, session: aiohttp.ClientSession, session_url: str) -> None:
        try:
            async with session.delete(session_url):
                pass
        except Exception:
            pass

    async def _consume_track(self, track) -> None:
        while self._running:
            frame: av.VideoFrame = await track.recv()
            img = frame.to_ndarray(format="bgr24")
            self._store_frame(img)

    async def _run_once(self) -> None:
        pc = RTCPeerConnection()
        tasks: list[asyncio.Task] = []

        @pc.on("track")
        def on_track(track) -> None:
            if track.kind == "video":
                tasks.append(asyncio.create_task(self._consume_track(track)))

        async with aiohttp.ClientSession() as session:
            self._set_status("connecting")
            session_url = await self._create_session(session, pc)
            self._set_status("connected")

            try:
                while self._running:
                    if pc.connectionState in ("failed", "closed"):
                        raise RuntimeError(f"Peer connection state: {pc.connectionState}")
                    await asyncio.sleep(0.05)
            finally:
                for t in tasks:
                    t.cancel()
                await self._delete_session(session, session_url)
                await pc.close()

    async def _run_forever(self) -> None:
        while self._running:
            try:
                await self._run_once()
            except Exception as e:
                self._set_status(f"retrying: {type(e).__name__}")
                if not self._running:
                    break
                await asyncio.sleep(1.0)
        self._set_status("stopped")


class LaneFollower:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.last_seen_time = 0.0
        self.last_turn_sign = 0
        self.last_target_x: Optional[float] = None
        self.lane_width_est_px = cfg.lane_width_est_px

        # PID state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = 0.0
        self.filtered_deriv = 0.0

    def _preprocess(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray, int, int, np.ndarray]:
        h, _ = frame.shape[:2]

        y1 = int(h * self.cfg.roi_y_start_frac)
        y2 = int(h * self.cfg.roi_y_end_frac)
        y1 = max(0, min(h - 1, y1))
        y2 = max(y1 + 1, min(h, y2))

        roi = frame[y1:y2].copy()
        
        # Convert to HSV and extract saturation and value channels
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        
        # Create masks:
        # - Low value: dark pixels only (black lines, not bright road)
        # We no longer use saturation masking; only value thresholding is applied
        val_mask = value < self.cfg.value_threshold
        combined_mask = val_mask

        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # IMPORTANT: set masked-out pixels to bright (255) so they are NOT
        # considered dark by Otsu/thresholding. Previously we zeroed masked
        # areas which caused them to be detected as dark lines.
        gray_masked = gray.copy()
        gray_masked[~combined_mask] = 255
        gray = gray_masked

        k = self.cfg.blur_kernel
        if k % 2 == 0:
            k += 1
        blur = cv2.GaussianBlur(gray, (k, k), 0)

        # black on white, detected in masked region only
        _, binary = cv2.threshold(
            blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        mk = self.cfg.morph_kernel
        kernel = np.ones((mk, mk), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        return roi, binary, y1, y2, combined_mask

    def _find_runs(self, row: np.ndarray) -> list[tuple[int, int]]:
        xs = np.where(row > 0)[0]
        if xs.size == 0:
            return []

        runs: list[tuple[int, int]] = []
        start = int(xs[0])
        prev = int(xs[0])

        for x in xs[1:]:
            x = int(x)
            if x != prev + 1:
                if prev - start + 1 >= self.cfg.min_black_run_width:
                    runs.append((start, prev))
                start = x
            prev = x

        if prev - start + 1 >= self.cfg.min_black_run_width:
            runs.append((start, prev))

        return runs

    def _pick_side_runs(
        self,
        runs: list[tuple[int, int]],
        width: int,
    ) -> tuple[Optional[float], Optional[float]]:
        img_center = width / 2.0

        left_x: Optional[float] = None
        right_x: Optional[float] = None
        best_left_score = -1e9
        best_right_score = -1e9

        for r1, r2 in runs:
            run_w = r2 - r1 + 1
            cx = 0.5 * (r1 + r2)

            if cx < img_center:
                score = 3.0 * run_w - 0.7 * abs(cx - img_center)
                if score > best_left_score:
                    best_left_score = score
                    left_x = cx

            if cx > img_center:
                score = 3.0 * run_w - 0.7 * abs(cx - img_center)
                if score > best_right_score:
                    best_right_score = score
                    right_x = cx

        return left_x, right_x

    def process(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        roi, binary, y1, y2, combined_mask = self._preprocess(frame)
        roi_h, roi_w = binary.shape[:2]
        
        # Create visualization: convert binary to BGR and color masked-out regions purple
        binary_viz = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        # Mask-out regions (where combined_mask is False) are shown in purple (B=255, G=0, R=255)
        masked_out = ~combined_mask
        binary_viz[masked_out] = (255, 0, 255)  # Purple in BGR

        img_center = roi_w / 2.0

        row_indices = []
        for frac in self.cfg.scan_row_fracs:
            y = int(roi_h * frac)
            y = max(0, min(roi_h - 1, y))
            row_indices.append(y)

        row_targets: list[float] = []
        left_pts: list[tuple[int, int]] = []
        right_pts: list[tuple[int, int]] = []
        side_mode = "none"

        # First pass: detect candidate left/right x positions per scan row
        rows_info: list[dict] = []
        for ry in row_indices:
            runs = self._find_runs(binary[ry])
            left_x, right_x = self._pick_side_runs(runs, roi_w)
            rows_info.append({"ry": ry, "left_x": left_x, "right_x": right_x})
            if left_x is not None:
                left_pts.append((int(left_x), int(ry)))
            if right_x is not None:
                right_pts.append((int(right_x), int(ry)))

        # If we have any detected points, pick the one closest to bottom-right
        # among the three scan points and use its connected component to
        # reclassify all points that belong to the same white area as "right".
        all_pts = left_pts + right_pts
        if len(all_pts) > 0:
            br_x, br_y = roi_w - 1, roi_h - 1
            dists = [((br_x - x) ** 2 + (br_y - y) ** 2, x, y) for (x, y) in all_pts]
            dists.sort()
            _, seed_x, seed_y = dists[0]

            # connected components on binary (white=255)
            num_labels, labels = cv2.connectedComponents(binary)
            if 0 <= seed_x < roi_w and 0 <= seed_y < roi_h:
                seed_label = labels[int(seed_y), int(seed_x)]
                if seed_label != 0:
                    mask = labels == seed_label
                    # reassign any left_x that lies within mask to right_x
                    for info in rows_info:
                        lx = info["left_x"]
                        if lx is not None:
                            x = int(round(lx))
                            y = int(info["ry"])
                            if 0 <= x < roi_w and mask[y, x]:
                                info["right_x"] = lx
                                info["left_x"] = None

                    # rebuild point lists
                    left_pts = []
                    right_pts = []
                    for info in rows_info:
                        if info["left_x"] is not None:
                            left_pts.append((int(info["left_x"]), int(info["ry"])))
                        if info["right_x"] is not None:
                            right_pts.append((int(info["right_x"]), int(info["ry"])))

        # Second pass: compute row targets (update lane width estimate sequentially)
        for info in rows_info:
            left_x = info["left_x"]
            right_x = info["right_x"]
            row_target: Optional[float] = None

            if left_x is not None and right_x is not None and right_x > left_x:
                lane_width = right_x - left_x
                if self.cfg.min_lane_width_px <= lane_width <= self.cfg.max_lane_width_px:
                    self.lane_width_est_px = (
                        (1.0 - self.cfg.lane_width_update_alpha) * self.lane_width_est_px
                        + self.cfg.lane_width_update_alpha * lane_width
                    )
                row_target = 0.5 * (left_x + right_x)
                side_mode = "both"

            elif left_x is not None:
                row_target = left_x + self.cfg.edge_to_center_factor * self.lane_width_est_px
                side_mode = "left_only"

            elif right_x is not None:
                row_target = right_x - self.cfg.edge_to_center_factor * self.lane_width_est_px
                side_mode = "right_only"

            if row_target is not None:
                row_target = float(np.clip(row_target, 0, roi_w - 1))
                row_targets.append(row_target)

        found = len(row_targets) >= 1

        error = 0.0
        preview = 0.0
        action = "lost_stop"
        target_x: Optional[float] = None
        left = 0
        right = 0

        if found:
            # Weighted target: bottom row matters most
            if len(row_targets) >= 3:
                bottom_x = row_targets[0]
                mid_x = row_targets[1]
                top_x = row_targets[2]

                # curve anticipation: if upper rows shift left/right compared to bottom,
                # turn early into the curve
                preview = np.clip(top_x - bottom_x, -self.cfg.max_preview_px, self.cfg.max_preview_px)
                target_x = 0.62 * bottom_x + 0.28 * mid_x + 0.10 * top_x + self.cfg.preview_gain * preview

            elif len(row_targets) == 2:
                bottom_x = row_targets[0]
                mid_x = row_targets[1]
                preview = np.clip(mid_x - bottom_x, -self.cfg.max_preview_px, self.cfg.max_preview_px)
                target_x = 0.72 * bottom_x + 0.28 * mid_x + self.cfg.preview_gain * preview

            else:
                target_x = row_targets[0]

            target_x = float(np.clip(target_x, 0, roi_w - 1))
            self.last_target_x = target_x

            error = target_x - img_center
            if abs(error) < self.cfg.deadband_px:
                error = 0.0

            # PID control
            current_time = time.time()
            dt = current_time - self.prev_time if self.prev_time > 0 else 0.01
            self.integral += error * dt
            self.integral = np.clip(self.integral, -50.0, 50.0)  # anti-windup
            deriv = (error - self.prev_error) / dt if dt > 0 else 0.0
            alpha = 0.1  # low-pass filter for derivative
            self.filtered_deriv = alpha * deriv + (1 - alpha) * self.filtered_deriv
            turn = self.cfg.kp * error + self.cfg.ki * self.integral + self.cfg.kd * self.filtered_deriv
            turn = clamp(turn, -self.cfg.max_turn, self.cfg.max_turn)

            self.prev_error = error
            self.prev_time = current_time

            self.last_seen_time = current_time
            self.last_turn_sign = -1 if error < 0 else (1 if error > 0 else 0)
            action = f"track_{side_mode}"
            slowdown = min(self.cfg.max_slowdown, int(abs(error) * self.cfg.slow_gain))
            speed = clamp(self.cfg.base_speed - slowdown, self.cfg.min_speed, self.cfg.max_speed)

            left = clamp(speed + turn, -255, 255)
            right = clamp(speed - turn, -255, 255)

        else:
            self.integral = 0.0  # reset integral when lanes lost
            if (time.time() - self.last_seen_time) < self.cfg.lost_hold_seconds:
                if self.last_turn_sign < 0:
                    action = "search_left"
                    left = self.cfg.lost_turn_speed - self.cfg.lost_turn_boost
                    right = self.cfg.lost_turn_speed + self.cfg.lost_turn_boost
                elif self.last_turn_sign > 0:
                    action = "search_right"
                    left = self.cfg.lost_turn_speed + self.cfg.lost_turn_boost
                    right = self.cfg.lost_turn_speed - self.cfg.lost_turn_boost

        debug = frame.copy()

        cv2.rectangle(debug, (0, y1), (w - 1, y2), (255, 200, 0), 2)
        cv2.line(debug, (w // 2, 0), (w // 2, h - 1), (255, 0, 0), 1)

        for ry in row_indices:
            cv2.line(debug, (0, y1 + ry), (w - 1, y1 + ry), (70, 70, 70), 1)

        for x, ry in left_pts:
            cv2.circle(debug, (x, y1 + ry), 4, (0, 255, 255), -1)

        for x, ry in right_pts:
            cv2.circle(debug, (x, y1 + ry), 4, (255, 255, 0), -1)

        if target_x is not None:
            cv2.line(debug, (int(target_x), y1), (int(target_x), y2), (0, 0, 255), 2)
            cv2.circle(debug, (int(target_x), y1 + 10), 6, (0, 0, 255), -1)

        cv2.putText(
            debug,
            f"action={action} err={error:.1f} int={self.integral:.1f} deriv={self.filtered_deriv:.1f} L={left} R={right}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        return left, right, debug, binary_viz, action, found


def main() -> int:
    robot = RobotClient(CFG)
    receiver = WHEPReceiver(
        whep_url=CFG.whep_url,
        width=CFG.width,
        height=CFG.height,
        flip=CFG.flip,
    )
    follower = LaneFollower(CFG)

    running = True
    armed = False
    following = False

    last_sent_left = 0
    last_sent_right = 0
    last_send_time = 0.0

    def _handle_signal(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    receiver.start()

    print("Controls:")
    print("  a = arm/disarm")
    print("  f = follow on/off")
    print("  s = emergency stop")
    print("  q = quit")

    try:
        while running:
            frame = receiver.get_frame()

            if frame is None:
                blank = np.zeros((CFG.height, CFG.width, 3), dtype=np.uint8)
                cv2.putText(
                    blank,
                    f"No frame ({receiver.get_status()})",
                    (20, CFG.height // 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

                if CFG.show_debug:
                    cv2.imshow(CFG.window_name, blank)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    running = False
                elif key == ord("a"):
                    armed = not armed
                    if not armed:
                        robot.stop()
                        last_sent_left = 0
                        last_sent_right = 0
                elif key == ord("f"):
                    following = not following
                elif key == ord("s"):
                    armed = False
                    following = False
                    robot.stop()
                    last_sent_left = 0
                    last_sent_right = 0

                time.sleep(0.02)
                continue

            left, right, debug, binary, action, found = follower.process(frame)

            if not armed or not following:
                send_left = 0
                send_right = 0
            else:
                send_left = left
                send_right = right

            now = time.time()

            if not armed:
                if last_sent_left != 0 or last_sent_right != 0:
                    robot.stop()
                    last_sent_left = 0
                    last_sent_right = 0
                    last_send_time = now
            else:
                must_send = (
                    send_left != last_sent_left
                    or send_right != last_sent_right
                    or (now - last_send_time) >= CFG.drive_resend_interval_s
                )

                if must_send:
                    if send_left == 0 and send_right == 0:
                        robot.stop()
                    else:
                        robot.drive(send_left, send_right)
                    last_sent_left = send_left
                    last_sent_right = send_right
                    last_send_time = now

            cv2.putText(
                debug,
                f"ARMED={armed} FOLLOW={following} STREAM={receiver.get_status()}",
                (10, CFG.height - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                debug,
                f"FOUND={found} ACT={action} SEND L={send_left} R={send_right}",
                (10, CFG.height - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            if CFG.show_debug:
                cv2.imshow(CFG.window_name, debug)
                cv2.imshow(CFG.binary_window_name, binary)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                running = False
            elif key == ord("a"):
                armed = not armed
                if not armed:
                    robot.stop()
                    last_sent_left = 0
                    last_sent_right = 0
                    last_send_time = time.time()
            elif key == ord("f"):
                following = not following
            elif key == ord("s"):
                armed = False
                following = False
                robot.stop()
                last_sent_left = 0
                last_sent_right = 0
                last_send_time = time.time()

            time.sleep(0.01)

    finally:
        robot.stop()
        receiver.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())