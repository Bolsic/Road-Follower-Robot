from __future__ import annotations

import asyncio
import threading
import time
from typing import Optional

import aiohttp
import av
import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription


class WHEPReceiver:
    def __init__(self, whep_url: str, width: int, height: int, flip: bool = False) -> None:
        """
        Initializes the WHEPReceiver to connect to a WHEP (WebRTC-HTTP Egress Protocol)
        source and receive a video stream.
        """
        self.whep_url = whep_url
        self.width = width
        self.height = height
        self.flip = flip

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._status = "stopped"
        self._last_error: Optional[str] = None
        self._frames_received = 0
        self._last_frame_ts = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def _set_status(self, status: str, error: Optional[str] = None) -> None:
        with self._lock:
            self._status = status
            if error is not None:
                self._last_error = error
            elif status in ("starting", "connecting", "awaiting_track", "receiving"):
                self._last_error = None

    def get_debug_state(self) -> dict:
        with self._lock:
            frame_age_s = None
            if self._last_frame_ts > 0:
                frame_age_s = time.time() - self._last_frame_ts
            return {
                "status": self._status,
                "last_error": self._last_error,
                "frames_received": self._frames_received,
                "last_frame_age_s": frame_age_s,
                "whep_url": self.whep_url,
            }

    def start(self) -> None:
        """
        Starts the receiver in a background thread.
        """
        if self._thread is not None and self._thread.is_alive():
            return
        self._set_status("starting")
        self._running = True
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """
        Stops the receiver and waits for the background thread to terminate.
        """
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._set_status("stopped")

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Returns the latest video frame received from the stream.
        This method is thread-safe.
        """
        with self._lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()

    def _thread_main(self) -> None:
        """
        The main function for the background thread. It sets up and runs the
        asyncio event loop.
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            while self._running:
                loop.run_until_complete(self._run_whep())
                if not self._running:
                    break
                self._set_status("reconnecting")
                time.sleep(1.0)
        except Exception as e:
            self._set_status("thread_crashed", str(e))
        finally:
            loop.close()

    async def _wait_for_ice_gathering(self, pc: RTCPeerConnection, timeout_s: float = 5.0) -> None:
        start = time.time()
        while pc.iceGatheringState != "complete" and (time.time() - start) < timeout_s:
            await asyncio.sleep(0.05)

    async def _run_whep(self) -> None:
        """
        The core async function that handles the WHEP connection and video stream.
        """
        self._set_status("connecting")
        pc = RTCPeerConnection()

        @pc.on("track")
        async def on_track(track: av.VideoFrame):
            """
            Callback for when a video track is received. This is where frames
            are read from the stream.
            """
            self._set_status("receiving")
            timeout_count = 0
            while self._running:
                try:
                    # Wait for the next frame
                    frame = await asyncio.wait_for(track.recv(), timeout=2.5)
                    # Convert the frame to a numpy array for OpenCV
                    img = frame.to_ndarray(format="bgr24")
                    if self.flip:
                        img = cv2.flip(img, -1)
                    # Store the frame safely
                    with self._lock:
                        self._latest_frame = img
                        self._frames_received += 1
                        self._last_frame_ts = time.time()
                    timeout_count = 0
                except asyncio.TimeoutError:
                    timeout_count += 1
                    self._set_status("awaiting_track", f"track.recv timed out ({timeout_count})")
                    # Keep waiting because initial keyframes can be delayed.
                    if timeout_count < 30:
                        continue
                    self._set_status("failed", "no frames received for 75s after track started")
                    break
                except Exception as e:
                    self._set_status("track_error", str(e))
                    break
            if self._running and timeout_count < 30:
                self._set_status("stopped")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            """
            Callback for WebRTC connection state changes.
            """
            self._set_status(f"pc_{pc.connectionState}")
            if pc.connectionState == "failed" or pc.connectionState == "closed":
                await self.stop_whep(pc)

        # WHEP client role: create local SDP offer, POST as application/sdp,
        # receive SDP answer in response body.
        pc.addTransceiver("video", direction="recvonly")
        await pc.setLocalDescription(await pc.createOffer())
        await self._wait_for_ice_gathering(pc, timeout_s=5.0)

        offer_sdp = pc.localDescription.sdp if pc.localDescription is not None else ""
        if not offer_sdp:
            self._set_status("failed", "local SDP offer was empty")
            return

        timeout = aiohttp.ClientTimeout(total=12)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            headers = {
                "Content-Type": "application/sdp",
                "Accept": "application/sdp",
            }

            async with session.post(
                self.whep_url,
                data=offer_sdp.encode("utf-8"),
                headers=headers,
                ssl=False,
            ) as response:
                if response.status != 201:
                    body = await response.text()
                    self._set_status("failed", f"POST {response.status}: {body[:240]}")
                    return

                answer_sdp = await response.text()
                if not answer_sdp.strip():
                    self._set_status("failed", "empty SDP answer from WHEP server")
                    return

                await pc.setRemoteDescription(
                    RTCSessionDescription(sdp=answer_sdp, type="answer")
                )

                self._set_status("awaiting_track")
                # Wait until the connection is closed or fails
                while self._running and pc.connectionState != "failed" and pc.connectionState != "closed":
                    await asyncio.sleep(0.1)

        if self._running and pc.connectionState != "closed":
            await self.stop_whep(pc)

    async def stop_whep(self, pc: RTCPeerConnection) -> None:
        """
        Closes the WebRTC peer connection.
        """
        await pc.close()
