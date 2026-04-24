from __future__ import annotations

import asyncio
import threading
import time
from typing import Optional
from urllib.parse import urljoin

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
            elif status in ("starting", "connecting", "receiving"):
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
            loop.run_until_complete(self._run_whep())
        except Exception as e:
            self._set_status("thread_crashed", str(e))
        finally:
            loop.close()

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
            while self._running:
                try:
                    # Wait for the next frame
                    frame = await asyncio.wait_for(track.recv(), timeout=1.0)
                    # Convert the frame to a numpy array for OpenCV
                    img = frame.to_ndarray(format="bgr24")
                    if self.flip:
                        img = cv2.flip(img, -1)
                    # Store the frame safely
                    with self._lock:
                        self._latest_frame = img
                        self._frames_received += 1
                        self._last_frame_ts = time.time()
                except asyncio.TimeoutError:
                    self._set_status("timeout", "track.recv timed out")
                    break
                except Exception as e:
                    self._set_status("track_error", str(e))
                    break
            if self._running:
                self._set_status("stopped")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            """
            Callback for WebRTC connection state changes.
            """
            self._set_status(f"pc_{pc.connectionState}")
            if pc.connectionState == "failed" or pc.connectionState == "closed":
                await self.stop_whep(pc)

        # The WHEP handshake process
        timeout = aiohttp.ClientTimeout(total=12)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            # 1. POST the initial offer to the WHEP endpoint
            async with session.post(self.whep_url, ssl=False) as response:
                if response.status != 201:
                    body = await response.text()
                    self._set_status("failed", f"POST {response.status}: {body[:240]}")
                    return
                
                # The server provides a resource URL for the session in the 'Link' or 'Location' header.
                resource_url = None
                link_header = response.headers.get("Link")
                if link_header:
                    parts = link_header.split(';')
                    if len(parts) > 0 and parts[0].startswith('<') and parts[0].endswith('>'):
                        resource_ref = parts[0][1:-1]
                        resource_url = urljoin(str(response.url), resource_ref)

                if resource_url is None:
                    location = response.headers.get("Location")
                    if location is None:
                        self._set_status("failed", "WHEP response missing Link/Location header")
                        return
                    resource_url = urljoin(str(response.url), location)


                # 2. Set the remote description from the server's answer
                offer = await response.json(content_type=None)
                if "sdp" not in offer or "type" not in offer:
                    self._set_status("failed", f"Invalid SDP payload keys: {list(offer.keys())}")
                    return
                await pc.setRemoteDescription(RTCSessionDescription(sdp=offer["sdp"], type=offer["type"]))
                
                # 3. Create our local answer
                await pc.setLocalDescription(await pc.createAnswer())
                
                payload = {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

                # 4. PATCH the answer back to the server to establish the connection
                async with session.patch(resource_url, json=payload, ssl=False) as patch_response:
                    if patch_response.status != 204:
                        body = await patch_response.text()
                        self._set_status("failed", f"PATCH {patch_response.status}: {body[:240]}")
                        return

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
