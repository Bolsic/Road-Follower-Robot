from __future__ import annotations

import asyncio
import threading
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
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """
        Starts the receiver in a background thread.
        """
        if self._thread is not None and self._thread.is_alive():
            return
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
        finally:
            loop.close()

    async def _run_whep(self) -> None:
        """
        The core async function that handles the WHEP connection and video stream.
        """
        pc = RTCPeerConnection()

        @pc.on("track")
        async def on_track(track: av.VideoFrame):
            """
            Callback for when a video track is received. This is where frames
            are read from the stream.
            """
            self._status = "receiving"
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
                except asyncio.TimeoutError:
                    self._status = "timeout"
                    break
            self._status = "stopped"

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            """
            Callback for WebRTC connection state changes.
            """
            if pc.connectionState == "failed" or pc.connectionState == "closed":
                await self.stop_whep(pc)

        # The WHEP handshake process
        async with aiohttp.ClientSession() as session:
            # 1. POST the initial offer to the WHEP endpoint
            async with session.post(self.whep_url, ssl=False) as response:
                if response.status != 201:
                    self._status = "failed"
                    return
                
                # The server provides a resource URL for the session in the 'Link' or 'Location' header.
                link_header = response.headers.get("Link")
                if link_header:
                    parts = link_header.split(';')
                    if len(parts) > 0 and parts[0].startswith('<') and parts[0].endswith('>'):
                        resource_url = parts[0][1:-1]
                        url_parts = list(response.url.parts)
                        url_parts[-1] = resource_url
                        resource_url = response.url.with_path(resource_url)
                else:
                    resource_url = response.headers["Location"]


                # 2. Set the remote description from the server's answer
                offer = await response.json()
                await pc.setRemoteDescription(RTCSessionDescription(sdp=offer["sdp"], type=offer["type"]))
                
                # 3. Create our local answer
                await pc.setLocalDescription(await pc.createAnswer())
                
                payload = {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

                # 4. PATCH the answer back to the server to establish the connection
                async with session.patch(resource_url, json=payload, ssl=False) as patch_response:
                    if patch_response.status != 204:
                        self._status = "failed"
                        return

                    # Wait until the connection is closed or fails
                    while self._running and pc.connectionState != "failed" and pc.connectionState != "closed":
                        await asyncio.sleep(0.1)

    async def stop_whep(self, pc: RTCPeerConnection) -> None:
        """
        Closes the WebRTC peer connection.
        """
        await pc.close()
