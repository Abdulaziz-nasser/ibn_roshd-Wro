"""One-camera capture service supporting CSI/Argus and USB/V4L2."""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any

import cv2 as cv
import numpy as np

LOG = logging.getLogger(__name__)


@dataclass(slots=True)
class CameraFrame:
    timestamp: float
    image: np.ndarray
    sequence: int


def build_csi_pipeline(config: dict[str, Any]) -> str:
    sensor_id = int(config.get("sensor_id", 0))
    capture_w = int(config.get("capture_width", 1280))
    capture_h = int(config.get("capture_height", 720))
    output_w = int(config.get("output_width", 640))
    output_h = int(config.get("output_height", 480))
    fps = int(config.get("fps", 30))
    flip = int(config.get("flip_method", 0))

    source_properties: list[str] = []
    exposure_range = config.get("exposure_time_range_ns")
    gain_range = config.get("gain_range")
    isp_gain_range = config.get("isp_digital_gain_range")
    if isinstance(exposure_range, list) and len(exposure_range) == 2:
        source_properties.append(f'exposuretimerange="{int(exposure_range[0])} {int(exposure_range[1])}"')
    if isinstance(gain_range, list) and len(gain_range) == 2:
        source_properties.append(f'gainrange="{float(gain_range[0])} {float(gain_range[1])}"')
    if isinstance(isp_gain_range, list) and len(isp_gain_range) == 2:
        source_properties.append(
            f'ispdigitalgainrange="{float(isp_gain_range[0])} {float(isp_gain_range[1])}"'
        )
    if bool(config.get("ae_lock", False)):
        source_properties.append("aelock=true")
    if bool(config.get("awb_lock", False)):
        source_properties.append("awblock=true")

    props = " ".join(source_properties)
    if props:
        props = " " + props

    return (
        f"nvarguscamerasrc sensor-id={sensor_id}{props} ! "
        f"video/x-raw(memory:NVMM), width={capture_w}, height={capture_h}, "
        f"format=(string)NV12, framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip} ! "
        f"video/x-raw, width={output_w}, height={output_h}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


class OneCamera:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.camera_type = str(config.get("type", "csi")).lower()
        self.reopen_s = float(config.get("reopen_s", 1.0))
        self._capture: cv.VideoCapture | None = None
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest: CameraFrame | None = None
        self._sequence = 0
        self._last_error = ""

    @property
    def last_error(self) -> str:
        return self._last_error

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._running.set()
        self._thread = threading.Thread(target=self._capture_loop, name="robot-camera", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._close()

    def _open(self) -> None:
        self._close()
        if self.camera_type == "csi":
            pipeline = build_csi_pipeline(self.config)
            LOG.info("Opening CSI camera with pipeline: %s", pipeline)
            capture = cv.VideoCapture(pipeline, cv.CAP_GSTREAMER)
        elif self.camera_type == "usb":
            index = int(self.config.get("device_index", 0))
            LOG.info("Opening USB camera index %d", index)
            capture = cv.VideoCapture(index, cv.CAP_V4L2)
            capture.set(cv.CAP_PROP_FRAME_WIDTH, int(self.config.get("output_width", 640)))
            capture.set(cv.CAP_PROP_FRAME_HEIGHT, int(self.config.get("output_height", 480)))
            capture.set(cv.CAP_PROP_FPS, int(self.config.get("fps", 30)))
            capture.set(cv.CAP_PROP_BUFFERSIZE, 1)
        else:
            raise ValueError(f"Unsupported camera type: {self.camera_type}")

        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"Could not open {self.camera_type} camera")
        self._capture = capture
        self._last_error = ""

    def _close(self) -> None:
        capture = self._capture
        self._capture = None
        if capture:
            try:
                capture.release()
            except Exception:
                pass

    def _capture_loop(self) -> None:
        while self._running.is_set():
            if self._capture is None:
                try:
                    self._open()
                except Exception as exc:
                    self._last_error = str(exc)
                    LOG.warning("Camera unavailable: %s", exc)
                    time.sleep(self.reopen_s)
                    continue

            ok, frame = self._capture.read() if self._capture else (False, None)
            if not ok or frame is None:
                self._last_error = "Camera frame read failed"
                LOG.warning(self._last_error)
                self._close()
                time.sleep(self.reopen_s)
                continue

            self._sequence += 1
            packet = CameraFrame(time.time(), frame, self._sequence)
            with self._lock:
                self._latest = packet

    def latest(self, copy_frame: bool = True) -> CameraFrame | None:
        with self._lock:
            packet = self._latest
            if packet is None:
                return None
            image = packet.image.copy() if copy_frame else packet.image
            return CameraFrame(packet.timestamp, image, packet.sequence)
