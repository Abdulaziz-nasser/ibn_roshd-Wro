"""Shared data structures."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class Telemetry:
    timestamp: float = 0.0
    boot_id: int = 0
    yaw: float = 0.0
    state: str = "UNKNOWN"
    steer: float = 0.0
    speed: int = 0
    dF: float = -1.0
    dL: float = -1.0
    dR: float = -1.0
    dB: float = -1.0
    enc_cm: float = 0.0
    enc_ticks: int = 0
    armed: bool = False

    def side_open_direction(self) -> int:
        """Return -1 for LEFT and +1 for RIGHT, favoring the more open side."""
        left_valid = self.dL >= 0
        right_valid = self.dR >= 0
        if left_valid and right_valid:
            return -1 if self.dL > self.dR else +1
        if left_valid:
            return -1
        if right_valid:
            return +1
        return +1


@dataclass(slots=True)
class MotionResult:
    command_id: int
    result: str
    progress_cm: float = 0.0
    detail: str = ""
    timestamp: float = 0.0

    @property
    def ok(self) -> bool:
        return self.result.upper() == "OK"


@dataclass(slots=True)
class Detection:
    color: str | None = None
    confidence: float = 0.0
    bbox: tuple[int, int, int, int] | None = None
    area: float = 0.0
    pixels: int = 0
    center_x_frac: float = 0.5
    center_y_frac: float = 0.5
    aspect: float = 0.0
    solidity: float = 0.0


@dataclass(slots=True)
class VisionObservation:
    timestamp: float = 0.0
    line: Detection = field(default_factory=Detection)
    pillar: Detection = field(default_factory=Detection)
    pixel_counts: dict[str, int] = field(default_factory=dict)
    brightness: float = 0.0
    saturation: float = 0.0
    blur_score: float = 0.0
    image_quality: float = 0.0
    debug: dict[str, Any] = field(default_factory=dict)
