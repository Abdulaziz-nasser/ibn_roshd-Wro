"""Single-owner motion command manager."""
from __future__ import annotations

import itertools
import logging
import time
from dataclasses import dataclass
from typing import Any

from .protocol import clamp
from .serial_link import SerialLink
from .types import MotionResult

LOG = logging.getLogger(__name__)


@dataclass(slots=True)
class ActiveCommand:
    command_id: int
    name: str
    payload: dict[str, Any]
    started_at: float
    host_deadline: float
    ack_deadline: float
    acknowledged: bool = False
    resend_count: int = 0


class MotionController:
    def __init__(self, link: SerialLink, config: dict[str, Any], dry_run: bool = False) -> None:
        self.link = link
        self.config = config
        self.dry_run = dry_run
        self._ids = itertools.count(1)
        self.active: ActiveCommand | None = None
        self.last_continuous_send = 0.0
        self.continuous_period_s = float(config.get("continuous_command_period_s", 0.05))
        self.steering = config.get("steering", {})

    def _next_id(self) -> int:
        value = next(self._ids)
        if value > 2_000_000_000:
            self._ids = itertools.count(1)
            value = next(self._ids)
        return value

    def steer_deg_to_norm(self, angle_deg: float) -> float:
        center = float(self.steering.get("center_deg", 90.0))
        left = float(self.steering.get("left_deg", 60.0))
        right = float(self.steering.get("right_deg", 115.0))
        angle = clamp(float(angle_deg), min(left, right), max(left, right))
        if angle >= center:
            return clamp((angle - center) / max(1.0, right - center), -1.0, 1.0)
        return clamp((angle - center) / max(1.0, center - left), -1.0, 1.0)

    def drive(self, direction: str, steer_norm: float, pwm: int, force: bool = False) -> None:
        now = time.time()
        if not force and now - self.last_continuous_send < self.continuous_period_s:
            return
        self.link.send(
            "DRIVE",
            dir=direction.upper(),
            steer=clamp(float(steer_norm), -1.0, 1.0),
            pwm=max(0, min(255, int(pwm))),
        )
        self.last_continuous_send = now

    def drive_forward(self, steer_norm: float, pwm: int, force: bool = False) -> None:
        self.drive("F", steer_norm, pwm, force)

    def drive_backward(self, steer_norm: float, pwm: int, force: bool = False) -> None:
        self.drive("B", steer_norm, pwm, force)

    def stop(self) -> None:
        self.link.send("STOP")
        self.active = None

    def cancel(self) -> None:
        if self.active is not None:
            self.link.send("CANCEL", id=self.active.command_id)
        self.stop()

    def _start(self, name: str, host_timeout_s: float, **payload: Any) -> int:
        if self.active is not None:
            raise RuntimeError(f"Cannot start {name}; command {self.active.command_id} is still active")
        command_id = self._next_id()
        payload = {"id": command_id, **payload}
        now = time.time()
        self.active = ActiveCommand(
            command_id=command_id,
            name=name,
            payload=payload,
            started_at=now,
            host_deadline=now + max(0.5, float(host_timeout_s) + 1.0),
            ack_deadline=now + 0.35,
        )
        self.link.send(name, **payload)
        return command_id

    def start_move(
        self,
        direction: str,
        distance_cm: float,
        pwm: int,
        timeout_s: float,
        steer_norm: float = 0.0,
        hold_heading: bool = True,
    ) -> int:
        return self._start(
            "MOVE",
            timeout_s,
            dir=direction.upper(),
            cm=max(0.0, float(distance_cm)),
            pwm=max(0, min(255, int(pwm))),
            steer=clamp(float(steer_norm), -1.0, 1.0),
            hold=1 if hold_heading else 0,
            timeout_ms=int(max(200, timeout_s * 1000.0)),
        )

    def start_turn(
        self,
        target_yaw: float,
        pwm: int,
        tolerance_deg: float,
        timeout_s: float,
        direction: int = 0,
        mode: str = "F",
    ) -> int:
        direction_text = "A" if direction == 0 else ("R" if direction > 0 else "L")
        return self._start(
            "TURN",
            timeout_s,
            target=float(target_yaw),
            pwm=max(0, min(255, int(pwm))),
            tol=max(0.5, float(tolerance_deg)),
            timeout_ms=int(max(200, timeout_s * 1000.0)),
            dir=direction_text,
            mode=mode.upper(),
        )

    def poll(self) -> MotionResult | None:
        command = self.active
        if command is None:
            return None

        if not command.acknowledged and self.link.consume_ack(command.command_id):
            command.acknowledged = True

        result = self.link.consume_done(command.command_id)
        if result is not None:
            self.active = None
            return result

        now = time.time()
        if not command.acknowledged and now >= command.ack_deadline:
            if command.resend_count < 2:
                command.resend_count += 1
                command.ack_deadline = now + 0.35
                LOG.warning("Resending unacknowledged %s command id=%d", command.name, command.command_id)
                self.link.send(command.name, **command.payload)
            else:
                self.link.send("STOP")
                self.active = None
                return MotionResult(
                    command_id=command.command_id,
                    result="NO_ACK",
                    detail=f"No ACK for {command.name}",
                    timestamp=now,
                )

        if now >= command.host_deadline:
            self.link.send("CANCEL", id=command.command_id)
            self.link.send("STOP")
            self.active = None
            return MotionResult(
                command_id=command.command_id,
                result="HOST_TIMEOUT",
                detail=f"Jetson host timeout for {command.name}",
                timestamp=now,
            )
        return None
