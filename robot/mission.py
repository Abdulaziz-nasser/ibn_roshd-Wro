"""Mission state machine for the one-camera autonomous robot.

The state machine is the only high-level owner of motion. Vision produces
observations; MotionController serializes commands; Arduino performs real-time
motor, encoder, ultrasonic, and yaw watchdogs.
"""
from __future__ import annotations

import logging
import time
from enum import Enum, auto
from typing import Any

from .motion import MotionController
from .protocol import clamp, shortest_error, wrap180
from .types import MotionResult, Telemetry, VisionObservation

LOG = logging.getLogger(__name__)

LEFT = -1
RIGHT = +1


class State(Enum):
    WAIT_START = auto()
    ESC_S1 = auto()
    ESC_S2 = auto()
    ESC_S3 = auto()
    ESC_HOLD = auto()
    ESC_S4 = auto()
    RUN = auto()
    CORNER_TURN = auto()
    CORNER_BACKCENTER = auto()
    PILLAR_TURN = auto()
    PILLAR_FWD = auto()
    PILLAR_RETURN = auto()
    RECOVERY_CLEAR = auto()
    RECOVERY_RETURN = auto()
    LAST_PREP_FWD = auto()
    LAST_BACK = auto()
    LAST_CORNER_TURN = auto()
    LAST_HOLD = auto()
    LAST_WAIT_FRONT = auto()
    LAST_T1 = auto()
    LAST_FWD1 = auto()
    LAST_RET1 = auto()
    LAST_FWD2 = auto()
    LAST_T2 = auto()
    LAST_FWD3 = auto()
    LAST_FINAL_RETURN = auto()
    DONE = auto()
    FAULT = auto()


TURN_STATES = {
    State.CORNER_TURN,
    State.PILLAR_TURN,
    State.PILLAR_RETURN,
    State.RECOVERY_RETURN,
    State.LAST_CORNER_TURN,
    State.LAST_T1,
    State.LAST_RET1,
    State.LAST_T2,
    State.LAST_FINAL_RETURN,
}

MOVE_STATES = {
    State.ESC_S1,
    State.ESC_S2,
    State.ESC_S3,
    State.ESC_S4,
    State.PILLAR_FWD,
    State.RECOVERY_CLEAR,
    State.LAST_BACK,
    State.LAST_FWD1,
    State.LAST_FWD2,
    State.LAST_FWD3,
}

CONTINUOUS_STATES = {
    State.ESC_HOLD,
    State.RUN,
    State.CORNER_BACKCENTER,
    State.LAST_PREP_FWD,
    State.LAST_HOLD,
    State.LAST_WAIT_FRONT,
}


class MissionController:
    def __init__(self, config: dict[str, Any], motion: MotionController) -> None:
        self.config = config
        self.motion = motion
        self.mission_cfg = config.get("mission", {})
        self.drive_cfg = config.get("drive", {})
        self.steer_cfg = config.get("steering", {})
        self.wall_cfg = config.get("wall_avoid", {})
        self.stuck_cfg = config.get("stuck_recovery", {})
        self.escape_cfg = config.get("escape", {})
        self.pillar_cfg = config.get("pillar", {})
        self.corner_cfg = config.get("corners", {})
        self.last_cfg = config.get("last_sequence", {})

        self.state = State.WAIT_START
        self.state_entered_at = time.time()
        self.entry_pending = True
        self.start_requested = False
        self.fault_reason = ""

        self.start_yaw = 0.0
        self.yaw_ref = 0.0
        self.corridor_index = 0
        self.turn_count = 0
        self.force_corner_direction: int | None = self._configured_global_direction()

        self.line_latched: str | None = None
        self.corner_direction = RIGHT
        self.corner_target_yaw = 0.0
        self.corner_mode = "BACKWARD"

        self.pillar_color: str | None = None
        self.pillar_corridor_yaw = 0.0
        self.pillar_target_yaw = 0.0
        self.pillar_side_fraction = 0.0

        self.escape_direction = RIGHT
        self.escape_retry_count = 0
        self.escape_hold_until = 0.0

        self.last_main_yaw = 0.0
        self.last_first_turn_sign = RIGHT
        self.last_turn_target = 0.0

        self.recovery_reason = ""
        self.recovery_resume_state = State.RUN
        self.recovery_clear_direction = "B"
        self.recovery_attempts: dict[State, int] = {}

        self._continuous_encoder_ref = 0.0
        self._continuous_progress_at = time.time()
        self._continuous_commanded_speed = 0
        self._continuous_direction = "F"
        self._last_status_log = 0.0

    # ------------------------------------------------------------------
    # Public controls
    # ------------------------------------------------------------------
    def request_start(self) -> None:
        self.start_requested = True

    def hard_reset(self, telemetry: Telemetry | None = None) -> None:
        self.motion.stop()
        self.state = State.WAIT_START
        self.entry_pending = True
        self.state_entered_at = time.time()
        self.start_requested = False
        self.fault_reason = ""
        self.turn_count = 0
        self.corridor_index = 0
        self.line_latched = None
        self.force_corner_direction = self._configured_global_direction()
        self.recovery_attempts.clear()
        if telemetry is not None:
            self.start_yaw = telemetry.yaw
            self.yaw_ref = telemetry.yaw
        LOG.info("Mission reset to WAIT_START")

    def status(self) -> dict[str, Any]:
        return {
            "state": self.state.name,
            "yaw_ref": self.yaw_ref,
            "turn_count": self.turn_count,
            "corridor_index": self.corridor_index,
            "line_latched": self.line_latched,
            "corner_direction": self.corner_direction,
            "corner_target_yaw": self.corner_target_yaw,
            "pillar_color": self.pillar_color,
            "fault_reason": self.fault_reason,
        }

    # ------------------------------------------------------------------
    # State transition helpers
    # ------------------------------------------------------------------
    def _configured_global_direction(self) -> int | None:
        text = str(self.mission_cfg.get("all_turns_direction", "AUTO")).strip().upper()
        if text == "LEFT":
            return LEFT
        if text == "RIGHT":
            return RIGHT
        return None

    def _set_state(self, state: State, telemetry: Telemetry, reason: str = "") -> None:
        if self.motion.active is not None and state not in {State.RECOVERY_CLEAR, State.FAULT, State.DONE}:
            # Every command state consumes its result before a transition. This
            # guard catches accidental overlapping command ownership.
            LOG.debug("Transition while motion active: %s -> %s", self.state.name, state.name)
        self.state = state
        self.state_entered_at = time.time()
        self.entry_pending = True
        self._continuous_encoder_ref = telemetry.enc_cm
        self._continuous_progress_at = time.time()
        self._continuous_commanded_speed = 0
        if reason:
            LOG.info("STATE -> %s (%s)", state.name, reason)
        else:
            LOG.info("STATE -> %s", state.name)

    def _on_enter(self, telemetry: Telemetry) -> None:
        self.entry_pending = False
        cfg = self.config

        if self.state == State.WAIT_START:
            self.motion.stop()
            return

        if self.state == State.ESC_S1:
            self._start_escape_step("s1")
        elif self.state == State.ESC_S2:
            self._start_escape_step("s2")
        elif self.state == State.ESC_S3:
            self._start_escape_step("s3")
        elif self.state == State.ESC_HOLD:
            self.escape_hold_until = time.time() + float(self.escape_cfg.get("hold_s", 0.5))
        elif self.state == State.ESC_S4:
            self._start_escape_step("s4")

        elif self.state == State.CORNER_TURN:
            turn_cfg = self._corner_turn_settings(self.turn_count + 1)
            self.motion.start_turn(
                target_yaw=self.corner_target_yaw,
                pwm=int(turn_cfg["speed"]),
                tolerance_deg=float(turn_cfg["tolerance_deg"]),
                timeout_s=float(turn_cfg["timeout_s"]),
                direction=self.corner_direction,
                mode="B" if self.corner_mode == "BACKWARD" else "F",
            )

        elif self.state == State.PILLAR_TURN:
            self.motion.start_turn(
                target_yaw=self.pillar_target_yaw,
                pwm=int(self.pillar_cfg.get("turn_speed", 38)),
                tolerance_deg=float(self.pillar_cfg.get("turn_tolerance_deg", 6.0)),
                timeout_s=float(self.pillar_cfg.get("turn_timeout_s", 2.0)),
                direction=RIGHT if self.pillar_color == "RED" else LEFT,
                mode="F",
            )
        elif self.state == State.PILLAR_FWD:
            distance = self._pillar_forward_distance()
            self.motion.start_move(
                direction="F",
                distance_cm=distance,
                pwm=int(self.pillar_cfg.get("forward_speed", 34)),
                timeout_s=float(self.pillar_cfg.get("forward_timeout_s", 2.5)),
                steer_norm=0.0,
                hold_heading=True,
            )
        elif self.state == State.PILLAR_RETURN:
            direction = LEFT if self.pillar_color == "RED" else RIGHT
            self.motion.start_turn(
                target_yaw=self.pillar_corridor_yaw,
                pwm=int(self.pillar_cfg.get("return_speed", self.pillar_cfg.get("turn_speed", 38))),
                tolerance_deg=float(self.pillar_cfg.get("return_tolerance_deg", 5.0)),
                timeout_s=float(self.pillar_cfg.get("return_timeout_s", 2.0)),
                direction=direction,
                mode="F",
            )

        elif self.state == State.RECOVERY_CLEAR:
            self.motion.start_move(
                direction=self.recovery_clear_direction,
                distance_cm=float(self.stuck_cfg.get("clear_distance_cm", 12.0)),
                pwm=int(self.stuck_cfg.get("clear_speed", 36)),
                timeout_s=float(self.stuck_cfg.get("clear_timeout_s", 2.5)),
                steer_norm=0.0,
                hold_heading=False,
            )
        elif self.state == State.RECOVERY_RETURN:
            self.motion.start_turn(
                target_yaw=self.yaw_ref,
                pwm=int(self.stuck_cfg.get("return_speed", 38)),
                tolerance_deg=float(self.stuck_cfg.get("return_tolerance_deg", 5.0)),
                timeout_s=float(self.stuck_cfg.get("return_timeout_s", 2.5)),
                direction=0,
                mode="F",
            )

        elif self.state == State.LAST_BACK:
            self.motion.start_move(
                direction="B",
                distance_cm=float(self.last_cfg.get("back_distance_cm", 16.0)),
                pwm=int(self.last_cfg.get("back_speed", 40)),
                timeout_s=float(self.last_cfg.get("back_timeout_s", 3.0)),
                steer_norm=0.0,
                hold_heading=True,
            )
        elif self.state == State.LAST_CORNER_TURN:
            last_corner = self.last_cfg.get("corner_turn", {})
            self.motion.start_turn(
                target_yaw=self.corner_target_yaw,
                pwm=int(last_corner.get("speed", 34)),
                tolerance_deg=float(last_corner.get("tolerance_deg", 6.0)),
                timeout_s=float(last_corner.get("timeout_s", 1.5)),
                direction=self.corner_direction,
                mode="F",
            )
        elif self.state == State.LAST_HOLD:
            self.escape_hold_until = time.time() + float(self.last_cfg.get("no_avoid_hold_s", 0.5))
        elif self.state == State.LAST_T1:
            t1 = self.last_cfg.get("t1", {})
            self.motion.start_turn(
                target_yaw=self.last_turn_target,
                pwm=int(t1.get("speed", 36)),
                tolerance_deg=float(t1.get("tolerance_deg", 4.0)),
                timeout_s=float(t1.get("timeout_s", 2.0)),
                direction=self.last_first_turn_sign,
                mode="F",
            )
        elif self.state == State.LAST_FWD1:
            self.motion.start_move(
                direction="F",
                distance_cm=float(self.last_cfg.get("fwd1_cm", 5.0)),
                pwm=int(self.last_cfg.get("forward_speed", self.drive_cfg.get("speed", 34))),
                timeout_s=float(self.last_cfg.get("fwd1_timeout_s", 2.0)),
                hold_heading=True,
            )
        elif self.state == State.LAST_RET1:
            # RET1 is explicitly opposite to T1 and targets the main corridor yaw.
            t1 = self.last_cfg.get("t1", {})
            self.motion.start_turn(
                target_yaw=self.last_main_yaw,
                pwm=int(t1.get("speed", 36)),
                tolerance_deg=float(t1.get("tolerance_deg", 4.0)),
                timeout_s=float(t1.get("timeout_s", 2.0)),
                direction=-self.last_first_turn_sign,
                mode="F",
            )
        elif self.state == State.LAST_FWD2:
            self.motion.start_move(
                direction="F",
                distance_cm=float(self.last_cfg.get("fwd2_cm", 18.0)),
                pwm=int(self.last_cfg.get("forward_speed", self.drive_cfg.get("speed", 34))),
                timeout_s=float(self.last_cfg.get("fwd2_timeout_s", 3.0)),
                hold_heading=True,
            )
        elif self.state == State.LAST_T2:
            t2 = self.last_cfg.get("t2", {})
            opposite = -self.last_first_turn_sign
            self.motion.start_turn(
                target_yaw=self.last_turn_target,
                pwm=int(t2.get("speed", 36)),
                tolerance_deg=float(t2.get("tolerance_deg", 4.0)),
                timeout_s=float(t2.get("timeout_s", 2.0)),
                direction=opposite,
                mode="F",
            )
        elif self.state == State.LAST_FWD3:
            self.motion.start_move(
                direction="F",
                distance_cm=float(self.last_cfg.get("fwd3_cm", 25.0)),
                pwm=int(self.last_cfg.get("forward_speed", self.drive_cfg.get("speed", 34))),
                timeout_s=float(self.last_cfg.get("fwd3_timeout_s", 2.0)),
                hold_heading=True,
            )
        elif self.state == State.LAST_FINAL_RETURN:
            final_cfg = self.last_cfg.get("final_return", {})
            self.motion.start_turn(
                target_yaw=self.last_turn_target,
                pwm=int(final_cfg.get("speed", 35)),
                tolerance_deg=float(final_cfg.get("tolerance_deg", 3.0)),
                timeout_s=float(final_cfg.get("timeout_s", 2.7)),
                direction=0,
                mode="F",
            )

        elif self.state in {State.DONE, State.FAULT}:
            self.motion.stop()

    # ------------------------------------------------------------------
    # Main step
    # ------------------------------------------------------------------
    def step(self, telemetry: Telemetry, vision: VisionObservation, now: float | None = None) -> None:
        now = time.time() if now is None else now
        if self.entry_pending:
            self._on_enter(telemetry)

        result = self.motion.poll()
        if result is not None:
            self._handle_motion_result(result, telemetry)
            if self.entry_pending:
                self._on_enter(telemetry)

        if self.state == State.WAIT_START:
            if self.start_requested or telemetry.armed:
                self.start_requested = False
                self._begin_run(telemetry)
            return

        if self.state == State.ESC_HOLD:
            self._drive_yaw_centered(telemetry, int(self.escape_cfg.get("hold_speed", 40)), wall_avoid=False)
            if now >= self.escape_hold_until:
                self.motion.stop()
                self._set_state(State.ESC_S4, telemetry, "escape hold finished")

        elif self.state == State.RUN:
            self._run_state(telemetry, vision)

        elif self.state == State.CORNER_BACKCENTER:
            self._drive_yaw_centered(
                telemetry,
                int(self.corner_cfg.get("back_center_speed", 31)),
                direction="B",
                wall_avoid=False,
            )
            back_stop = float(self.corner_cfg.get("back_stop_cm", 55.0))
            if telemetry.dB >= 0 and telemetry.dB <= back_stop:
                self.motion.stop()
                self._complete_corner(telemetry)

        elif self.state == State.LAST_PREP_FWD:
            self._drive_yaw_centered(
                telemetry,
                int(self.last_cfg.get("approach_speed", self.drive_cfg.get("speed", 34))),
                wall_avoid=False,
            )
            threshold = float(self.last_cfg.get("preturn_front_cm", 5.0))
            if telemetry.dF >= 0 and telemetry.dF <= threshold:
                self.motion.stop()
                self._set_state(State.LAST_BACK, telemetry, "front preturn distance reached")

        elif self.state == State.LAST_HOLD:
            self._drive_yaw_centered(
                telemetry,
                int(self.last_cfg.get("forward_speed", self.drive_cfg.get("speed", 34))),
                wall_avoid=False,
            )
            if now >= self.escape_hold_until:
                self.motion.stop()
                self._set_state(State.LAST_WAIT_FRONT, telemetry, "last hold finished")

        elif self.state == State.LAST_WAIT_FRONT:
            self._drive_yaw_centered(
                telemetry,
                int(self.last_cfg.get("forward_speed", self.drive_cfg.get("speed", 34))),
                wall_avoid=False,
            )
            threshold = float(self.last_cfg.get("wait_front_stop_cm", 20.0))
            if telemetry.dF >= 0 and telemetry.dF < threshold:
                self.motion.stop()
                self.last_first_turn_sign = telemetry.side_open_direction()
                t1_deg = float(self.last_cfg.get("t1", {}).get("angle_deg", 90.0))
                self.last_turn_target = wrap180(self.last_main_yaw + self.last_first_turn_sign * t1_deg)
                self._set_state(State.LAST_T1, telemetry, "side ultrasonic selected T1 direction")

        if self.state in CONTINUOUS_STATES:
            self._check_continuous_stall(telemetry, now)

        if now - self._last_status_log >= 2.0:
            LOG.debug(
                "state=%s yaw=%.1f ref=%.1f enc=%.1f F/L/R/B=%.0f/%.0f/%.0f/%.0f",
                self.state.name,
                telemetry.yaw,
                self.yaw_ref,
                telemetry.enc_cm,
                telemetry.dF,
                telemetry.dL,
                telemetry.dR,
                telemetry.dB,
            )
            self._last_status_log = now

    # ------------------------------------------------------------------
    # Entry/action helpers
    # ------------------------------------------------------------------
    def _begin_run(self, telemetry: Telemetry) -> None:
        self.motion.stop()
        self.start_yaw = telemetry.yaw
        self.yaw_ref = telemetry.yaw
        self.corridor_index = 0
        self.turn_count = 0
        self.line_latched = None
        self.pillar_color = None
        self.fault_reason = ""
        self.recovery_attempts.clear()
        self.escape_direction = telemetry.side_open_direction()
        self.escape_retry_count = 0
        if bool(self.escape_cfg.get("enabled", True)):
            self._set_state(State.ESC_S1, telemetry, "mission start with escape sequence")
        else:
            self._set_state(State.RUN, telemetry, "mission start")

    def _start_escape_step(self, name: str) -> None:
        step = self.escape_cfg.get("steps", {}).get(name, {})
        direction = str(step.get("direction", "F")).upper()
        steer = float(step.get("steer", 0.0)) * self.escape_direction
        self.motion.start_move(
            direction=direction,
            distance_cm=float(step.get("distance_cm", 5.0)),
            pwm=int(step.get("speed", 36)),
            timeout_s=float(step.get("timeout_s", 2.0)),
            steer_norm=steer,
            hold_heading=False,
        )

    def _corner_turn_settings(self, turn_number: int) -> dict[str, Any]:
        defaults = self.corner_cfg.get("default_turn", {})
        per_turn = self.corner_cfg.get("per_turn", {})
        specific = per_turn.get(str(turn_number), {}) if isinstance(per_turn, dict) else {}
        return {
            "angle_deg": float(specific.get("angle_deg", defaults.get("angle_deg", 90.0))),
            "speed": int(specific.get("speed", defaults.get("speed", 34))),
            "tolerance_deg": float(specific.get("tolerance_deg", defaults.get("tolerance_deg", 6.0))),
            "timeout_s": float(specific.get("timeout_s", defaults.get("timeout_s", 1.5))),
            "mode": str(specific.get("mode", defaults.get("mode", "BACKWARD"))).upper(),
            "front_trigger_cm": float(
                specific.get("front_trigger_cm", defaults.get("front_trigger_cm", 36.0))
            ),
        }

    def _pillar_forward_distance(self) -> float:
        base = float(self.pillar_cfg.get("forward_base_cm", 2.0))
        gain = float(self.pillar_cfg.get("forward_gain_cm", 19.0))
        minimum = float(self.pillar_cfg.get("forward_min_cm", 0.0))
        maximum = float(self.pillar_cfg.get("forward_max_cm", 20.0))
        return clamp(base + gain * self.pillar_side_fraction, minimum, maximum)

    def _run_state(self, telemetry: Telemetry, vision: VisionObservation) -> None:
        # Floor line takes priority over a new pillar because it defines a corner.
        if self.line_latched is None and vision.line.color in {"BLUE", "ORANGE"}:
            self._latch_corner(vision.line.color)

        if self.line_latched is None and vision.pillar.color in {"RED", "GREEN"}:
            self._begin_pillar(telemetry, vision)
            return

        if self.line_latched is not None:
            settings = self._corner_turn_settings(self.turn_count + 1)
            trigger = float(settings["front_trigger_cm"])
            if telemetry.dF >= 0 and telemetry.dF < trigger:
                if bool(self.last_cfg.get("enabled", True)) and self.turn_count + 1 >= int(
                    self.mission_cfg.get("max_turns", 4)
                ):
                    self._set_state(State.LAST_PREP_FWD, telemetry, "last corner line reached")
                else:
                    self._set_state(State.CORNER_TURN, telemetry, "corner front trigger reached")
                return

        speed = self._run_speed(telemetry)
        self._drive_yaw_centered(telemetry, speed, wall_avoid=True)

    def _latch_corner(self, color: str) -> None:
        base_direction = LEFT if color == "BLUE" else RIGHT
        lock_all = bool(self.mission_cfg.get("lock_all_turns_one_direction", True))
        if self.force_corner_direction is None:
            self.force_corner_direction = base_direction if lock_all else None
        self.corner_direction = self.force_corner_direction or base_direction
        next_turn = self.turn_count + 1
        settings = self._corner_turn_settings(next_turn)
        self.corner_mode = str(settings["mode"]).upper()
        is_last = bool(self.last_cfg.get("enabled", True)) and next_turn >= int(
            self.mission_cfg.get("max_turns", 4)
        )
        if is_last:
            angle = float(self.last_cfg.get("corner_turn", {}).get("angle_deg", settings["angle_deg"]))
            # The custom last-corner turn is intentionally performed forward.
            self.corner_mode = "FORWARD"
        else:
            angle = float(settings["angle_deg"])
        self.corner_target_yaw = wrap180(self.yaw_ref + self.corner_direction * angle)
        self.line_latched = color
        LOG.info(
            "Corner latched: color=%s direction=%s mode=%s target=%.1f",
            color,
            "LEFT" if self.corner_direction < 0 else "RIGHT",
            self.corner_mode,
            self.corner_target_yaw,
        )

    def _begin_pillar(self, telemetry: Telemetry, vision: VisionObservation) -> None:
        self.pillar_color = vision.pillar.color
        self.pillar_corridor_yaw = self.yaw_ref
        sign = RIGHT if self.pillar_color == "RED" else LEFT
        self.pillar_target_yaw = wrap180(
            self.pillar_corridor_yaw + sign * float(self.pillar_cfg.get("turn_angle_deg", 90.0))
        )
        self.pillar_side_fraction = float(clamp(abs(vision.pillar.center_x_frac - 0.5) * 2.0, 0.0, 1.0))
        self._set_state(State.PILLAR_TURN, telemetry, f"{self.pillar_color} pillar accepted")

    def _complete_corner(self, telemetry: Telemetry) -> None:
        self.corridor_index += self.corner_direction
        self.yaw_ref = self.corner_target_yaw
        self.turn_count += 1
        self.line_latched = None
        self._set_state(State.RUN, telemetry, f"corner committed; turns={self.turn_count}")

    # ------------------------------------------------------------------
    # Motion result routing
    # ------------------------------------------------------------------
    def _handle_motion_result(self, result: MotionResult, telemetry: Telemetry) -> None:
        state = self.state
        LOG.info("Motion result state=%s id=%d result=%s progress=%.1f", state.name, result.command_id, result.result, result.progress_cm)
        accepted_turn = result.result in {"OK", "TIMEOUT"}

        if state == State.ESC_S1:
            self._escape_result(result, telemetry, State.ESC_S2)
        elif state == State.ESC_S2:
            self._escape_result(result, telemetry, State.ESC_S3)
        elif state == State.ESC_S3:
            self._escape_result(result, telemetry, State.ESC_HOLD)
        elif state == State.ESC_S4:
            self._escape_result(result, telemetry, State.RUN)

        elif state == State.CORNER_TURN:
            if accepted_turn:
                self.yaw_ref = self.corner_target_yaw
                if self.corner_mode == "BACKWARD":
                    self._set_state(State.CORNER_BACKCENTER, telemetry, f"corner turn {result.result}")
                else:
                    self._complete_corner(telemetry)
            else:
                self._trigger_recovery(telemetry, f"corner turn {result.result}", State.RUN, "B")

        elif state == State.PILLAR_TURN:
            if accepted_turn:
                self.yaw_ref = self.pillar_target_yaw
                self._set_state(State.PILLAR_FWD, telemetry, f"pillar turn {result.result}")
            else:
                self._trigger_recovery(telemetry, f"pillar turn {result.result}", State.RUN, "B")

        elif state == State.PILLAR_FWD:
            if result.ok:
                self._set_state(State.PILLAR_RETURN, telemetry, "pillar forward complete")
            else:
                # Never keep pushing into a wall waiting for an impossible distance.
                self.pillar_color = None
                self.yaw_ref = self.pillar_corridor_yaw
                self._trigger_recovery(
                    telemetry,
                    f"pillar forward {result.result}",
                    State.RUN,
                    "B",
                )

        elif state == State.PILLAR_RETURN:
            if accepted_turn:
                self.yaw_ref = self.pillar_corridor_yaw
                self.pillar_color = None
                self._set_state(State.RUN, telemetry, f"pillar return {result.result}")
            else:
                self._trigger_recovery(telemetry, f"pillar return {result.result}", State.RUN, "B")

        elif state == State.RECOVERY_CLEAR:
            if result.result in {"OK", "TIMEOUT", "BLOCKED"}:
                self._set_state(State.RECOVERY_RETURN, telemetry, f"recovery clear {result.result}")
            else:
                self._fault(telemetry, f"Recovery clear failed: {result.result}")

        elif state == State.RECOVERY_RETURN:
            if accepted_turn:
                resume = self.recovery_resume_state
                self._set_state(resume, telemetry, f"recovery return {result.result}")
            else:
                self._fault(telemetry, f"Recovery yaw return failed: {result.result}")

        elif state == State.LAST_BACK:
            if result.result in {"OK", "TIMEOUT", "BLOCKED"}:
                self._set_state(State.LAST_CORNER_TURN, telemetry, f"last back {result.result}")
            else:
                self._trigger_recovery(telemetry, f"last back {result.result}", State.LAST_BACK, "F")

        elif state == State.LAST_CORNER_TURN:
            if accepted_turn:
                self.corridor_index += self.corner_direction
                self.yaw_ref = self.corner_target_yaw
                self.last_main_yaw = self.yaw_ref
                self.turn_count += 1
                self.line_latched = None
                self._set_state(State.LAST_HOLD, telemetry, f"last corner {result.result}")
            else:
                self._trigger_recovery(
                    telemetry,
                    f"last corner turn {result.result}",
                    State.LAST_CORNER_TURN,
                    "B",
                )

        elif state == State.LAST_T1:
            if accepted_turn:
                self.yaw_ref = self.last_turn_target
                self._set_state(State.LAST_FWD1, telemetry, f"T1 {result.result}")
            else:
                self._trigger_recovery(telemetry, f"T1 {result.result}", State.LAST_T1, "B")

        elif state == State.LAST_FWD1:
            if result.ok:
                self._set_state(State.LAST_RET1, telemetry, "FWD1 complete")
            else:
                self._trigger_recovery(telemetry, f"FWD1 {result.result}", State.LAST_FWD1, "B")

        elif state == State.LAST_RET1:
            if accepted_turn:
                self.yaw_ref = self.last_main_yaw
                self._set_state(State.LAST_FWD2, telemetry, f"RET1 {result.result}")
            else:
                self._trigger_recovery(telemetry, f"RET1 {result.result}", State.LAST_RET1, "B")

        elif state == State.LAST_FWD2:
            if result.ok:
                opposite = -self.last_first_turn_sign
                angle = float(self.last_cfg.get("t2", {}).get("angle_deg", 90.0))
                self.last_turn_target = wrap180(self.last_main_yaw + opposite * angle)
                self._set_state(State.LAST_T2, telemetry, "FWD2 complete")
            else:
                self._trigger_recovery(telemetry, f"FWD2 {result.result}", State.LAST_FWD2, "B")

        elif state == State.LAST_T2:
            if accepted_turn:
                self.yaw_ref = self.last_turn_target
                self._set_state(State.LAST_FWD3, telemetry, f"T2 {result.result}")
            else:
                self._trigger_recovery(telemetry, f"T2 {result.result}", State.LAST_T2, "B")

        elif state == State.LAST_FWD3:
            # User-requested safety: distance completion OR timeout/block/stall stops
            # the robot and advances to the final yaw return.
            final_offset = float(self.last_cfg.get("final_return", {}).get("offset_deg", 0.0))
            self.last_turn_target = wrap180(self.last_main_yaw + final_offset)
            self._set_state(State.LAST_FINAL_RETURN, telemetry, f"FWD3 ended with {result.result}")

        elif state == State.LAST_FINAL_RETURN:
            if accepted_turn:
                self.yaw_ref = self.last_turn_target
                self._set_state(State.DONE, telemetry, f"final return {result.result}")
            else:
                self._fault(telemetry, f"Final return failed: {result.result}")

    def _escape_result(self, result: MotionResult, telemetry: Telemetry, next_state: State) -> None:
        if result.ok:
            self.escape_retry_count = 0
            self._set_state(next_state, telemetry, f"escape step {result.result}")
            return
        max_retries = int(self.escape_cfg.get("max_step_retries", 1))
        if self.escape_retry_count < max_retries:
            self.escape_retry_count += 1
            LOG.warning("Retrying escape state %s after %s", self.state.name, result.result)
            self.entry_pending = True
            self.state_entered_at = time.time()
            return
        LOG.error("Escape step failed twice; stopping escape safely and entering RUN")
        self.motion.stop()
        self.escape_retry_count = 0
        self._set_state(State.RUN, telemetry, f"escape skipped after {result.result}")

    # ------------------------------------------------------------------
    # Driving and watchdog helpers
    # ------------------------------------------------------------------
    def _yaw_steer_norm(self, telemetry: Telemetry) -> float:
        error = shortest_error(self.yaw_ref, telemetry.yaw)
        deadband = float(self.steer_cfg.get("deadband_deg", 0.5))
        if abs(error) <= deadband:
            error = 0.0
        kp = float(self.steer_cfg.get("yaw_kp_norm_per_deg", 0.012))
        sign = float(self.steer_cfg.get("yaw_sign", 1.0))
        maximum = float(self.steer_cfg.get("max_yaw_steer_norm", 0.30))
        return clamp(sign * kp * error, -maximum, maximum)

    def _wall_bias(self, telemetry: Telemetry) -> float:
        on_cm = float(self.wall_cfg.get("on_cm", 15.5))
        max_bias = float(self.wall_cfg.get("max_bias_norm", 0.18))
        min_bias = float(self.wall_cfg.get("min_bias_norm", 0.04))
        exponent = float(self.wall_cfg.get("exponent", 1.0))
        candidates: list[tuple[str, float]] = []
        if telemetry.dL >= 0 and telemetry.dL <= on_cm:
            candidates.append(("L", telemetry.dL))
        if telemetry.dR >= 0 and telemetry.dR <= on_cm:
            candidates.append(("R", telemetry.dR))
        if not candidates:
            return 0.0
        side, distance = min(candidates, key=lambda item: item[1])
        proximity = clamp((on_cm - distance) / max(1e-6, on_cm), 0.0, 1.0)
        magnitude = min_bias + (max_bias - min_bias) * (proximity ** exponent)
        # Left wall -> steer right (+); right wall -> steer left (-).
        bias = magnitude if side == "L" else -magnitude
        if bool(self.wall_cfg.get("invert", False)):
            bias *= -1.0
        return bias

    def _run_speed(self, telemetry: Telemetry) -> int:
        normal = int(self.drive_cfg.get("speed", 34))
        minimum = min(normal, int(self.drive_cfg.get("minimum_speed", 25)))
        speed = normal
        side_on = float(self.drive_cfg.get("side_slow_on_cm", 15.5))
        front_on = float(self.drive_cfg.get("front_slow_on_cm", 45.0))
        factor = float(self.drive_cfg.get("slowdown_factor", 0.65))
        valid_sides = [distance for distance in (telemetry.dL, telemetry.dR) if distance >= 0]
        if valid_sides and min(valid_sides) < side_on:
            speed = max(minimum, int(normal * factor))
        if telemetry.dF >= 0 and telemetry.dF < front_on:
            speed = max(minimum, int(speed * factor))
        return speed

    def _drive_yaw_centered(
        self,
        telemetry: Telemetry,
        speed: int,
        direction: str = "F",
        wall_avoid: bool = True,
    ) -> None:
        steer = self._yaw_steer_norm(telemetry)
        if wall_avoid:
            steer += self._wall_bias(telemetry)
        steer = clamp(steer, -1.0, 1.0)
        if direction.upper() == "B":
            self.motion.drive_backward(steer, speed)
        else:
            self.motion.drive_forward(steer, speed)
        self._continuous_commanded_speed = speed
        self._continuous_direction = direction.upper()

    def _check_continuous_stall(self, telemetry: Telemetry, now: float) -> None:
        if self._continuous_commanded_speed < int(self.stuck_cfg.get("minimum_command_speed", 20)):
            self._continuous_encoder_ref = telemetry.enc_cm
            self._continuous_progress_at = now
            return
        delta_required = float(self.stuck_cfg.get("minimum_progress_cm", 0.6))
        if abs(telemetry.enc_cm - self._continuous_encoder_ref) >= delta_required:
            self._continuous_encoder_ref = telemetry.enc_cm
            self._continuous_progress_at = now
            return
        grace = float(self.stuck_cfg.get("state_entry_grace_s", 0.5))
        freeze_s = float(self.stuck_cfg.get("freeze_s", 1.2))
        if now - self.state_entered_at < grace:
            return
        if now - self._continuous_progress_at >= freeze_s:
            resume = self.state
            clear_direction = "B" if self._continuous_direction == "F" else "F"
            self._trigger_recovery(
                telemetry,
                f"continuous encoder freeze in {self.state.name}",
                resume,
                clear_direction,
            )

    def _trigger_recovery(
        self,
        telemetry: Telemetry,
        reason: str,
        resume_state: State,
        clear_direction: str,
    ) -> None:
        attempts = self.recovery_attempts.get(resume_state, 0) + 1
        self.recovery_attempts[resume_state] = attempts
        max_attempts = int(self.stuck_cfg.get("max_attempts_per_state", 2))
        if attempts > max_attempts:
            self._fault(telemetry, f"Recovery attempts exceeded for {resume_state.name}: {reason}")
            return
        self.motion.cancel()
        self.recovery_reason = reason
        self.recovery_resume_state = resume_state
        self.recovery_clear_direction = clear_direction.upper()
        self._set_state(State.RECOVERY_CLEAR, telemetry, reason)

    def _fault(self, telemetry: Telemetry, reason: str) -> None:
        self.fault_reason = reason
        LOG.error("MISSION FAULT: %s", reason)
        self._set_state(State.FAULT, telemetry, reason)
