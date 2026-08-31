#!/usr/bin/env python3
"""Main entry point for the one-camera robot."""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from pathlib import Path

import cv2 as cv

# Support both `python -m app.robot_runner` and direct execution.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.camera import OneCamera
from robot.config import env_flag, load_project_config
from robot.logging_setup import configure_logging
from robot.mission import MissionController, State
from robot.motion import MotionController
from robot.serial_link import SerialLink
from robot.types import VisionObservation
from robot.vision import VisionEngine

LOG = logging.getLogger("robot_runner")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="One-camera Jetson robot mission")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config" / "robot.yaml"))
    parser.add_argument("--vision", default=str(PROJECT_ROOT / "config" / "vision.yaml"))
    parser.add_argument("--headless", action="store_true", help="Disable OpenCV windows")
    parser.add_argument("--dry-run", action="store_true", help="Log commands without opening Arduino serial")
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def restart_process(reason: str, camera: OneCamera, link: SerialLink, motion: MotionController) -> None:
    LOG.warning("Restarting Python process: %s", reason)
    try:
        motion.stop()
    except Exception:
        pass
    try:
        camera.stop()
    except Exception:
        pass
    try:
        link.stop()
    except Exception:
        pass
    try:
        cv.destroyAllWindows()
    except Exception:
        pass
    os.execv(sys.executable, [sys.executable, *sys.argv])


def put_hud(frame, mission: MissionController, observation: VisionObservation, telemetry) -> None:
    status = mission.status()
    lines = [
        f"STATE {status['state']}  yaw={telemetry.yaw:6.1f}  ref={status['yaw_ref']:6.1f}",
        f"turns={status['turn_count']} idx={status['corridor_index']} enc={telemetry.enc_cm:6.1f}cm",
        f"US F/L/R/B={telemetry.dF:.0f}/{telemetry.dL:.0f}/{telemetry.dR:.0f}/{telemetry.dB:.0f}",
        f"LINE {observation.line.color or '-'} {observation.line.confidence:.2f}  "
        f"PILLAR {observation.pillar.color or '-'} {observation.pillar.confidence:.2f}",
        f"PIX B/O/R/G={observation.pixel_counts.get('BLUE',0)}/"
        f"{observation.pixel_counts.get('ORANGE',0)}/"
        f"{observation.pixel_counts.get('RED',0)}/"
        f"{observation.pixel_counts.get('GREEN',0)}",
        f"image quality={observation.image_quality:.2f} blur={observation.blur_score:.0f}",
    ]
    if status["fault_reason"]:
        lines.append(f"FAULT: {status['fault_reason']}")
    for index, text in enumerate(lines):
        cv.putText(
            frame,
            text,
            (10, 24 + index * 22),
            cv.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0) if index < 3 else (255, 255, 255),
            2,
        )


def main() -> int:
    args = parse_args()
    robot_cfg, vision_cfg = load_project_config(args.config, args.vision)
    runtime_cfg = robot_cfg.get("runtime", {})
    log_dir = str(runtime_cfg.get("log_dir", PROJECT_ROOT / "logs"))
    configure_logging(log_dir, args.log_level)

    headless = args.headless or env_flag("ROBOT_HEADLESS", bool(runtime_cfg.get("headless", False)))
    dry_run = args.dry_run
    LOG.info("Robot config: %s", robot_cfg.get("_source_path"))
    LOG.info("Vision config: %s", vision_cfg.get("_source_path"))
    LOG.info("Mode: headless=%s dry_run=%s", headless, dry_run)

    link = SerialLink(robot_cfg.get("serial", {}), dry_run=dry_run)
    camera = OneCamera(vision_cfg.get("camera", {}))
    motion = MotionController(link, robot_cfg, dry_run=dry_run)
    vision = VisionEngine(vision_cfg)
    mission = MissionController(robot_cfg, motion)

    link.start()
    camera.start()

    if not headless:
        cv.namedWindow("robot", cv.WINDOW_NORMAL)

    control_hz = float(runtime_cfg.get("control_hz", 50.0))
    control_period = 1.0 / max(5.0, control_hz)
    vision_hz = float(runtime_cfg.get("vision_hz", 20.0))
    vision_period = 1.0 / max(1.0, vision_hz)
    camera_stale_s = float(runtime_cfg.get("camera_stale_s", 2.0))
    restart_on_boot = bool(runtime_cfg.get("restart_on_arduino_boot", True))

    latest_observation = VisionObservation()
    last_vision_time = 0.0
    last_frame_sequence = -1
    last_good_frame_time = time.time()
    events_path = Path(log_dir).expanduser().resolve() / "events.jsonl"

    try:
        with events_path.open("a", encoding="utf-8") as events:
            while True:
                loop_started = time.time()
                link.tick()

                if link.consume_boot_change():
                    if restart_on_boot:
                        restart_process("Arduino boot ID changed", camera, link, motion)
                    else:
                        mission.hard_reset(link.telemetry())

                if link.consume_armed():
                    mission.request_start()

                telemetry = link.telemetry()
                packet = camera.latest(copy_frame=True)
                frame = None
                if packet is not None:
                    frame = packet.image
                    last_good_frame_time = packet.timestamp
                    if packet.sequence != last_frame_sequence and loop_started - last_vision_time >= vision_period:
                        latest_observation = vision.process(frame, packet.timestamp)
                        last_frame_sequence = packet.sequence
                        last_vision_time = loop_started

                if loop_started - last_good_frame_time > camera_stale_s:
                    LOG.error("Camera stale for %.1fs; stopping and returning to WAIT_START", loop_started - last_good_frame_time)
                    motion.stop()
                    mission.hard_reset(telemetry)
                    last_good_frame_time = loop_started

                mission.step(telemetry, latest_observation, loop_started)

                event = {
                    "t": loop_started,
                    "state": mission.state.name,
                    "yaw": telemetry.yaw,
                    "yaw_ref": mission.yaw_ref,
                    "enc_cm": telemetry.enc_cm,
            "enc_ticks": telemetry.enc_ticks,
                    "dF": telemetry.dF,
                    "dL": telemetry.dL,
                    "dR": telemetry.dR,
                    "dB": telemetry.dB,
                    "line": latest_observation.line.color,
                    "line_conf": latest_observation.line.confidence,
                    "pillar": latest_observation.pillar.color,
                    "pillar_conf": latest_observation.pillar.confidence,
                }
                events.write(json.dumps(event, separators=(",", ":")) + "\n")
                if int(loop_started * 10) % 10 == 0:
                    events.flush()

                if not headless and frame is not None:
                    display = vision.draw_debug(frame, latest_observation)
                    put_hud(display, mission, latest_observation, telemetry)
                    cv.imshow("robot", display)
                    key = cv.waitKey(1) & 0xFF
                    if key in (ord("q"), ord("Q")):
                        break
                    if key in (ord("s"), ord("S")):
                        mission.request_start()
                    if key in (ord("x"), ord("X")):
                        restart_process("manual X key", camera, link, motion)

                elapsed = time.time() - loop_started
                if elapsed < control_period:
                    time.sleep(control_period - elapsed)
    except KeyboardInterrupt:
        LOG.info("Keyboard interrupt")
    finally:
        try:
            motion.stop()
        except Exception:
            pass
        camera.stop()
        link.stop()
        cv.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
