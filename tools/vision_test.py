#!/usr/bin/env python3
"""Run the exact mission vision pipeline without sending any motor commands."""
from __future__ import annotations
import argparse, sys, time
from pathlib import Path
import cv2 as cv
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
from robot.camera import OneCamera
from robot.config import load_yaml
from robot.vision import VisionEngine

parser = argparse.ArgumentParser()
parser.add_argument("--vision", default=str(PROJECT_ROOT / "config" / "vision.yaml"))
args = parser.parse_args()
cfg = load_yaml(args.vision)
cam = OneCamera(cfg["camera"]); engine = VisionEngine(cfg)
cam.start(); cv.namedWindow("vision_test", cv.WINDOW_NORMAL); last=-1
try:
    while True:
        packet=cam.latest(copy_frame=True)
        if packet is None or packet.sequence==last:
            time.sleep(0.01); continue
        last=packet.sequence
        obs=engine.process(packet.image, packet.timestamp)
        vis=engine.draw_debug(packet.image, obs)
        text=(f"line={obs.line.color or '-'} {obs.line.confidence:.2f} "
              f"pillar={obs.pillar.color or '-'} {obs.pillar.confidence:.2f} "
              f"quality={obs.image_quality:.2f}")
        cv.putText(vis,text,(10,25),cv.FONT_HERSHEY_SIMPLEX,0.55,(0,255,0),2)
        cv.imshow("vision_test",vis)
        if cv.waitKey(1)&0xFF in (ord('q'),ord('Q')): break
finally:
    cam.stop(); cv.destroyAllWindows()
