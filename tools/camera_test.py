#!/usr/bin/env python3
"""Open the configured one camera, display FPS, and optionally save snapshots."""
from __future__ import annotations
import argparse, sys, time
from pathlib import Path
import cv2 as cv
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
from robot.camera import OneCamera
from robot.config import load_yaml

parser = argparse.ArgumentParser()
parser.add_argument("--vision", default=str(PROJECT_ROOT / "config" / "vision.yaml"))
args = parser.parse_args()
cfg = load_yaml(args.vision)
cam = OneCamera(cfg["camera"])
cam.start()
cv.namedWindow("camera_test", cv.WINDOW_NORMAL)
count = 0; t0 = time.time(); last = -1
try:
    while True:
        packet = cam.latest(copy_frame=True)
        if packet is None or packet.sequence == last:
            time.sleep(0.01); continue
        last = packet.sequence; count += 1
        elapsed = max(0.001, time.time() - t0)
        fps = count / elapsed
        cv.putText(packet.image, f"FPS {fps:.1f}  S=snapshot Q=quit", (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0), 2)
        cv.imshow("camera_test", packet.image)
        key = cv.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q')): break
        if key in (ord('s'), ord('S')):
            path = PROJECT_ROOT / "logs" / f"camera_{int(time.time())}.png"
            path.parent.mkdir(exist_ok=True)
            cv.imwrite(str(path), packet.image)
            print("Saved", path)
finally:
    cam.stop(); cv.destroyAllWindows()
