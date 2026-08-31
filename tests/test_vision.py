import cv2 as cv
import numpy as np
from robot.config import load_yaml
from robot.vision import VisionEngine


def test_synthetic_blue_line(tmp_path):
    cfg = load_yaml("config/vision.yaml")
    cfg["colors"]["BLUE"]["combine"] = "union"
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:] = (80, 80, 80)
    cv.rectangle(frame, (180, 390), (460, 455), (255, 0, 0), -1)
    engine = VisionEngine(cfg)
    obs = None
    for i in range(6):
        obs = engine.process(frame, float(i))
    assert obs is not None
    assert obs.pixel_counts["BLUE"] > 500
