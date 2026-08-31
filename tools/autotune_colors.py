#!/usr/bin/env python3
"""Interactive color autotune using the exact same camera and vision code as main."""
from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

import cv2 as cv
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.camera import OneCamera
from robot.config import dump_yaml, load_yaml
from robot.vision import VisionEngine


class Selector:
    def __init__(self) -> None:
        self.dragging = False
        self.start = (0, 0)
        self.end = (0, 0)
        self.valid = False

    def callback(self, event, x, y, flags, userdata) -> None:
        if event == cv.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.start = (x, y)
            self.end = (x, y)
            self.valid = False
        elif event == cv.EVENT_MOUSEMOVE and self.dragging:
            self.end = (x, y)
        elif event == cv.EVENT_LBUTTONUP:
            self.dragging = False
            self.end = (x, y)
            self.valid = True

    def rectangle(self, shape) -> tuple[int, int, int, int] | None:
        if not (self.valid or self.dragging):
            return None
        h, w = shape[:2]
        x0, x1 = sorted((max(0, self.start[0]), min(w - 1, self.end[0])))
        y0, y1 = sorted((max(0, self.start[1]), min(h - 1, self.end[1])))
        if x1 - x0 < 4 or y1 - y0 < 4:
            return None
        return x0, y0, x1, y1

    def clear(self) -> None:
        self.valid = False
        self.dragging = False


def circular_hue_ranges(hues: np.ndarray, coverage: float = 0.98, padding: int = 3) -> list[tuple[int, int]]:
    values = np.sort(hues.astype(np.int16).reshape(-1))
    if values.size == 0:
        return [(0, 179)]
    count = max(1, int(round(values.size * coverage)))
    extended = np.concatenate([values, values + 180])
    best_start = 0
    best_width = 999
    for index in range(values.size):
        end_index = index + count - 1
        width = int(extended[end_index] - extended[index])
        if width < best_width:
            best_width = width
            best_start = int(extended[index])
    start = (best_start - padding) % 180
    end = (best_start + best_width + padding) % 180
    if start <= end:
        return [(int(start), int(end))]
    return [(0, int(end)), (int(start), 179)]


def learn_color_model(sample_bgr: np.ndarray) -> dict:
    hsv = cv.cvtColor(sample_bgr, cv.COLOR_BGR2HSV)
    lab = cv.cvtColor(sample_bgr, cv.COLOR_BGR2LAB)
    pixels_hsv = hsv.reshape(-1, 3)
    pixels_lab = lab.reshape(-1, 3).astype(np.float64)

    # Reject nearly grey/black pixels accidentally included around the target.
    keep = (pixels_hsv[:, 1] >= 35) & (pixels_hsv[:, 2] >= 25)
    if int(np.count_nonzero(keep)) >= 50:
        pixels_hsv = pixels_hsv[keep]
        pixels_lab = pixels_lab[keep]

    hue_ranges = circular_hue_ranges(pixels_hsv[:, 0], coverage=0.98, padding=3)
    s_low = max(0, int(np.percentile(pixels_hsv[:, 1], 2)) - 12)
    s_high = min(255, int(np.percentile(pixels_hsv[:, 1], 99)) + 12)
    v_low = max(0, int(np.percentile(pixels_hsv[:, 2], 2)) - 15)
    v_high = min(255, int(np.percentile(pixels_hsv[:, 2], 99)) + 15)

    hsv_ranges = [
        {"low": [h0, s_low, v_low], "high": [h1, s_high, v_high]}
        for h0, h1 in hue_ranges
    ]

    mean = np.mean(pixels_lab, axis=0)
    covariance = np.cov(pixels_lab, rowvar=False)
    if covariance.shape != (3, 3):
        covariance = np.eye(3, dtype=np.float64) * 25.0
    covariance += np.eye(3, dtype=np.float64) * 4.0

    return {
        "combine": "intersection",
        "hsv": hsv_ranges,
        "lab": {
            "mean": [round(float(v), 4) for v in mean],
            "covariance": [[round(float(v), 4) for v in row] for row in covariance],
            "max_mahalanobis": 4.5,
        },
    }


def roi_to_fractions(rect: tuple[int, int, int, int], shape) -> list[float]:
    h, w = shape[:2]
    x0, y0, x1, y1 = rect
    return [round(x0 / w, 5), round(y0 / h, 5), round(x1 / w, 5), round(y1 / h, 5)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vision", default=str(PROJECT_ROOT / "config" / "vision.yaml"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    vision_path = Path(args.vision).expanduser().resolve()
    config = load_yaml(vision_path)
    camera = OneCamera(config.get("camera", {}))
    engine = VisionEngine(config)
    selector = Selector()

    camera.start()
    cv.namedWindow("autotune", cv.WINDOW_NORMAL)
    cv.setMouseCallback("autotune", selector.callback)
    print("Drag a tight rectangle over a real target color.")
    print("1=BLUE  2=ORANGE  3=RED  4=GREEN  L=line ROI  P=pillar ROI")
    print("S=save  C=clear selection  Q=quit")

    last_sequence = -1
    try:
        while True:
            packet = camera.latest(copy_frame=True)
            if packet is None or packet.sequence == last_sequence:
                time.sleep(0.01)
                continue
            last_sequence = packet.sequence
            frame = packet.image
            observation = engine.process(frame, packet.timestamp)
            vis = engine.draw_debug(frame, observation)
            rect = selector.rectangle(frame.shape)
            if rect:
                x0, y0, x1, y1 = rect
                cv.rectangle(vis, (x0, y0), (x1, y1), (255, 255, 255), 2)

            instructions = [
                "Drag sample | 1 BLUE 2 ORANGE 3 RED 4 GREEN",
                "L line ROI | P pillar ROI | S save | C clear | Q quit",
                f"Detected line={observation.line.color or '-'} {observation.line.confidence:.2f} "
                f"pillar={observation.pillar.color or '-'} {observation.pillar.confidence:.2f}",
            ]
            for i, text in enumerate(instructions):
                cv.putText(vis, text, (10, 24 + 22 * i), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv.imshow("autotune", vis)
            key = cv.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("c"), ord("C")):
                selector.clear()
                continue
            if key in (ord("1"), ord("2"), ord("3"), ord("4")):
                if rect is None:
                    print("No selection. Drag a rectangle first.")
                    continue
                color = {ord("1"): "BLUE", ord("2"): "ORANGE", ord("3"): "RED", ord("4"): "GREEN"}[key]
                x0, y0, x1, y1 = rect
                sample = frame[y0:y1, x0:x1]
                config.setdefault("colors", {})[color] = learn_color_model(sample)
                engine.update_config(config)
                print(f"Learned {color} from {sample.shape[1]}x{sample.shape[0]} sample")
                continue
            if key in (ord("l"), ord("L")):
                if rect is None:
                    print("No selection. Drag the complete floor-line ROI first.")
                else:
                    config.setdefault("line", {})["roi"] = roi_to_fractions(rect, frame.shape)
                    engine.update_config(config)
                    print("Line ROI updated:", config["line"]["roi"])
                continue
            if key in (ord("p"), ord("P")):
                if rect is None:
                    print("No selection. Drag the complete pillar ROI first.")
                else:
                    config.setdefault("pillar", {})["roi"] = roi_to_fractions(rect, frame.shape)
                    engine.update_config(config)
                    print("Pillar ROI updated:", config["pillar"]["roi"])
                continue
            if key in (ord("s"), ord("S")):
                backup = vision_path.with_suffix(vision_path.suffix + ".bak")
                if vision_path.exists():
                    shutil.copy2(vision_path, backup)
                dump_yaml(config, vision_path)
                print(f"Saved {vision_path}; backup: {backup}")
    finally:
        camera.stop()
        cv.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
