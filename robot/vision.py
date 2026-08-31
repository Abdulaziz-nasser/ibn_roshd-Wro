"""Shared one-camera vision pipeline used by both autotune and the mission."""
from __future__ import annotations

import copy
import logging
import math
from dataclasses import dataclass
from typing import Any

import cv2 as cv
import numpy as np

from .protocol import clamp
from .types import Detection, VisionObservation

LOG = logging.getLogger(__name__)


def roi_from_fractions(frame: np.ndarray, fractions: list[float]) -> tuple[np.ndarray, tuple[int, int, int, int]]:
    h, w = frame.shape[:2]
    x0f, y0f, x1f, y1f = [float(v) for v in fractions]
    x0 = int(clamp(x0f, 0.0, 1.0) * w)
    y0 = int(clamp(y0f, 0.0, 1.0) * h)
    x1 = int(clamp(x1f, 0.0, 1.0) * w)
    y1 = int(clamp(y1f, 0.0, 1.0) * h)
    x0, x1 = sorted((max(0, x0), min(w, x1)))
    y0, y1 = sorted((max(0, y0), min(h, y1)))
    if x1 <= x0 or y1 <= y0:
        raise ValueError(f"Invalid ROI fractions: {fractions}")
    return frame[y0:y1, x0:x1], (x0, y0, x1, y1)


def mask_hsv_ranges(hsv: np.ndarray, ranges: Any) -> np.ndarray:
    if isinstance(ranges, dict) and "low" in ranges and "high" in ranges:
        ranges = [ranges]
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for item in ranges if isinstance(ranges, list) else []:
        if not isinstance(item, dict) or "low" not in item or "high" not in item:
            continue
        low = np.array(item["low"], dtype=np.uint8)
        high = np.array(item["high"], dtype=np.uint8)
        mask = cv.bitwise_or(mask, cv.inRange(hsv, low, high))
    return mask


def _regularized_inverse(covariance: np.ndarray) -> np.ndarray:
    cov = np.asarray(covariance, dtype=np.float64).reshape(3, 3)
    cov = cov + np.eye(3, dtype=np.float64) * 1.0
    return np.linalg.pinv(cov)


def lab_distance_mask(lab: np.ndarray, model: dict[str, Any]) -> tuple[np.ndarray, np.ndarray] | None:
    mean = model.get("mean")
    covariance = model.get("covariance")
    if not (isinstance(mean, list) and len(mean) == 3 and isinstance(covariance, list)):
        return None
    try:
        inv_cov = _regularized_inverse(np.asarray(covariance, dtype=np.float64))
    except Exception:
        return None
    delta = lab.astype(np.float64) - np.asarray(mean, dtype=np.float64)
    d2 = np.einsum("...i,ij,...j->...", delta, inv_cov, delta, optimize=True)
    max_distance = float(model.get("max_mahalanobis", 4.0))
    mask = np.where(d2 <= max_distance * max_distance, 255, 0).astype(np.uint8)
    confidence = np.exp(-0.5 * np.clip(d2, 0.0, 40.0)).astype(np.float32)
    return mask, confidence


def clean_mask(mask: np.ndarray, processing: dict[str, Any]) -> np.ndarray:
    result = mask
    erode_iter = max(0, int(processing.get("erode_iter", 0)))
    dilate_iter = max(0, int(processing.get("dilate_iter", 1)))
    close_kernel = max(0, int(processing.get("close_kernel", 5)))
    open_kernel = max(0, int(processing.get("open_kernel", 0)))
    if erode_iter:
        result = cv.erode(result, np.ones((3, 3), np.uint8), iterations=erode_iter)
    if open_kernel >= 2:
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (open_kernel, open_kernel))
        result = cv.morphologyEx(result, cv.MORPH_OPEN, kernel)
    if close_kernel >= 2:
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (close_kernel, close_kernel))
        result = cv.morphologyEx(result, cv.MORPH_CLOSE, kernel)
    if dilate_iter:
        result = cv.dilate(result, np.ones((3, 3), np.uint8), iterations=dilate_iter)
    return result


@dataclass(slots=True)
class BlobCandidate:
    contour: np.ndarray
    bbox: tuple[int, int, int, int]
    area: float
    solidity: float
    aspect: float
    center_x_frac: float
    center_y_frac: float


def best_blob(mask: np.ndarray, min_area: float) -> BlobCandidate | None:
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    best: BlobCandidate | None = None
    roi_h, roi_w = mask.shape[:2]
    for contour in contours:
        contour_area = float(cv.contourArea(contour))
        if contour_area < min_area:
            continue
        x, y, w, h = cv.boundingRect(contour)
        if w <= 0 or h <= 0:
            continue
        hull_area = float(cv.contourArea(cv.convexHull(contour)))
        solidity = contour_area / hull_area if hull_area > 1.0 else 0.0
        aspect = float(max(w, h)) / float(max(1, min(w, h)))
        candidate = BlobCandidate(
            contour=contour,
            bbox=(x, y, w, h),
            area=contour_area,
            solidity=solidity,
            aspect=aspect,
            center_x_frac=(x + w / 2.0) / max(1.0, roi_w),
            center_y_frac=(y + h / 2.0) / max(1.0, roi_h),
        )
        if best is None or candidate.area > best.area:
            best = candidate
    return best


class ConfidenceEMA:
    def __init__(self, alpha_up: float = 0.55, alpha_down: float = 0.25) -> None:
        self.alpha_up = alpha_up
        self.alpha_down = alpha_down
        self.values: dict[str, float] = {}

    def update(self, raw: dict[str, float]) -> dict[str, float]:
        all_keys = set(self.values) | set(raw)
        updated: dict[str, float] = {}
        for key in all_keys:
            previous = self.values.get(key, 0.0)
            target = raw.get(key, 0.0)
            alpha = self.alpha_up if target >= previous else self.alpha_down
            value = (1.0 - alpha) * previous + alpha * target
            updated[key] = float(clamp(value, 0.0, 1.0))
        self.values = updated
        return dict(updated)

    def reset(self) -> None:
        self.values.clear()


class VisionEngine:
    """Detect BLUE/ORANGE floor lines and RED/GREEN pillars from one frame."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = copy.deepcopy(config)
        temporal = self.config.get("temporal", {})
        self.line_ema = ConfidenceEMA(
            float(temporal.get("alpha_up", 0.55)),
            float(temporal.get("alpha_down", 0.25)),
        )
        self.pillar_ema = ConfidenceEMA(
            float(temporal.get("alpha_up", 0.55)),
            float(temporal.get("alpha_down", 0.25)),
        )

    def update_config(self, config: dict[str, Any]) -> None:
        self.config = copy.deepcopy(config)
        self.line_ema.reset()
        self.pillar_ema.reset()

    def _color_mask(
        self,
        hsv: np.ndarray,
        lab: np.ndarray,
        color_name: str,
        processing: dict[str, Any],
    ) -> tuple[np.ndarray, np.ndarray | None]:
        color_cfg = self.config.get("colors", {}).get(color_name, {})
        hsv_mask = mask_hsv_ranges(hsv, color_cfg.get("hsv", []))
        lab_result = lab_distance_mask(lab, color_cfg.get("lab", {}))
        lab_confidence: np.ndarray | None = None
        if lab_result is None:
            combined = hsv_mask
        else:
            lab_mask, lab_confidence = lab_result
            mode = str(color_cfg.get("combine", "intersection")).lower()
            if mode == "union":
                combined = cv.bitwise_or(hsv_mask, lab_mask)
            else:
                combined = cv.bitwise_and(hsv_mask, lab_mask)
        return clean_mask(combined, processing), lab_confidence

    @staticmethod
    def _quality(frame: np.ndarray) -> tuple[float, float, float, float]:
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        brightness = float(np.mean(hsv[:, :, 2]))
        saturation = float(np.mean(hsv[:, :, 1]))
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        blur_score = float(cv.Laplacian(gray, cv.CV_64F).var())
        bright_score = 1.0 - min(1.0, abs(brightness - 130.0) / 130.0)
        blur_quality = min(1.0, blur_score / 120.0)
        quality = float(clamp(0.65 * bright_score + 0.35 * blur_quality, 0.0, 1.0))
        return brightness, saturation, blur_score, quality

    def _detect_group(
        self,
        frame: np.ndarray,
        group_cfg: dict[str, Any],
        color_names: list[str],
        kind: str,
    ) -> tuple[Detection, dict[str, int], dict[str, Any]]:
        roi, rect = roi_from_fractions(frame, group_cfg.get("roi", [0.0, 0.0, 1.0, 1.0]))
        blur_kernel = int(group_cfg.get("blur_kernel", 3))
        if blur_kernel >= 3:
            blur_kernel |= 1
            processed = cv.GaussianBlur(roi, (blur_kernel, blur_kernel), 0)
        else:
            processed = roi
        hsv = cv.cvtColor(processed, cv.COLOR_BGR2HSV)
        lab = cv.cvtColor(processed, cv.COLOR_BGR2LAB)
        processing = group_cfg.get("processing", {})
        min_area = float(group_cfg.get("min_area", 500.0))
        min_pixels = int(group_cfg.get("min_pixels", 400))
        min_solidity = float(group_cfg.get("min_solidity", 0.25))
        min_aspect = float(group_cfg.get("min_aspect", 1.1))
        expected_vertical = kind == "pillar"

        raw_confidences: dict[str, float] = {}
        candidates: dict[str, BlobCandidate | None] = {}
        pixel_counts: dict[str, int] = {}
        masks: dict[str, np.ndarray] = {}

        for color_name in color_names:
            mask, lab_conf = self._color_mask(hsv, lab, color_name, processing)
            pixels = int(np.count_nonzero(mask))
            candidate = best_blob(mask, min_area)
            masks[color_name] = mask
            pixel_counts[color_name] = pixels
            candidates[color_name] = candidate
            if candidate is None or pixels < min_pixels or candidate.solidity < min_solidity:
                raw_confidences[color_name] = 0.0
                continue

            x, y, w, h = candidate.bbox
            oriented_aspect = (h / max(1.0, w)) if expected_vertical else (w / max(1.0, h))
            shape_score = float(clamp(oriented_aspect / max(1e-6, min_aspect), 0.0, 1.0))
            area_score = float(clamp(candidate.area / max(min_area * 4.0, 1.0), 0.0, 1.0))
            pixel_score = float(clamp(pixels / max(min_pixels * 4.0, 1.0), 0.0, 1.0))
            position_score = candidate.center_y_frac if kind == "line" else 1.0
            lab_score = 0.5
            if lab_conf is not None:
                contour_mask = np.zeros(mask.shape, dtype=np.uint8)
                cv.drawContours(contour_mask, [candidate.contour], -1, 255, thickness=-1)
                values = lab_conf[contour_mask > 0]
                if values.size:
                    lab_score = float(clamp(float(np.mean(values)) * 2.5, 0.0, 1.0))
            raw = (
                0.30 * pixel_score
                + 0.25 * area_score
                + 0.20 * shape_score
                + 0.15 * lab_score
                + 0.10 * position_score
            )
            raw_confidences[color_name] = float(clamp(raw, 0.0, 1.0))

        filtered = self.line_ema.update(raw_confidences) if kind == "line" else self.pillar_ema.update(raw_confidences)
        selected = max(color_names, key=lambda name: filtered.get(name, 0.0))
        confidence = filtered.get(selected, 0.0)
        minimum_confidence = float(group_cfg.get("minimum_confidence", 0.72))
        candidate = candidates.get(selected)

        detection = Detection()
        if candidate is not None and confidence >= minimum_confidence:
            x, y, w, h = candidate.bbox
            roi_x0, roi_y0, _, _ = rect
            detection = Detection(
                color=selected,
                confidence=confidence,
                bbox=(roi_x0 + x, roi_y0 + y, w, h),
                area=candidate.area,
                pixels=pixel_counts.get(selected, 0),
                center_x_frac=candidate.center_x_frac,
                center_y_frac=candidate.center_y_frac,
                aspect=candidate.aspect,
                solidity=candidate.solidity,
            )

        debug = {
            "roi": rect,
            "masks": masks,
            "raw_confidences": raw_confidences,
            "filtered_confidences": filtered,
        }
        return detection, pixel_counts, debug

    def process(self, frame: np.ndarray, timestamp: float) -> VisionObservation:
        brightness, saturation, blur_score, quality = self._quality(frame)
        line_cfg = self.config.get("line", {})
        pillar_cfg = self.config.get("pillar", {})
        line, line_pixels, line_debug = self._detect_group(
            frame, line_cfg, ["BLUE", "ORANGE"], "line"
        )
        pillar, pillar_pixels, pillar_debug = self._detect_group(
            frame, pillar_cfg, ["RED", "GREEN"], "pillar"
        )

        # Reduce trust when the image is very dark, overexposed, or blurred.
        line.confidence *= quality
        pillar.confidence *= quality
        if line.confidence < float(line_cfg.get("minimum_confidence", 0.72)):
            line.color = None
        if pillar.confidence < float(pillar_cfg.get("minimum_confidence", 0.72)):
            pillar.color = None

        return VisionObservation(
            timestamp=timestamp,
            line=line,
            pillar=pillar,
            pixel_counts={**line_pixels, **pillar_pixels},
            brightness=brightness,
            saturation=saturation,
            blur_score=blur_score,
            image_quality=quality,
            debug={"line": line_debug, "pillar": pillar_debug},
        )

    def draw_debug(self, frame: np.ndarray, observation: VisionObservation) -> np.ndarray:
        vis = frame.copy()
        for group_name, color in (("line", (0, 200, 255)), ("pillar", (255, 255, 0))):
            rect = observation.debug.get(group_name, {}).get("roi")
            if rect:
                x0, y0, x1, y1 = rect
                cv.rectangle(vis, (x0, y0), (x1, y1), color, 1)

        for detection, color in ((observation.line, (0, 165, 255)), (observation.pillar, (0, 255, 255))):
            if detection.bbox:
                x, y, w, h = detection.bbox
                cv.rectangle(vis, (x, y), (x + w, y + h), color, 2)
                cv.putText(
                    vis,
                    f"{detection.color} {detection.confidence:.2f}",
                    (x, max(18, y - 5)),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                )
        return vis
