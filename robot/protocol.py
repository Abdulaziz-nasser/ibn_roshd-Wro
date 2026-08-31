"""Jetson <-> Arduino line protocol parsing and math helpers."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any

from .types import MotionResult, Telemetry


def wrap180(angle: float) -> float:
    value = float(angle)
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def shortest_error(target: float, current: float) -> float:
    return wrap180(float(target) - float(current))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_key_values(parts: list[str]) -> dict[str, str]:
    values: dict[str, str] = {}
    for token in parts:
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def _as_float(values: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(values.get(key, default))
    except (TypeError, ValueError):
        return default


def _as_int(values: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(values.get(key, default)))
    except (TypeError, ValueError):
        return default


@dataclass(slots=True)
class ParsedLine:
    kind: str
    values: dict[str, Any]


def parse_line(line: str) -> ParsedLine:
    text = line.strip()
    if not text:
        return ParsedLine("EMPTY", {})
    parts = [part.strip() for part in text.split(",")]
    kind = parts[0].upper()
    kv = parse_key_values(parts[1:])

    if kind == "TLM":
        telemetry = Telemetry(
            timestamp=time.time(),
            boot_id=_as_int(kv, "boot"),
            yaw=_as_float(kv, "yaw"),
            state=kv.get("state", "UNKNOWN"),
            steer=_as_float(kv, "steer"),
            speed=_as_int(kv, "speed"),
            dF=_as_float(kv, "dF", -1.0),
            dL=_as_float(kv, "dL", -1.0),
            dR=_as_float(kv, "dR", -1.0),
            dB=_as_float(kv, "dB", -1.0),
            enc_cm=_as_float(kv, "enc_cm"),
            enc_ticks=_as_int(kv, "enc_ticks", 0),
            armed=bool(_as_int(kv, "armed", 0)),
        )
        return ParsedLine("TLM", {"telemetry": telemetry})

    if kind == "DONE":
        result = MotionResult(
            command_id=_as_int(kv, "id", -1),
            result=kv.get("result", "UNKNOWN").upper(),
            progress_cm=_as_float(kv, "progress_cm", 0.0),
            detail=kv.get("detail", ""),
            timestamp=time.time(),
        )
        return ParsedLine("DONE", {"result": result})

    converted: dict[str, Any] = dict(kv)
    for int_key in ("boot", "id"):
        if int_key in kv:
            converted[int_key] = _as_int(kv, int_key)
    return ParsedLine(kind, converted)


def format_command(name: str, **values: Any) -> bytes:
    tokens = [name.upper()]
    for key, value in values.items():
        if value is None:
            continue
        if isinstance(value, float):
            if math.isfinite(value):
                tokens.append(f"{key}={value:.4f}")
        else:
            tokens.append(f"{key}={value}")
    return (",".join(tokens) + "\n").encode("ascii", "strict")
