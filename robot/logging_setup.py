"""Logging configuration."""
from __future__ import annotations

import logging
from pathlib import Path


def configure_logging(log_dir: str, level: str = "INFO") -> Path:
    target = Path(log_dir).expanduser().resolve()
    target.mkdir(parents=True, exist_ok=True)
    log_path = target / "robot.log"
    numeric_level = getattr(logging, level.upper(), logging.INFO)
    logging.basicConfig(
        level=numeric_level,
        format="%(asctime)s.%(msecs)03d %(levelname)-8s %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler(log_path, encoding="utf-8"),
        ],
        force=True,
    )
    return log_path
