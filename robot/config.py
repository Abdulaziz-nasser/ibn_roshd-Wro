"""Configuration helpers for the one-camera robot project."""
from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any

import yaml


class ConfigError(RuntimeError):
    """Raised when a configuration file is missing or malformed."""


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def load_yaml(path: str | os.PathLike[str]) -> dict[str, Any]:
    file_path = Path(path).expanduser().resolve()
    if not file_path.is_file():
        raise ConfigError(f"Configuration file not found: {file_path}")
    try:
        with file_path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except yaml.YAMLError as exc:
        raise ConfigError(f"Invalid YAML in {file_path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"Top-level YAML value must be a mapping: {file_path}")
    data["_source_path"] = str(file_path)
    return data


def load_project_config(
    robot_path: str | os.PathLike[str],
    vision_path: str | os.PathLike[str],
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Load the motion/mission configuration and shared vision configuration."""
    robot_cfg = load_yaml(robot_path)
    vision_cfg = load_yaml(vision_path)
    return robot_cfg, vision_cfg


def env_flag(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def get_nested(config: dict[str, Any], dotted_path: str, default: Any = None) -> Any:
    value: Any = config
    for part in dotted_path.split("."):
        if not isinstance(value, dict) or part not in value:
            return default
        value = value[part]
    return value


def require_nested(config: dict[str, Any], dotted_path: str) -> Any:
    value = get_nested(config, dotted_path, default=None)
    if value is None:
        source = config.get("_source_path", "<configuration>")
        raise ConfigError(f"Missing required setting '{dotted_path}' in {source}")
    return value


def dump_yaml(config: dict[str, Any], path: str | os.PathLike[str]) -> None:
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    clean = {k: v for k, v in config.items() if not k.startswith("_")}
    with target.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(clean, handle, sort_keys=False, allow_unicode=True)
