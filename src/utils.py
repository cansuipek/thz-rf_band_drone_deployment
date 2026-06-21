"""
Utility functions for loading configs and saving lightweight run summaries.
"""

from __future__ import annotations
import json
import math
from pathlib import Path
from typing import Any, Mapping
import yaml

def load_config(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def parse_time_limit(value: Any) -> float:
    if value is None:
        return math.inf
    if isinstance(value, str):
        if value.strip().lower() in {"inf", "infinity", "none", "null"}:
            return math.inf
        return float(value)
    return float(value)

def ensure_dir(path: str | Path) -> Path:
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    return path

def stringify_tuple_keys(obj: Any) -> Any:
    """Convert tuple-keyed dictionaries into JSON-safe dictionaries."""
    if isinstance(obj, dict):
        converted = {}
        for key, value in obj.items():
            if isinstance(key, tuple):
                key = "_".join(str(part) for part in key)
            else:
                key = str(key)
            converted[key] = stringify_tuple_keys(value)
        return converted
    if isinstance(obj, (list, tuple)):
        return [stringify_tuple_keys(item) for item in obj]
    return obj

def save_json(data: Mapping[str, Any], path: str | Path) -> None:
    path = Path(path)
    ensure_dir(path.parent)
    with path.open("w", encoding="utf-8") as f:
        json.dump(stringify_tuple_keys(dict(data)), f, indent=2)
