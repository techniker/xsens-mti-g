"""Persist user settings to a JSON file next to the executable."""

import json
import os

_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")

_DEFAULTS = {
    "metric": False,
    "p0_pa": 101325.0,
    "spd_bands_enabled": True,
    "spd_vso": 40,
    "spd_vs1": 48,
    "spd_vfe": 85,
    "spd_vno": 129,
    "spd_vne": 163,
    "auto_zero_on_start": True,
    "hdg_bug": 0,
    "alt_source": "baro",  # "baro" or "gnss"
}


def load() -> dict:
    """Load saved settings, falling back to defaults for missing keys."""
    cfg = dict(_DEFAULTS)
    try:
        with open(_CONFIG_PATH, "r") as f:
            saved = json.load(f)
        cfg.update({k: saved[k] for k in saved if k in _DEFAULTS})
    except (FileNotFoundError, json.JSONDecodeError, KeyError):
        pass
    return cfg


def save(cfg: dict):
    """Write current settings to disk."""
    with open(_CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
