# Copyright 2026. RFID DimOS integration.

"""Load repo-root ``.env`` into ``os.environ`` without overriding exports."""

from __future__ import annotations

import os
from pathlib import Path


def load_repo_dotenv(*, override: bool = False) -> Path | None:
    """Load ``<repo>/.env`` if present. Returns the path loaded, or ``None``.

    Existing environment variables win unless ``override=True``.
    """
    env_path = Path(__file__).resolve().parents[1] / ".env"
    if not env_path.is_file():
        return None
    try:
        from dotenv import load_dotenv
    except ImportError:
        for line in env_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, val = line.partition("=")
            key = key.strip()
            val = val.strip().strip("'").strip('"')
            if not key:
                continue
            if override or key not in os.environ:
                os.environ[key] = val
        return env_path
    load_dotenv(env_path, override=override)
    return env_path
