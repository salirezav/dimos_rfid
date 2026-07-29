# Copyright 2026. Shared collection-round context between DimOS worker processes.

"""File-based context so the collection orchestrator can stamp the recorder.

DimOS modules usually run in separate processes, so in-memory object references
do not work. Both sides read/write a small JSON file under XDG_RUNTIME_DIR.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

CONTEXT_ENV = "RFID_COLLECTION_CONTEXT_FILE"


def context_path() -> Path:
    override = os.environ.get(CONTEXT_ENV, "").strip()
    if override:
        return Path(override).expanduser().resolve()
    runtime = os.environ.get("XDG_RUNTIME_DIR", "/tmp")
    return Path(runtime) / "dimos_rfid_collection_context.json"


def write_collection_context(context: dict[str, Any] | None) -> Path:
    path = context_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    if not context:
        if path.is_file():
            path.unlink(missing_ok=True)
        return path
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(context, indent=2), encoding="utf-8")
    tmp.replace(path)
    return path


def read_collection_context() -> dict[str, Any]:
    path = context_path()
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}
