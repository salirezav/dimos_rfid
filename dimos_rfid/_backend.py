# Copyright 2026. RFID DimOS integration.

from __future__ import annotations

import sys
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rfid_service import RfidScanner


def rfid_scanner_python_dir() -> Path:
    """Directory containing rfid_service.py."""
    import os

    if env := os.environ.get("RFID_SCANNER_PYTHON_DIR"):
        return Path(env)

    # Standalone package layout: ../rfid scanner python next to dimos_rfid/
    standalone = Path(__file__).resolve().parent.parent / "rfid scanner python"
    if standalone.is_dir():
        return standalone

    # Typical WSL project layout: ~/projects/Dimos/rfid scanner python
    home_layout = Path.home() / "projects" / "Dimos" / "rfid scanner python"
    if home_layout.is_dir():
        return home_layout

    return standalone


def ensure_rfid_scanner_importable() -> Path:
    """Add the existing RFID API package to sys.path."""
    scanner_dir = rfid_scanner_python_dir()
    if not scanner_dir.is_dir():
        raise FileNotFoundError(
            f"RFID scanner code not found at {scanner_dir}. "
            "Expected the 'rfid scanner python' folder next to dimos_rfid/."
        )
    path = str(scanner_dir)
    if path not in sys.path:
        sys.path.insert(0, path)
    return scanner_dir


def create_direct_scanner(
    *,
    host: str,
    user: str,
    password: str,
    stale_seconds: float,
) -> RfidScanner:
    """Import and construct RfidScanner from the existing API code."""
    ensure_rfid_scanner_importable()
    from rfid_service import RfidScanner, ScannerConfig

    return RfidScanner(
        ScannerConfig(
            host=host,
            user=user,
            password=password,
            stale_seconds=stale_seconds,
        )
    )
