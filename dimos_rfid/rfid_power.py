# Copyright 2026. RFID DimOS integration — HTTP client for Vulcan TX power.

"""Thin HTTP helpers for reader read/write power and sensitivity.

Talks to the dog's ``rfid_scanner_server.py`` endpoints documented in RFID_API.md:

- ``GET/PUT {api_base}/power``
- ``PUT {api_base}/power/read_power?value=…``
"""

from __future__ import annotations

from typing import Any

import requests

# AdvanNet RF_READ_POWER accepted range on Vulcan Titanium (0.5 dBm steps).
DEFAULT_POWER_MIN = 0.0
DEFAULT_POWER_MAX = 31.5
DEFAULT_POWER_STEP = 0.5


class RfidPowerError(RuntimeError):
    """Raised when the RFID HTTP power API fails or rejects a value."""


def _normalize_api_base(api_base: str) -> str:
    return api_base.rstrip("/")


def snap_read_power(
    dbm: float,
    *,
    min_dbm: float = DEFAULT_POWER_MIN,
    max_dbm: float = DEFAULT_POWER_MAX,
    step: float = DEFAULT_POWER_STEP,
) -> float:
    """Clamp and quantize to the reader step size."""
    if step <= 0:
        raise ValueError(f"step must be > 0, got {step}")
    clamped = min(max(float(dbm), float(min_dbm)), float(max_dbm))
    steps = round((clamped - min_dbm) / step)
    return round(min_dbm + steps * step, 1)


def get_power(api_base: str, *, timeout: float = 3.0) -> dict[str, Any]:
    """Return current read/write power and sensitivity from the dog API."""
    url = f"{_normalize_api_base(api_base)}/power"
    try:
        response = requests.get(url, timeout=timeout)
        response.raise_for_status()
        payload = response.json()
    except requests.RequestException as exc:
        raise RfidPowerError(f"GET {url} failed: {exc}") from exc
    if not payload.get("ok", True):
        raise RfidPowerError(f"GET {url} returned error: {payload}")
    return payload


def set_read_power(
    api_base: str,
    read_power_dbm: float,
    *,
    timeout: float = 5.0,
    snap: bool = True,
) -> dict[str, Any]:
    """Set inventory transmit power (dBm) on the reader via HTTP."""
    value = snap_read_power(read_power_dbm) if snap else float(read_power_dbm)
    base = _normalize_api_base(api_base)
    # Prefer the dedicated path; fall back to the bulk /power endpoint.
    url = f"{base}/power/read_power"
    try:
        response = requests.put(url, params={"value": value}, timeout=timeout)
        if response.status_code == 404:
            response = requests.put(
                f"{base}/power",
                params={"read_power": value},
                timeout=timeout,
            )
        response.raise_for_status()
        payload = response.json()
    except requests.RequestException as exc:
        raise RfidPowerError(f"PUT read_power={value} failed: {exc}") from exc
    if not payload.get("ok", True):
        raise RfidPowerError(f"PUT read_power={value} returned error: {payload}")
    payload.setdefault("read_power", value)
    return payload


def set_power(
    api_base: str,
    *,
    read_power: float | None = None,
    write_power: float | None = None,
    sensitivity: float | None = None,
    timeout: float = 5.0,
) -> dict[str, Any]:
    """Set one or more power-related fields via ``PUT /power``."""
    body: dict[str, float] = {}
    if read_power is not None:
        body["read_power"] = snap_read_power(read_power)
    if write_power is not None:
        body["write_power"] = snap_read_power(write_power)
    if sensitivity is not None:
        body["sensitivity"] = float(sensitivity)
    if not body:
        raise ValueError("At least one of read_power, write_power, sensitivity is required")
    url = f"{_normalize_api_base(api_base)}/power"
    try:
        response = requests.put(url, json=body, timeout=timeout)
        response.raise_for_status()
        payload = response.json()
    except requests.RequestException as exc:
        raise RfidPowerError(f"PUT {url} {body} failed: {exc}") from exc
    if not payload.get("ok", True):
        raise RfidPowerError(f"PUT {url} returned error: {payload}")
    return payload


def parse_power_ladder(spec: str) -> list[float]:
    """Parse ``'30,25,20,15'`` into snapped dBm values (deduped, order preserved)."""
    values: list[float] = []
    seen: set[float] = set()
    for part in spec.replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        snapped = snap_read_power(float(part))
        if snapped not in seen:
            seen.add(snapped)
            values.append(snapped)
    if not values:
        raise ValueError(f"Empty power ladder: {spec!r}")
    return values
