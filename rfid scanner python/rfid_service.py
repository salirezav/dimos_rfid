#!/usr/bin/env python3
"""
High-level RFID scanner service for the Vulcan / Keonn AdvanNet reader.

Use this module from Python scripts, ROS nodes, or the HTTP API server.

Example:
    from rfid_service import RfidScanner

    with RfidScanner() as scanner:
        scanner.start()
        time.sleep(5)
        for tag in scanner.get_active_tags():
            print(tag["epc"], tag["rssi_dbm"])
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Iterator, Optional

from vulcan_rfid_reader import (
    DEFAULT_HOST,
    DEFAULT_PASS,
    DEFAULT_USER,
    AdvanNetClient,
    AdvanNetEventStream,
    TagRead,
    parse_event_xml,
)


def _tag_display_name(epc: str) -> str:
    if len(epc) >= 8:
        return f"Tag …{epc[-8:].upper()}"
    return f"Tag {epc.upper()}"


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%H:%M:%S UTC")


@dataclass
class ScannerConfig:
    host: str = field(default_factory=lambda: os.environ.get("VULCAN_READER_HOST", DEFAULT_HOST))
    user: str = field(default_factory=lambda: os.environ.get("VULCAN_READER_USER", DEFAULT_USER))
    password: str = field(default_factory=lambda: os.environ.get("VULCAN_READER_PASS", DEFAULT_PASS))
    stale_seconds: float = field(
        default_factory=lambda: float(os.environ.get("RFID_STALE_SECONDS", "5"))
    )
    stream_dead_seconds: float = field(
        default_factory=lambda: float(os.environ.get("RFID_STREAM_DEAD_SECONDS", "45"))
    )


class RfidScanner:
    """
    Connects to the Vulcan RFID reader, runs continuous inventory, and tracks tags.

    Thread-safe. Safe to call get_tags() from multiple threads while scanning.
    """

    def __init__(self, config: Optional[ScannerConfig] = None):
        self.config = config or ScannerConfig()
        self._lock = threading.Lock()
        self._tags: dict[str, dict[str, Any]] = {}
        self._client: Optional[AdvanNetClient] = None
        self._device_id: Optional[str] = None
        self._reader_started = False
        self._stream_thread: Optional[threading.Thread] = None
        self._stream_stop = threading.Event()
        self._stream_status: dict[str, Any] = {
            "connected": False,
            "error": None,
            "device_id": None,
            "last_activity": 0.0,
        }
        self._callbacks: list[Callable[[dict[str, Any]], None]] = []

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> str:
        """Discover the reader device. Returns device ID. Does not start inventory."""
        self._client = AdvanNetClient(
            self.config.host, self.config.user, self.config.password
        )
        devices = self._client.devices()
        if not devices:
            raise RuntimeError(f"No RFID devices found at {self.config.host}")
        self._device_id = devices[0]["id"]
        self._stream_status["device_id"] = self._device_id
        return self._device_id

    def start(self) -> None:
        """Start reader inventory and background event-stream listener."""
        if self._reader_started:
            return
        if not self._client or not self._device_id:
            self.connect()
        assert self._client is not None
        assert self._device_id is not None
        self._client.start(self._device_id)
        self._reader_started = True
        self._stream_stop.clear()
        self._stream_thread = threading.Thread(
            target=self._stream_loop, name="rfid-event-stream", daemon=True
        )
        self._stream_thread.start()

    def stop(self) -> None:
        """Stop background listener and send reader stop command."""
        self._stream_stop.set()
        if self._stream_thread and self._stream_thread.is_alive():
            self._stream_thread.join(timeout=3)
        self._stream_thread = None
        if self._client and self._device_id and self._reader_started:
            try:
                self._client.stop(self._device_id)
            except Exception:
                pass
        self._reader_started = False
        self._stream_status["connected"] = False

    def is_running(self) -> bool:
        return self._reader_started

    @property
    def device_id(self) -> Optional[str]:
        return self._device_id

    def __enter__(self) -> "RfidScanner":
        self.connect()
        self.start()
        return self

    def __exit__(self, *_) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # Reading API
    # ------------------------------------------------------------------

    def get_tags(self, active_only: bool = False) -> list[dict[str, Any]]:
        """Return discovered tags, in-range first then out-of-range."""
        tags = self._annotate_and_sort()
        if active_only:
            return [t for t in tags if t["in_range"]]
        return tags

    def get_active_tags(self) -> list[dict[str, Any]]:
        """Tags seen within stale_seconds (currently in range)."""
        return self.get_tags(active_only=True)

    def get_tag(self, epc: str) -> Optional[dict[str, Any]]:
        """Look up a single tag by EPC. Returns None if never seen."""
        epc = epc.lower()
        with self._lock:
            entry = self._tags.get(epc)
            if entry is None:
                # try case-insensitive match
                for key, val in self._tags.items():
                    if key.lower() == epc:
                        entry = val
                        break
        if entry is None:
            return None
        annotated = self._annotate_entry(dict(entry))
        return annotated

    def poll_inventory(self) -> list[dict[str, Any]]:
        """
        One-shot inventory via REST (reader buffer snapshot).
        Does not replace the live event stream; useful for a synchronous read.
        """
        if not self._client or not self._device_id:
            self.connect()
        assert self._client is not None
        assert self._device_id is not None
        reads = self._client.inventory(self._device_id)
        result = []
        for tag in reads:
            self._apply_read(tag)
            entry = self.get_tag(tag.epc)
            if entry:
                result.append(entry)
        return result

    def clear_tags(self) -> int:
        """Clear all discovered tags from memory. Returns count removed."""
        with self._lock:
            count = len(self._tags)
            self._tags.clear()
        return count

    def on_tag(self, callback: Callable[[dict[str, Any]], None]) -> None:
        """Register callback invoked on each new tag read event (in-range update)."""
        self._callbacks.append(callback)

    def get_status(self) -> dict[str, Any]:
        """Reader, stream, and tag summary."""
        tags = self.get_tags()
        stream = dict(self._stream_status)
        stream["connected"] = self._stream_is_healthy()
        return {
            "reader_host": self.config.host,
            "device_id": self._device_id,
            "reader_started": self._reader_started,
            "stream": stream,
            "tag_count": len(tags),
            "active_count": sum(1 for t in tags if t["in_range"]),
            "stale_seconds": self.config.stale_seconds,
            "updated_at": datetime.now(timezone.utc).isoformat(),
        }

    def to_api_payload(self) -> dict[str, Any]:
        """Full tag list payload (used by HTTP API and web UI)."""
        tags = self.get_tags()
        stream = dict(self._stream_status)
        stream["connected"] = self._stream_is_healthy()
        return {
            "tags": tags,
            "count": len(tags),
            "active_count": sum(1 for t in tags if t["in_range"]),
            "stale_seconds": self.config.stale_seconds,
            "scanner": stream,
            "reader_host": self.config.host,
            "device_id": self._device_id,
            "reader_started": self._reader_started,
            "updated_at": datetime.now(timezone.utc).isoformat(),
        }

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _stream_is_healthy(self) -> bool:
        last = self._stream_status.get("last_activity", 0.0)
        return (time.time() - last) <= self.config.stream_dead_seconds

    def _annotate_entry(self, entry: dict[str, Any]) -> dict[str, Any]:
        in_range = (time.time() - entry["last_seen"]) <= self.config.stale_seconds
        return {**entry, "in_range": in_range}

    def _annotate_and_sort(self) -> list[dict[str, Any]]:
        with self._lock:
            entries = [dict(v) for v in self._tags.values()]
        tags = [self._annotate_entry(e) for e in entries]
        in_range = sorted(
            [t for t in tags if t["in_range"]],
            key=lambda t: t.get("rssi_dbm") if t.get("rssi_dbm") is not None else -999,
            reverse=True,
        )
        out_of_range = sorted(
            [t for t in tags if not t["in_range"]],
            key=lambda t: t.get("last_seen", 0),
            reverse=True,
        )
        return in_range + out_of_range

    def _apply_read(self, tag: TagRead) -> dict[str, Any]:
        now = time.time()
        now_iso = _utc_now_iso()
        with self._lock:
            prev = self._tags.get(tag.epc)
            event_reads = int(tag.raw_props.get("READ_COUNT", "1") or "1")
            if prev:
                total_reads = prev.get("read_count", 0) + 1
                entry = {
                    **prev,
                    "rssi_dbm": tag.rssi if tag.rssi is not None else prev.get("rssi_dbm"),
                    "antenna": tag.antenna if tag.antenna is not None else prev.get("antenna"),
                    "frequency_khz": tag.frequency_khz
                    if tag.frequency_khz is not None
                    else prev.get("frequency_khz"),
                    "phase": tag.raw_props.get("RF_PHASE") or prev.get("phase"),
                    "read_count": max(total_reads, event_reads),
                    "last_seen": now,
                    "last_seen_iso": now_iso,
                }
            else:
                entry = {
                    "id": tag.epc,
                    "name": _tag_display_name(tag.epc),
                    "epc": tag.epc,
                    "rssi_dbm": tag.rssi,
                    "antenna": tag.antenna,
                    "frequency_khz": tag.frequency_khz,
                    "read_count": event_reads,
                    "first_seen": now,
                    "first_seen_iso": now_iso,
                    "last_seen": now,
                    "last_seen_iso": now_iso,
                    "phase": tag.raw_props.get("RF_PHASE"),
                    "device_id": tag.raw_props.get("deviceId") or self._device_id,
                }
            self._tags[tag.epc] = entry
        annotated = self._annotate_entry(entry)
        for cb in self._callbacks:
            try:
                cb(annotated)
            except Exception:
                pass
        return annotated

    def _stream_loop(self) -> None:
        while not self._stream_stop.is_set():
            stream = AdvanNetEventStream(self.config.host)
            try:
                stream.connect()
                assert stream._sock is not None
                stream._sock.settimeout(None)
                self._stream_status["connected"] = True
                self._stream_status["error"] = None
                self._stream_status["last_activity"] = time.time()
                for xml_msg in stream.messages():
                    if self._stream_stop.is_set():
                        break
                    self._stream_status["last_activity"] = time.time()
                    self._stream_status["connected"] = True
                    if xml_msg:
                        for tag in parse_event_xml(xml_msg):
                            self._apply_read(tag)
            except Exception as exc:
                if self._stream_stop.is_set():
                    break
                self._stream_status["connected"] = False
                self._stream_status["error"] = str(exc)
                time.sleep(2)
            finally:
                stream.close()


# Shared singleton for the web/API server process
_scanner: Optional[RfidScanner] = None
_scanner_lock = threading.Lock()


def get_scanner(*, autostart: bool = False) -> RfidScanner:
    """Return the process-wide RfidScanner singleton."""
    global _scanner
    with _scanner_lock:
        if _scanner is None:
            _scanner = RfidScanner()
        if autostart and not _scanner.is_running():
            _scanner.connect()
            _scanner.start()
        return _scanner


def reset_scanner() -> None:
    """Stop and discard the singleton (mainly for tests)."""
    global _scanner
    with _scanner_lock:
        if _scanner is not None:
            _scanner.stop()
            _scanner = None
