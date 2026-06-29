# Copyright 2026. RFID DimOS integration.
#
# DimOS Module that reads RFID tags via the existing rfid_service API
# (direct in-process) or the HTTP API (rfid_scanner_server.py).

from __future__ import annotations

import os
import time
from typing import Any, Literal

import requests
from pydantic import Field
import reactivex as rx
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

from dimos_rfid._backend import create_direct_scanner
from dimos_rfid.msgs import RfidTagArray

logger = setup_logger()


class RfidModuleConfig(ModuleConfig):
    """How RfidModule connects to the reader."""

    connection_mode: Literal["http", "direct"] = "http"
    api_base: str = Field(
        default_factory=lambda: os.environ.get(
            "RFID_API_BASE", "http://localhost:8765/api/v1"
        )
    )
    reader_host: str = Field(
        default_factory=lambda: os.environ.get("VULCAN_READER_HOST", "192.168.123.2")
    )
    reader_user: str = Field(
        default_factory=lambda: os.environ.get("VULCAN_READER_USER", "admin")
    )
    reader_password: str = Field(
        default_factory=lambda: os.environ.get("VULCAN_READER_PASS", "admin")
    )
    poll_hz: float = Field(default=1.0, gt=0)
    stale_seconds: float = Field(default=5.0, gt=0)
    antenna_frame_id: str = "rfid_antenna"
    antenna_offset_z: float = Field(
        default=0.25,
        description="Height of RFID antenna above base_link (meters).",
    )


class RfidModule(Module):
    """
    Publishes RFID tag reads on the ``rfid_tags`` stream.

    - **http**: poll ``rfid_scanner_server.py`` (default). Run the server on the robot first.
    - **direct**: import ``rfid_service.RfidScanner`` in-process (DimOS must reach the reader IP).
    """

    config: RfidModuleConfig
    rfid_tags: Out[RfidTagArray]

    _scanner: Any = None
    _latest: RfidTagArray | None = None
    _connection_mode: Literal["http", "direct"] = "http"
    _last_publish_key: tuple[tuple[str, Any, bool], ...] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        mode = os.environ.get("RFID_CONNECTION_MODE", self.config.connection_mode)
        if mode not in ("http", "direct"):
            raise ValueError(f"RFID_CONNECTION_MODE must be 'http' or 'direct', got {mode!r}")
        self._connection_mode = mode
        self._publish_antenna_tf()

        if self._connection_mode == "direct":
            self._start_direct()
        else:
            self._start_http_poll()

    def _publish_antenna_tf(self) -> None:
        """Static transform: base_link → rfid_antenna (for Rerun / future localization)."""
        antenna = Transform(
            translation=Vector3(0.0, 0.0, self.config.antenna_offset_z),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id=self.config.antenna_frame_id,
            ts=time.time(),
        )
        self.tf.publish(antenna)

    def _start_direct(self) -> None:
        try:
            self._scanner = create_direct_scanner(
                host=self.config.reader_host,
                user=self.config.reader_user,
                password=self.config.reader_password,
                stale_seconds=self.config.stale_seconds,
            )
            self._scanner.on_tag(self._on_direct_tag)
            self._scanner.connect()
            self._scanner.start()
            logger.info(
                "RFID direct mode: reader %s device %s",
                self.config.reader_host,
                self._scanner.device_id,
            )
        except Exception as exc:
            logger.error("RFID direct connect failed: %s", exc)
            raise

        interval = 1.0 / self.config.poll_hz
        self.register_disposable(
            rx.interval(interval).subscribe(lambda _: self._publish_direct_snapshot())
        )

    def _on_direct_tag(self, tag: dict[str, Any]) -> None:
        array = RfidTagArray.from_tag_dict(
            tag, frame_id=self.config.antenna_frame_id
        )
        self._latest = array
        self._publish_tags(array)

    def _publish_direct_snapshot(self) -> None:
        if self._scanner is None:
            return
        payload = self._scanner.to_api_payload()
        array = RfidTagArray.from_api_payload(
            payload, frame_id=self.config.antenna_frame_id
        )
        self._publish_tags(array)

    def _start_http_poll(self) -> None:
        base = self.config.api_base.rstrip("/")
        logger.info("RFID HTTP mode: polling %s", base)
        if not self._verify_http_reachable(base):
            logger.warning(
                "RFID polls will retry. If curl works from Windows but not Ubuntu/WSL, "
                "DimOS cannot reach the Jetson — test: curl %s/health from the same shell "
                "you use for `dimos run`.",
                base,
            )
        interval = 1.0 / self.config.poll_hz
        self._poll_http()
        self.register_disposable(
            rx.interval(interval).subscribe(lambda _: self._poll_http())
        )

    def _verify_http_reachable(self, base: str) -> bool:
        try:
            response = requests.get(f"{base}/health", timeout=3)
            response.raise_for_status()
            logger.info("RFID API reachable at %s", base)
            return True
        except requests.RequestException as exc:
            logger.error("RFID API not reachable at %s: %s", base, exc)
            return False

    def _poll_http(self) -> None:
        base = self.config.api_base.rstrip("/")
        try:
            response = requests.get(f"{base}/tags", timeout=1.5)
            response.raise_for_status()
            payload = response.json()
            if not payload.get("ok", True) and "tags" not in payload:
                logger.warning("RFID API error: %s", payload.get("error"))
                return
            array = RfidTagArray.from_api_payload(
                payload, frame_id=self.config.antenna_frame_id
            )
            self._publish_tags(array)
        except requests.RequestException as exc:
            logger.warning("RFID HTTP poll failed (%s): %s", base, exc)

    @staticmethod
    def _publish_key(array: RfidTagArray) -> tuple[tuple[str, Any, bool], ...]:
        return tuple(
            (t.epc, t.rssi_dbm, t.in_range)
            for t in sorted(array.tags, key=lambda x: x.epc)
        )

    def _should_publish(self, array: RfidTagArray) -> bool:
        key = self._publish_key(array)
        if key == self._last_publish_key:
            return False
        self._last_publish_key = key
        return True

    def _publish_tags(self, array: RfidTagArray) -> None:
        self._latest = array
        if not self._should_publish(array):
            return
        self.rfid_tags.publish(array)

    @skill
    def get_active_rfid_tags(self) -> str:
        """List RFID tags currently in range with signal strength.

        Returns a human-readable summary for the agent. Tags are identified by EPC hex string.
        """
        if self._connection_mode == "http":
            try:
                response = requests.get(
                    f"{self.config.api_base.rstrip('/')}/tags/active",
                    timeout=3,
                )
                response.raise_for_status()
                tags = response.json().get("tags", [])
            except requests.RequestException as exc:
                return f"RFID reader unreachable: {exc}"
        elif self._scanner is not None:
            tags = self._scanner.get_active_tags()
        elif self._latest is not None:
            tags = [t.__dict__ for t in self._latest.active_tags()]
        else:
            return "RFID scanner not started."

        if not tags:
            return "No RFID tags in range."

        lines = []
        for tag in tags:
            epc = tag.get("epc", "?")
            rssi = tag.get("rssi_dbm")
            rssi_s = f"{rssi} dBm" if rssi is not None else "unknown RSSI"
            lines.append(f"- {epc}: {rssi_s}")
        return f"{len(lines)} tag(s) in range:\n" + "\n".join(lines)

    @skill
    def lookup_rfid_tag(self, epc: str) -> str:
        """Look up one RFID tag by EPC hex string.

        Args:
            epc: Full or partial EPC hex (case-insensitive).
        """
        epc = epc.strip().lower()
        if self._connection_mode == "http":
            try:
                response = requests.get(
                    f"{self.config.api_base.rstrip('/')}/tags/{epc}",
                    timeout=3,
                )
                if response.status_code == 404:
                    return f"Tag never seen: {epc}"
                response.raise_for_status()
                tag = response.json().get("tag")
                if tag is None:
                    return f"Tag not found: {epc}"
                return self._format_tag(tag)
            except requests.RequestException as exc:
                return f"RFID reader unreachable: {exc}"

        if self._scanner is None:
            return "RFID scanner not started."
        tag = self._scanner.get_tag(epc)
        if tag is None:
            return f"Tag never seen: {epc}"
        return self._format_tag(tag)

    @skill
    def get_rfid_reader_status(self) -> str:
        """Return RFID reader connection health and tag counts."""
        if self._connection_mode == "http":
            try:
                response = requests.get(
                    f"{self.config.api_base.rstrip('/')}/reader/status",
                    timeout=3,
                )
                response.raise_for_status()
                status = response.json()
            except requests.RequestException as exc:
                return f"RFID reader unreachable: {exc}"
        elif self._scanner is not None:
            status = self._scanner.get_status()
        else:
            return "RFID scanner not started."

        stream = status.get("stream", {})
        return (
            f"host={status.get('reader_host')} "
            f"device={status.get('device_id')} "
            f"running={status.get('reader_started')} "
            f"stream_ok={stream.get('connected')} "
            f"tags={status.get('tag_count')} "
            f"active={status.get('active_count')}"
        )

    @staticmethod
    def _format_tag(tag: dict[str, Any]) -> str:
        epc = tag.get("epc", "?")
        in_range = tag.get("in_range", False)
        rssi = tag.get("rssi_dbm")
        reads = tag.get("read_count", 0)
        state = "in range" if in_range else "out of range"
        rssi_s = f"{rssi} dBm" if rssi is not None else "no RSSI"
        return f"{epc}: {state}, {rssi_s}, {reads} reads this session"

    @rpc
    def stop(self) -> None:
        if self._scanner is not None:
            try:
                self._scanner.stop()
            except Exception:
                pass
            self._scanner = None
        super().stop()
