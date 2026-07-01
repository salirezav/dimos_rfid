# Copyright 2026. RFID DimOS integration.
#
# Message types published on the rfid_tags LCM stream.

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any

import rerun as rr


@dataclass
class RfidTag:
    """Single RFID tag observation."""

    epc: str
    rssi_dbm: float | None = None
    antenna: int | None = None
    frequency_khz: int | None = None
    read_count: int = 0
    in_range: bool = False
    last_seen: float = 0.0
    name: str = ""
    phase: str = ""

    @classmethod
    def from_api_dict(cls, data: dict[str, Any]) -> RfidTag:
        return cls(
            epc=str(data.get("epc", "")),
            rssi_dbm=data.get("rssi_dbm"),
            antenna=data.get("antenna"),
            frequency_khz=data.get("frequency_khz"),
            read_count=int(data.get("read_count", 0)),
            in_range=bool(data.get("in_range", False)),
            last_seen=float(data.get("last_seen", 0.0)),
            name=str(data.get("name", "")),
            phase=str(data.get("phase") or ""),
        )


@dataclass
class RfidTagArray:
    """Batch of RFID tags for one publish cycle."""

    tags: list[RfidTag] = field(default_factory=list)
    frame_id: str = "rfid_antenna"
    active_count: int = 0
    total_count: int = 0
    connection_status: str = ""

    @classmethod
    def from_api_payload(cls, payload: dict[str, Any], *, frame_id: str = "rfid_antenna") -> RfidTagArray:
        tags = [RfidTag.from_api_dict(t) for t in payload.get("tags", [])]
        active = sum(1 for t in tags if t.in_range)
        return cls(
            tags=tags,
            frame_id=frame_id,
            active_count=active,
            total_count=len(tags),
        )

    @classmethod
    def from_tag_dict(cls, tag: dict[str, Any], *, frame_id: str = "rfid_antenna") -> RfidTagArray:
        """Wrap a single live tag event from the direct scanner callback."""
        return cls.from_api_payload({"tags": [tag]}, frame_id=frame_id)

    def active_tags(self) -> list[RfidTag]:
        return [t for t in self.tags if t.in_range]

    @staticmethod
    def _display_name(tag: RfidTag) -> str:
        return tag.name or f"...{tag.epc[-8:].upper()}"

    @staticmethod
    def _short_epc(tag: RfidTag) -> str:
        return f"...{tag.epc[-8:].upper()}" if tag.epc else "N/A"

    @staticmethod
    def _display_rssi(tag: RfidTag) -> str:
        return f"{tag.rssi_dbm} dBm" if tag.rssi_dbm is not None else "unknown RSSI"

    @staticmethod
    def _display_value(value: Any) -> str:
        if value is None or value == "":
            return "N/A"
        return str(value)

    @staticmethod
    def _display_timestamp(ts: float) -> str:
        if not ts:
            return "N/A"
        return datetime.fromtimestamp(ts).isoformat(timespec="seconds")

    @staticmethod
    def _markdown_cell(value: Any) -> str:
        return str(value).replace("|", "\\|")

    def to_terminal_summary(self) -> str:
        """Single-line tag details for logs."""
        if not self.tags:
            return "No RFID tags discovered."

        parts = []
        for tag in sorted(
            self.tags,
            key=lambda t: (
                not t.in_range,
                -(t.rssi_dbm if t.rssi_dbm is not None else -999),
                t.epc,
            ),
        ):
            state = "live" if tag.in_range else "seen"
            antenna = f", ant={tag.antenna}" if tag.antenna is not None else ""
            reads = f", reads={tag.read_count}" if tag.read_count else ""
            parts.append(
                f"{self._display_name(tag)} {state} "
                f"epc={tag.epc} rssi={self._display_rssi(tag)}{antenna}{reads}"
            )

        return "; ".join(parts)

    def to_markdown_panel(self) -> str:
        """Human-readable tag list for the RFID side panel."""
        in_range = sorted(
            self.active_tags(),
            key=lambda t: t.rssi_dbm if t.rssi_dbm is not None else -999,
            reverse=True,
        )
        lines = [
            "# RFID scanner",
            "",
        ]
        if self.connection_status:
            lines.append(f"**Status:** {self.connection_status}")
            lines.append("")

        lines.extend(
            [
                f"**In range:** {len(in_range)}  -  **Discovered:** {self.total_count}",
                "",
            ]
        )

        if not in_range:
            lines.append("**No active RFID tags**")
            lines.append("")

        lines.append("| timestamp | status | epc | short_epc | rssi_dbm | antenna | read_count | phase | robot_x | robot_y | robot_z | robot_yaw |")
        lines.append("|---|---|---|---|---:|---:|---:|---|---:|---:|---:|---:|")

        if self.tags:
            for tag in sorted(
                self.tags,
                key=lambda t: (
                    not t.in_range,
                    -(t.rssi_dbm if t.rssi_dbm is not None else -999),
                    t.epc,
                ),
            ):
                status = "live" if tag.in_range else "seen"
                lines.append(
                    "| "
                    f"{self._markdown_cell(self._display_timestamp(tag.last_seen))} | "
                    f"{status} | "
                    f"`{self._markdown_cell(tag.epc)}` | "
                    f"{self._short_epc(tag)} | "
                    f"{self._display_value(tag.rssi_dbm)} | "
                    f"{self._display_value(tag.antenna)} | "
                    f"{self._display_value(tag.read_count)} | "
                    f"{self._markdown_cell(self._display_value(tag.phase))} | "
                    "N/A | N/A | N/A | N/A |"
                )
        else:
            lines.append("| N/A | seen | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A |")

        return "\n".join(lines)

    def to_rerun(self) -> rr.TextDocument | rr.TextLog:
        """Convert this message for DimOS RerunBridgeModule."""
        text = self.to_markdown_panel()
        try:
            return rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN)
        except (AttributeError, TypeError):
            return rr.TextLog(text, level=rr.TextLogLevel.INFO)
