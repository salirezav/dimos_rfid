# Copyright 2026. RFID DimOS integration.
#
# Message types published on the rfid_tags LCM stream.

from __future__ import annotations

from dataclasses import dataclass, field
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
    def _display_rssi(tag: RfidTag) -> str:
        return f"{tag.rssi_dbm} dBm" if tag.rssi_dbm is not None else "unknown RSSI"

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
        out_of_range = [t for t in self.tags if not t.in_range]

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

        if in_range:
            lines.append("## In range")
            lines.append("")
            lines.append("| Tag | RSSI | EPC |")
            lines.append("|-----|------|-----|")
            for tag in in_range:
                name = self._display_name(tag)
                rssi = self._display_rssi(tag)
                lines.append(f"| {name} | {rssi} | `{tag.epc}` |")
            lines.append("")
        else:
            lines.append("_No tags in range right now._")
            lines.append("")

        if out_of_range:
            lines.append("## Out of range (seen earlier)")
            lines.append("")
            for tag in out_of_range[:10]:
                name = self._display_name(tag)
                lines.append(f"- {name} - `{tag.epc}`")
            if len(out_of_range) > 10:
                lines.append(f"- _...and {len(out_of_range) - 10} more_")

        return "\n".join(lines)

    def to_rerun(self) -> rr.TextDocument | rr.TextLog:
        """For RerunBridge LCM path (also logged directly from RfidModule)."""
        text = self.to_markdown_panel()
        try:
            return rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN)
        except (AttributeError, TypeError):
            return rr.TextLog(text, level=rr.TextLogLevel.INFO)
