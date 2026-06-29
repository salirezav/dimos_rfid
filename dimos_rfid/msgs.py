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
            f"**In range:** {len(in_range)}  ·  **Discovered:** {self.total_count}",
            "",
        ]

        if in_range:
            lines.append("## In range")
            lines.append("")
            lines.append("| Tag | RSSI | EPC |")
            lines.append("|-----|------|-----|")
            for tag in in_range:
                name = tag.name or f"…{tag.epc[-8:].upper()}"
                rssi = f"{tag.rssi_dbm} dBm" if tag.rssi_dbm is not None else "—"
                lines.append(f"| {name} | {rssi} | `{tag.epc}` |")
            lines.append("")
        else:
            lines.append("_No tags in range right now._")
            lines.append("")

        if out_of_range:
            lines.append("## Out of range (seen earlier)")
            lines.append("")
            for tag in out_of_range[:10]:
                name = tag.name or f"…{tag.epc[-8:].upper()}"
                lines.append(f"- {name} — `{tag.epc}`")
            if len(out_of_range) > 10:
                lines.append(f"- _…and {len(out_of_range) - 10} more_")

        return "\n".join(lines)

    def to_rerun(self) -> list[tuple[str, rr.Archetype]]:
        """RFID tag list panel (right column in Rerun blueprint)."""
        text = self.to_markdown_panel()
        try:
            doc = rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN)
        except (AttributeError, TypeError):
            doc = rr.TextLog(text, level=rr.TextLogLevel.INFO)
        return [("world/rfid/panel", doc)]
