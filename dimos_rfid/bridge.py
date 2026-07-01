# Copyright 2026. RFID DimOS integration.
#
# RFID-specific Rerun bridge glue for DimOS.

from __future__ import annotations

from typing import Any

from dimos.visualization.rerun.bridge import RerunBridgeModule

RFID_LCM_TOPIC = "/rfid/tags"
RFID_RERUN_ENTITY = "world/rfid/tags"


def rfid_topic_to_entity(topic: Any) -> str:
    """Map DimOS/LCM topics to Rerun entity paths.

    This mirrors DimOS' default ``entity_prefix + topic`` behavior, but makes
    the RFID topic explicit so the text-document view and logged component
    always agree on the exact entity path.
    """
    topic_str = getattr(topic, "name", None) or str(topic)
    topic_str = topic_str.split("#", 1)[0]
    if topic_str == RFID_LCM_TOPIC:
        return RFID_RERUN_ENTITY
    if not topic_str.startswith("/"):
        topic_str = f"/{topic_str}"
    return f"world{topic_str}"


class RfidRerunBridgeModule(RerunBridgeModule):
    """DimOS Rerun bridge with a stable RFID entity mapping."""

    def _get_entity_path(self, topic: Any) -> str:
        return rfid_topic_to_entity(topic)


__all__ = ["RFID_LCM_TOPIC", "RFID_RERUN_ENTITY", "RfidRerunBridgeModule", "rfid_topic_to_entity"]
