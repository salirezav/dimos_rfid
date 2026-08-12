# Copyright 2026. RFID DimOS integration — Rerun layout with RFID tag list panel.

from __future__ import annotations

from typing import Any

# Must match LCM topic /rfid/tags → Rerun entity prefix world + /rfid/tags
RFID_RERUN_ENTITY = "world/rfid/tags"
# Live multi-round collection status / log (written by RfidCollectionRoundsModule).
COLLECTION_RERUN_ENTITY = "world/rfid/collection"


def _text_view(origin: str, name: str) -> Any:
    import rerun.blueprint as rrb

    if hasattr(rrb, "TextDocumentView"):
        return rrb.TextDocumentView(origin=origin, name=name)
    return rrb.TextLogView(origin=origin, name=name)


def go2_rfid_rerun_blueprint() -> Any:
    """Go2 layout: Camera | 3D map | RFID tag list."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                },
            ),
            _text_view(RFID_RERUN_ENTITY, "RFID"),
            column_shares=[2, 3, 1],
        ),
        rrb.TimePanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
    )


def go2_rfid_collection_rerun_blueprint() -> Any:
    """Go2 layout: Camera | 3D | RFID tags | Collection live log."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                },
            ),
            rrb.Vertical(
                _text_view(RFID_RERUN_ENTITY, "RFID"),
                _text_view(COLLECTION_RERUN_ENTITY, "Collection"),
                row_shares=[1, 1],
            ),
            column_shares=[2, 3, 2],
        ),
        rrb.TimePanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
    )


def _rfid_visual_override(msg: Any) -> Any:
    if hasattr(msg, "to_rerun"):
        return msg.to_rerun()
    return None


def go2_rfid_rerun_config() -> dict[str, Any]:
    """Merge Go2 Rerun settings with RFID panel layout."""
    from dimos.protocol.pubsub.impl.lcmpubsub import LCM, PickleLCM
    from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config

    cfg = {**rerun_config}
    cfg["blueprint"] = go2_rfid_rerun_blueprint

    visual_override = dict(cfg.get("visual_override", {}))
    visual_override[RFID_RERUN_ENTITY] = _rfid_visual_override
    cfg["visual_override"] = visual_override

    max_hz = dict(cfg.get("max_hz", {}))
    max_hz[RFID_RERUN_ENTITY] = 1.0
    cfg["max_hz"] = max_hz

    # RFID uses pLCMTransport (PickleLCM). The default Go2 bridge only
    # listens on typed LCM, so without PickleLCM the RFID panel never updates.
    pubsubs = list(cfg.get("pubsubs") or [])
    has_typed = any(type(p) is LCM for p in pubsubs)
    has_pickle = any(isinstance(p, PickleLCM) for p in pubsubs)
    if not has_typed:
        pubsubs.append(LCM())
    if not has_pickle:
        pubsubs.append(PickleLCM())
    cfg["pubsubs"] = pubsubs

    return cfg


def go2_rfid_collection_rerun_config() -> dict[str, Any]:
    """Rerun layout that includes the live collection status panel."""
    cfg = go2_rfid_rerun_config()
    cfg["blueprint"] = go2_rfid_collection_rerun_blueprint
    return cfg
