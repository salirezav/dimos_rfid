# Copyright 2026. RFID DimOS integration — Rerun layout with RFID tag list panel.

from __future__ import annotations

from typing import Any

# Must match LCM topic /rfid/tags → Rerun entity prefix world + /rfid/tags
RFID_RERUN_ENTITY = "world/rfid/tags"
RFID_EMPTY_PANEL = """# RFID scanner

**Status:** Waiting for RFID data

**In range:** 0  -  **Discovered:** 0

_No tag updates have reached the viewer yet._
"""


def go2_rfid_rerun_blueprint() -> Any:
    """Go2 layout: Camera | 3D map | RFID tag list."""
    import rerun as rr
    import rerun.blueprint as rrb

    rfid_view: Any
    if hasattr(rrb, "TextDocumentView"):
        rfid_view = rrb.TextDocumentView(
            origin=RFID_RERUN_ENTITY,
            name="RFID",
        )
    else:
        rfid_view = rrb.TextLogView(
            origin=RFID_RERUN_ENTITY,
            name="RFID",
        )

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
            rfid_view,
            column_shares=[2, 3, 1],
        ),
        rrb.TimePanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
    )


def _rfid_visual_override(msg: Any) -> Any:
    if hasattr(msg, "to_rerun"):
        return msg.to_rerun()
    return None


def _rfid_static_panel(rr: Any) -> Any:
    try:
        return rr.TextDocument(RFID_EMPTY_PANEL, media_type=rr.MediaType.MARKDOWN)
    except (AttributeError, TypeError):
        return rr.TextLog(RFID_EMPTY_PANEL, level=rr.TextLogLevel.INFO)


def go2_rfid_rerun_config() -> dict[str, Any]:
    """Merge Go2 Rerun settings with RFID panel layout."""
    from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config

    cfg = {**rerun_config}
    cfg["blueprint"] = go2_rfid_rerun_blueprint

    visual_override = dict(cfg.get("visual_override", {}))
    visual_override[RFID_RERUN_ENTITY] = _rfid_visual_override
    cfg["visual_override"] = visual_override

    max_hz = dict(cfg.get("max_hz", {}))
    max_hz[RFID_RERUN_ENTITY] = 1.0
    cfg["max_hz"] = max_hz

    static = dict(cfg.get("static", {}))
    static[RFID_RERUN_ENTITY] = _rfid_static_panel
    cfg["static"] = static

    if "pubsubs" not in cfg:
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM

        cfg["pubsubs"] = [LCM()]

    return cfg
