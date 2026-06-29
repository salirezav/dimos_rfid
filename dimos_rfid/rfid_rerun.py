# Copyright 2026. RFID DimOS integration — Rerun layout with RFID tag list panel.

from __future__ import annotations

from typing import Any


def go2_rfid_rerun_blueprint() -> Any:
    """Go2 layout: Camera | 3D map | RFID tag list."""
    import rerun as rr
    import rerun.blueprint as rrb

    if hasattr(rrb, "TextDocumentView"):
        rfid_view: Any = rrb.TextDocumentView(
            origin="world/rfid/panel",
            name="RFID",
        )
    else:
        rfid_view = rrb.TextLogView(
            origin="world/rfid/panel",
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


def go2_rfid_rerun_config() -> dict[str, Any]:
    """Merge Go2 Rerun settings with RFID panel + throttle RFID UI updates."""
    from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config

    cfg = {**rerun_config}
    cfg["blueprint"] = go2_rfid_rerun_blueprint
    max_hz = dict(cfg.get("max_hz", {}))
    max_hz["world/rfid/panel"] = 1.0  # at most 1 UI refresh per second
    cfg["max_hz"] = max_hz
    return cfg
