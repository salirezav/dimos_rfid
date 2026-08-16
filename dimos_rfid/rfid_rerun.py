# Copyright 2026. RFID DimOS integration — upstream Go2 layout + RFID panel.

from __future__ import annotations

from typing import Any

# Must match LCM topic /rfid/tags → Rerun entity prefix world + /rfid/tags
RFID_RERUN_ENTITY = "world/rfid/tags"


def _rfid_view() -> Any:
    """Build the Rerun text view used by both RFID viewer modes."""
    import rerun.blueprint as rrb

    if hasattr(rrb, "TextDocumentView"):
        return rrb.TextDocumentView(
            origin=RFID_RERUN_ENTITY,
            name="RFID",
        )
    return rrb.TextLogView(
        origin=RFID_RERUN_ENTITY,
        name="RFID",
    )


def rfid_only_rerun_blueprint() -> Any:
    """Standalone RFID layout used when no Go2 viewer exists."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        _rfid_view(),
        rrb.TimePanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
    )


def _with_root(base: Any, root: Any) -> Any:
    """Replace only a blueprint's root while preserving all panel settings."""
    import rerun.blueprint as rrb

    parts = [root]
    for attribute in ("top_panel", "blueprint_panel", "selection_panel", "time_panel"):
        panel = getattr(base, attribute, None)
        if panel is not None:
            parts.append(panel)

    return rrb.Blueprint(
        *parts,
        auto_layout=base.auto_layout,
        auto_views=base.auto_views,
        collapse_panels=base.collapse_panels,
    )


def go2_rfid_rerun_blueprint() -> Any:
    """Keep the exact upstream Go2 viewport and append a visible RFID panel."""
    import rerun.blueprint as rrb

    from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config

    upstream_factory = rerun_config["blueprint"]
    upstream = upstream_factory()
    root = rrb.Horizontal(
        # Do not reconstruct Camera or 3D views here. Keeping the upstream root
        # retains its point-cloud entity, visibility overrides, and proportions.
        upstream.root_container,
        _rfid_view(),
        column_shares=[5, 1],
        name="DimOS + RFID",
    )
    return _with_root(upstream, root)


def go2_rfid_rerun_config() -> dict[str, Any]:
    """Use every upstream Go2 bridge setting, changing only its layout factory."""
    from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config

    cfg = {**rerun_config}
    cfg["blueprint"] = go2_rfid_rerun_blueprint

    # Do NOT add PickleLCM to bridge pubsubs. RFID uses pLCMTransport, but
    # RerunBridgeModule.subscribe_all() would then try to pickle.loads() every
    # typed LCM message on the bus (lidar, costmaps, …) and spam UnpicklingError.
    # RfidModule logs the panel directly over Rerun gRPC instead.

    if "pubsubs" not in cfg:
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM

        cfg["pubsubs"] = [LCM()]

    return cfg
