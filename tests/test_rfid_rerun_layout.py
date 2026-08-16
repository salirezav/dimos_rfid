"""Regression tests for the upstream-preserving Go2 + RFID layout."""

from __future__ import annotations

from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config

from dimos_rfid.rfid_rerun import (
    RFID_RERUN_ENTITY,
    go2_rfid_rerun_blueprint,
    go2_rfid_rerun_config,
)


def test_bridge_config_changes_only_the_blueprint_factory() -> None:
    custom = go2_rfid_rerun_config()

    assert custom.keys() == rerun_config.keys()
    for key, value in rerun_config.items():
        if key != "blueprint":
            assert custom[key] is value
    assert custom["blueprint"] is go2_rfid_rerun_blueprint


def test_layout_embeds_the_exact_upstream_root_and_adds_rfid_view(monkeypatch) -> None:
    upstream = rerun_config["blueprint"]()
    monkeypatch.setitem(rerun_config, "blueprint", lambda: upstream)
    custom = go2_rfid_rerun_blueprint()

    upstream_root, rfid_view = custom.root_container.contents
    assert upstream_root is upstream.root_container
    assert rfid_view.origin == RFID_RERUN_ENTITY
    assert rfid_view.name == "RFID"

    for attribute in ("top_panel", "blueprint_panel", "selection_panel", "time_panel"):
        assert hasattr(custom, attribute) == hasattr(upstream, attribute)
