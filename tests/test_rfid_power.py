# Copyright 2026. RFID DimOS integration.

from __future__ import annotations

import pytest

from dimos_rfid.rfid_power import parse_power_ladder, snap_read_power


def test_snap_read_power_clamps_and_quantizes() -> None:
    assert snap_read_power(14.9) == 15.0
    assert snap_read_power(15.2) == 15.0
    assert snap_read_power(15.3) == 15.5
    assert snap_read_power(-1.0) == 0.0
    assert snap_read_power(40.0) == 31.5


def test_parse_power_ladder_order_and_dedupe() -> None:
    assert parse_power_ladder("30,25,20,15") == [30.0, 25.0, 20.0, 15.0]
    assert parse_power_ladder("30; 25.2; 30; 15") == [30.0, 25.0, 15.0]


def test_parse_power_ladder_rejects_empty() -> None:
    with pytest.raises(ValueError, match="Empty power ladder"):
        parse_power_ladder(" , ; ")
