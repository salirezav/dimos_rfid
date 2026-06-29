# Copyright 2026. RFID DimOS integration — Go2 + RFID blueprints.
#
# Requires: uv sync --extra unitree  (dimos[base,unitree])

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import pLCMTransport
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

from dimos_rfid.msgs import RfidTagArray
from dimos_rfid.rfid_module import RfidModule
from dimos_rfid.rfid_overlay_module import RfidCameraOverlay, RfidOverlayModule

_RFID_TRANSPORTS = {
    ("rfid_tags", RfidTagArray): pLCMTransport("/rfid/tags"),
    ("rfid_overlay", RfidCameraOverlay): pLCMTransport("/rfid/camera_overlay"),
}


def _rfid_module_blueprint():
    return RfidModule.blueprint(
        connection_mode=os.environ.get("RFID_CONNECTION_MODE", "http"),
        api_base=os.environ.get(
            "RFID_API_BASE",
            "http://192.168.123.18:8765/api/v1",
        ),
    )


unitree_go2_rfid = autoconnect(
    unitree_go2,
    _rfid_module_blueprint(),
    RfidOverlayModule.blueprint(),
).transports(_RFID_TRANSPORTS)

__all__ = ["unitree_go2_rfid", "_RFID_TRANSPORTS", "_rfid_module_blueprint"]
