# Copyright 2026. RFID DimOS integration — standalone demo blueprint.

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import pLCMTransport
from dimos.protocol.pubsub.impl.lcmpubsub import PickleLCM
from dimos.visualization.rerun.bridge import RerunBridgeModule

from dimos_rfid.msgs import RfidTagArray
from dimos_rfid.rfid_module import RfidModule
from dimos_rfid.rfid_rerun import (
    RFID_RERUN_ENTITY,
    _rfid_visual_override,
    go2_rfid_rerun_blueprint,
)

rfid_demo = autoconnect(
    RfidModule.blueprint(
        connection_mode=os.environ.get("RFID_CONNECTION_MODE", "http"),
        api_base=os.environ.get("RFID_API_BASE", "http://localhost:8765/api/v1"),
    ),
    # Listen on PickleLCM — rfid_tags uses pLCMTransport, not typed LCM.
    RerunBridgeModule.blueprint(
        pubsubs=[PickleLCM()],
        blueprint=go2_rfid_rerun_blueprint,
        visual_override={RFID_RERUN_ENTITY: _rfid_visual_override},
        max_hz={RFID_RERUN_ENTITY: 1.0},
    ),
).transports(
    {
        ("rfid_tags", RfidTagArray): pLCMTransport("/rfid/tags"),
    }
)

__all__ = ["rfid_demo"]
