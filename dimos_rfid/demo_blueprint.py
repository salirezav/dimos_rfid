# Copyright 2026. RFID DimOS integration — standalone demo blueprint.

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import pLCMTransport
from dimos.visualization.rerun.bridge import RerunBridgeModule

from dimos_rfid.msgs import RfidTagArray
from dimos_rfid.rfid_module import RfidModule
from dimos_rfid.rfid_rerun import rfid_only_rerun_blueprint

rfid_demo = autoconnect(
    RfidModule.blueprint(
        connection_mode=os.environ.get("RFID_CONNECTION_MODE", "http"),
        api_base=os.environ.get("RFID_API_BASE", "http://localhost:8765/api/v1"),
    ),
    # Layout only — RfidModule pushes tag text over Rerun gRPC (not PickleLCM
    # subscribe_all, which would try to unpickle unrelated LCM traffic).
    RerunBridgeModule.blueprint(blueprint=rfid_only_rerun_blueprint),
).transports(
    {
        ("rfid_tags", RfidTagArray): pLCMTransport("/rfid/tags"),
    }
)

__all__ = ["rfid_demo"]
