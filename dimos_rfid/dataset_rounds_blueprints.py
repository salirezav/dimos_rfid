# Copyright 2026. RFID DimOS integration — Go2 dataset collection with power rounds.

"""Blueprint: Go2 + RFID + recorder + multi-round power-ladder collection."""

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import pLCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.visualization.rerun.bridge import RerunBridgeModule

from dimos_rfid.collection_rounds import RfidCollectionRoundsModule
from dimos_rfid.go2_blueprints import _recorder_blueprint, _rfid_module_blueprint
from dimos_rfid.msgs import RfidTagArray
from dimos_rfid.rfid_rerun import go2_rfid_collection_rerun_config
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2


def _collection_blueprint(*, auto_start: bool = True):
    configured = os.environ.get("RFID_COLLECTION_AUTO_START")
    if configured is not None:
        auto_start = configured.strip().lower() in {"1", "true", "yes", "on"}
    kwargs: dict = {
        "api_base": os.environ.get(
            "RFID_API_BASE",
            "http://192.168.123.18:8765/api/v1",
        ),
        "power_ladder": os.environ.get("RFID_POWER_LADDER", "30,25,20,15"),
        "auto_start": auto_start,
        "session_name": os.environ.get("RFID_DATASET_SESSION", ""),
    }
    path_file = os.environ.get("RFID_COLLECTION_PATH", "").strip()
    if path_file:
        kwargs["path_file"] = path_file
    return RfidCollectionRoundsModule.blueprint(**kwargs)


_TRANSPORTS = {
    ("rfid_tags", RfidTagArray): pLCMTransport("/rfid/tags"),
    ("rfid_samples", RfidTagArray): pLCMTransport("/rfid/samples"),
    ("goal_request", PoseStamped): pLCMTransport("/goal_request"),
}

# Go2 stack + RFID ingest + offline recorder + power-ladder path rounds.
# Recorder auto-starts; collection rounds auto-start unless RFID_COLLECTION_AUTO_START=0.
unitree_go2_rfid_dataset_rounds = autoconnect(
    unitree_go2,
    _rfid_module_blueprint(),
    _recorder_blueprint(auto_start=True),
    _collection_blueprint(auto_start=True),
    RerunBridgeModule.blueprint(**go2_rfid_collection_rerun_config()),
).transports(_TRANSPORTS).global_config(n_workers=11)

__all__ = [
    "unitree_go2_rfid_dataset_rounds",
    "_collection_blueprint",
]
