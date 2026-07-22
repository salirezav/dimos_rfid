# Copyright 2026. RFID DimOS integration — Go2 + RFID blueprints.
#
# Requires: uv sync --extra unitree  (dimos[base,unitree])

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import pLCMTransport
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.visualization.rerun.bridge import RerunBridgeModule

from dimos_rfid.recorder import RfidRecorderModule
from dimos_rfid.msgs import RfidTagArray
from dimos_rfid.rfid_module import RfidModule
from dimos_rfid.rfid_rerun import go2_rfid_rerun_config

_RFID_TRANSPORTS = {
    ("rfid_tags", RfidTagArray): pLCMTransport("/rfid/tags"),
    ("rfid_samples", RfidTagArray): pLCMTransport("/rfid/samples"),
}


def _rfid_module_blueprint():
    return RfidModule.blueprint(
        connection_mode=os.environ.get("RFID_CONNECTION_MODE", "http"),
        api_base=os.environ.get(
            "RFID_API_BASE",
            "http://192.168.123.18:8765/api/v1",
        ),
        poll_hz=float(os.environ.get("RFID_POLL_HZ", "1")),
    )


def _recorder_blueprint(*, auto_start: bool = False):
    """Recorder configuration shared by dataset and custom agentic blueprints."""
    configured_auto_start = os.environ.get("RFID_DATASET_AUTO_START")
    if configured_auto_start is not None:
        auto_start = configured_auto_start.strip().lower() in {"1", "true", "yes", "on"}
    return RfidRecorderModule.blueprint(
        output_dir=os.environ.get(
            "RFID_DATASET_DIR",
            os.path.expanduser("~/Downloads/dimos_rfid_datasets"),
        ),
        auto_start=auto_start,
        session_name=os.environ.get("RFID_DATASET_SESSION", ""),
    )


unitree_go2_rfid = autoconnect(
    unitree_go2,
    _rfid_module_blueprint(),
    # Override Go2 Rerun layout: add RFID tag list panel (later module wins).
    RerunBridgeModule.blueprint(**go2_rfid_rerun_config()),
).transports(_RFID_TRANSPORTS)

# Collection variant: recording begins with the stack and is finalized (including
# a ZIP archive) on Ctrl+C / normal shutdown.
unitree_go2_rfid_dataset = autoconnect(
    unitree_go2_rfid,
    _recorder_blueprint(auto_start=True),
).global_config(n_workers=10)

__all__ = [
    "unitree_go2_rfid",
    "unitree_go2_rfid_dataset",
    "_RFID_TRANSPORTS",
    "_recorder_blueprint",
    "_rfid_module_blueprint",
]
