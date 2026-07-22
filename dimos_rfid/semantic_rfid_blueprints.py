# Copyright 2026. RFID DimOS integration — Go2 + semantic particle-filter localizer.

"""Blueprints that run DimOS with RFID + semantic particle-filter localization."""

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.visualization.rerun.bridge import RerunBridgeModule

from dimos_rfid.go2_blueprints import _RFID_TRANSPORTS, _rfid_module_blueprint
from dimos_rfid.rfid_rerun import go2_rfid_rerun_config
from dimos_rfid.rfid_semantic_localizer import RfidSemanticLocalizerModule
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2


def _localizer_blueprint():
    return RfidSemanticLocalizerModule.blueprint(
        n_particles=int(os.environ.get("RFID_PF_PARTICLES", "5000")),
        xmin=float(os.environ.get("RFID_PF_XMIN", "-5")),
        xmax=float(os.environ.get("RFID_PF_XMAX", "15")),
        ymin=float(os.environ.get("RFID_PF_YMIN", "-5")),
        ymax=float(os.environ.get("RFID_PF_YMAX", "15")),
        zmin=float(os.environ.get("RFID_PF_ZMIN", "0")),
        zmax=float(os.environ.get("RFID_PF_ZMAX", "3")),
        map_resolution=float(os.environ.get("RFID_PF_MAP_RES", "0.2")),
        map_npz_path=os.environ.get("RFID_SEMANTIC_MAP", ""),
        log_estimates_hz=float(os.environ.get("RFID_PF_LOG_HZ", "0.5")),
    )


# Go2 stack + RFID ingest + semantic particle filter.
unitree_go2_rfid_semantic = autoconnect(
    unitree_go2,
    _rfid_module_blueprint(),
    _localizer_blueprint(),
    RerunBridgeModule.blueprint(**go2_rfid_rerun_config()),
).transports(_RFID_TRANSPORTS)

__all__ = ["unitree_go2_rfid_semantic", "_localizer_blueprint"]
