# Copyright 2026. RFID DimOS integration — Go2 + RFID + agent blueprints.
#
# Separate file so `unitree-go2-rfid` does not pull agentic dependencies at import time.

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.visualization.rerun.bridge import RerunBridgeModule

from dimos_rfid.agentic_skills import rfid_agentic_skills
from dimos_rfid.go2_blueprints import _RFID_TRANSPORTS, _rfid_module_blueprint
from dimos_rfid.rfid_rerun import go2_rfid_rerun_config
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

# Uses rfid_agentic_skills (no WebInput) — see agentic_skills.py.
unitree_go2_rfid_agentic = autoconnect(
    unitree_go2,
    _rfid_module_blueprint(),
    RerunBridgeModule.blueprint(**go2_rfid_rerun_config()),
    McpServer.blueprint(),
    McpClient.blueprint(),
    rfid_agentic_skills,
).transports(_RFID_TRANSPORTS)

__all__ = ["unitree_go2_rfid_agentic"]
