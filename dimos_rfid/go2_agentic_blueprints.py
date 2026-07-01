# Copyright 2026. RFID DimOS integration — Go2 + RFID + agent blueprints.
#
# Separate file so `unitree-go2-rfid` does not pull agentic dependencies at import time.

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import _common_agentic

from dimos_rfid.bridge import RfidRerunBridgeModule
from dimos_rfid.go2_blueprints import _RFID_TRANSPORTS, _rfid_module_blueprint
from dimos_rfid.rfid_rerun import go2_rfid_rerun_config
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

unitree_go2_rfid_agentic = autoconnect(
    unitree_go2,
    _rfid_module_blueprint(),
    RfidRerunBridgeModule.blueprint(**go2_rfid_rerun_config()),
    McpServer.blueprint(),
    McpClient.blueprint(),
    _common_agentic,
).transports(_RFID_TRANSPORTS)

__all__ = ["unitree_go2_rfid_agentic"]
