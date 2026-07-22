#!/usr/bin/env python3
"""Run DimOS with Go2 + RFID + semantic particle-filter localization.

Usage (from the repository root):

    export ROBOT_IP=<go2-wifi-ip>
    export RFID_API_BASE=http://<go2-wifi-ip>:8765/api/v1
    uv run python run_semantic_rfid.py

Optional flags:

    uv run python run_semantic_rfid.py --agentic   # also start MCP agent skills
    uv run python run_semantic_rfid.py --help

See dimos_rfid/README.md → “Semantic particle filter”.

Workflow:
  1. Edit dimos_rfid/rfid_focus.txt with your tag EPC/suffix (empty = all tags)
  2. export ROBOT_IP and RFID_API_BASE
  3. uv run python run_semantic_rfid.py
  4. Read TOI [x,y,z] from logs or agent skills
"""

from __future__ import annotations

import argparse
import os
import sys


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run DimOS with RFID semantic particle-filter localization",
    )
    parser.add_argument(
        "--agentic",
        action="store_true",
        help="Include MCP server/client so agent skills "
        "(get_estimated_target_location, get_location_confidence) are callable.",
    )
    parser.add_argument(
        "--particles",
        type=int,
        default=None,
        help="Override RFID_PF_PARTICLES (default 5000).",
    )
    args = parser.parse_args(argv)

    if args.particles is not None:
        os.environ["RFID_PF_PARTICLES"] = str(args.particles)

    # Fail fast with a clear message if robot/API env is missing.
    robot_ip = os.environ.get("ROBOT_IP", "").strip()
    api_base = os.environ.get("RFID_API_BASE", "").strip()
    if not robot_ip:
        print(
            "Warning: ROBOT_IP is not set. Go2 WebRTC connection will likely fail.\n"
            "  export ROBOT_IP=<go2-wifi-ip>",
            file=sys.stderr,
        )
    if not api_base:
        print(
            "Warning: RFID_API_BASE is not set. Defaulting to localhost.\n"
            "  export RFID_API_BASE=http://<go2-wifi-ip>:8765/api/v1",
            file=sys.stderr,
        )

    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    if args.agentic:
        from dimos.agents.mcp.mcp_client import McpClient
        from dimos.agents.mcp.mcp_server import McpServer
        from dimos.core.coordination.blueprints import autoconnect
        from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import (
            _common_agentic,
        )
        from dimos_rfid.semantic_rfid_blueprints import unitree_go2_rfid_semantic

        blueprint = autoconnect(
            unitree_go2_rfid_semantic,
            McpServer.blueprint(),
            McpClient.blueprint(),
            _common_agentic,
        )
        print("Starting DimOS: unitree-go2 + RFID + semantic PF + agentic MCP …")
    else:
        from dimos_rfid.semantic_rfid_blueprints import unitree_go2_rfid_semantic

        blueprint = unitree_go2_rfid_semantic
        print("Starting DimOS: unitree-go2 + RFID + semantic particle filter …")

    print("  Skills (agentic): get_estimated_target_location, get_location_confidence")
    print("  Optional: RFID_SEMANTIC_MAP=/path/to/map.npz")
    print("Ctrl+C to stop.")
    ModuleCoordinator.build(blueprint).loop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
