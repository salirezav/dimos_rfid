"""Find the RFID tags selected by ``rfid_module/rfid_focus.txt``.

This intentionally small agent reuses the robust localizer in
``rfid_module/rfid_module.py``.  It does not command robot motion: drive the
Go2 to five or more separated poses, stop at each pose long enough to collect
RSSI samples, and the agent reports the resulting world-frame location.

Run from the repository root:

    python Agent/rfid_finder_agent.py --go2

Add ``--agentic`` to expose ``find_focused_rfid_tag`` through DimOS MCP.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

# Executing this file directly puts Agent/ rather than the repository root on
# sys.path. Add the root so the existing standalone module can be imported.
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.utils.logging_config import setup_logger

from rfid_module.rfid_module import (
    DEFAULT_FOCUS_FILE,
    GO2_RFID_RERUN_ENTITY,
    RFIDConfig,
    RFIDModule,
    _go2_rfid_rerun_config,
)

logger = setup_logger()


class FocusedRfidFinderAgent(RFIDModule):
    """Report a usable world location for every EPC selected by the focus file."""

    _last_finder_signature: tuple[Any, ...] | None = None
    _last_active_epcs: set[str] | None = None

    def setup(self) -> None:
        self._last_finder_signature = None
        self._last_active_epcs = set()
        super().setup()
        logger.info(
            "RFID finder agent started; targets are read live from %s",
            self.config.focus_file or DEFAULT_FOCUS_FILE,
        )

    def _log_spatial(self, payload: dict) -> None:
        # The parent performs pose/RSSI collection, robust estimation, and Rerun
        # visualization. We only turn its result into a simple agent state.
        super()._log_spatial(payload)
        self._last_active_epcs = {
            str(tag.get("epc"))
            for tag in payload.get("tags", []) or []
            if tag.get("epc") and self._is_focused(str(tag["epc"]))
        }
        snapshot = self._finder_snapshot()
        signature = self._snapshot_signature(snapshot)
        if signature != self._last_finder_signature:
            self._last_finder_signature = signature
            print(self._human_status(snapshot), flush=True)

    @staticmethod
    def _snapshot_signature(snapshot: dict[str, Any]) -> tuple[Any, ...]:
        return (
            snapshot.get("state"),
            tuple(
                (
                    result.get("epc"),
                    result.get("state"),
                    result.get("observations"),
                    round(float(result.get("confidence", 0.0)), 2),
                )
                for result in snapshot.get("results", [])
            ),
        )

    def _finder_snapshot(self) -> dict[str, Any]:
        focus = self._focus
        patterns = focus.patterns() if focus is not None else []
        if not patterns:
            return {
                "state": "NEEDS_FOCUS",
                "focus": [],
                "results": [],
                "message": "Add an EPC or EPC suffix to rfid_module/rfid_focus.txt.",
            }

        localizers = self._locs or {}
        seen = self._seen_epcs or {}
        matching_epcs = sorted(
            epc
            for epc in set(localizers) | set(seen)
            if focus is not None and focus.matches(epc)
        )
        if not matching_epcs:
            return {
                "state": "SEARCHING",
                "focus": patterns,
                "results": [],
                "message": "Focused tag has not been detected. Move/rotate until it is in range.",
            }

        results: list[dict[str, Any]] = []
        for epc in matching_epcs:
            localizer = localizers.get(epc)
            active = epc in (self._last_active_epcs or set())
            if localizer is None or not localizer.obs:
                results.append(
                    {
                        "epc": epc,
                        "state": "COLLECTING",
                        "active": active,
                        "observations": 0,
                        "required_observations": self.config.min_observations,
                        "confidence": 0.0,
                        "position_world_m": None,
                    }
                )
                continue

            position, confidence, observations = localizer.estimate()
            usable = observations >= self.config.min_observations
            found = usable and confidence >= self.config.quality_blue
            results.append(
                {
                    "epc": epc,
                    "state": "FOUND" if found else ("REFINING" if usable else "COLLECTING"),
                    "active": active,
                    "observations": observations,
                    "required_observations": self.config.min_observations,
                    "confidence": round(float(confidence), 3),
                    "position_world_m": (
                        [round(float(value), 3) for value in position] if usable else None
                    ),
                    "frame": self.config.world_frame,
                }
            )

        if any(result["state"] == "FOUND" for result in results):
            state = "FOUND"
        elif any(result["state"] == "REFINING" for result in results):
            state = "REFINING"
        else:
            state = "COLLECTING"
        return {"state": state, "focus": patterns, "results": results}

    @staticmethod
    def _human_status(snapshot: dict[str, Any]) -> str:
        state = snapshot["state"]
        if not snapshot.get("results"):
            return f"[RFID FINDER] {state}: {snapshot.get('message', '')}"
        lines = [f"[RFID FINDER] {state}"]
        for result in snapshot["results"]:
            location = result.get("position_world_m")
            if location is None:
                detail = (
                    f"collecting poses {result['observations']}/"
                    f"{result['required_observations']}"
                )
            else:
                detail = (
                    f"world xyz={location} m, confidence={result['confidence']:.2f}, "
                    f"n={result['observations']}"
                )
            lines.append(f"  {result['epc']}: {result['state']} — {detail}")
        return "\n".join(lines)

    @rpc
    def get_focused_tag_location(self) -> dict[str, Any]:
        """Return machine-readable state and world coordinates for focused tags."""
        return self._finder_snapshot()

    @skill
    def find_focused_rfid_tag(self) -> str:
        """Find RFID tags listed in rfid_focus.txt and report their world location.

        The result explains whether the tag is still being searched for, needs
        more independent robot poses, or has a usable robust location estimate.
        """
        snapshot = self._finder_snapshot()
        return self._human_status(snapshot) + "\n\n" + json.dumps(snapshot, indent=2)


def run(*, agentic: bool = False) -> None:
    """Run the Go2 stack with the focused RFID finder agent."""
    # These imports construct network-backed DimOS blueprint objects, so keep
    # them out of module import/--help and load them only for a real run.
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    config = RFIDConfig(
        rerun=True,
        rerun_spawn=False,
        rerun_entity=GO2_RFID_RERUN_ENTITY,
        spatial=True,
        focus_file=DEFAULT_FOCUS_FILE,
        focus_only_localize=True,
    )
    components: list[Any] = [
        unitree_go2,
        FocusedRfidFinderAgent.blueprint(config=config),
        RerunBridgeModule.blueprint(**_go2_rfid_rerun_config()),
    ]
    if agentic:
        # Imported lazily so the ordinary deterministic finder stays lightweight.
        from dimos.agents.mcp.mcp_client import McpClient
        from dimos.agents.mcp.mcp_server import McpServer
        from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import _common_agentic

        components.extend([McpServer.blueprint(), McpClient.blueprint(), _common_agentic])

    coordinator = ModuleCoordinator.build(autoconnect(*components))
    try:
        coordinator.loop()
    except KeyboardInterrupt:
        coordinator.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Find RFID tags selected by rfid_module/rfid_focus.txt."
    )
    parser.add_argument(
        "--go2",
        action="store_true",
        help="Compatibility flag; the finder always runs with the Go2 stack.",
    )
    parser.add_argument(
        "--agentic",
        action="store_true",
        help="Also expose find_focused_rfid_tag through the DimOS MCP agent platform.",
    )
    arguments = parser.parse_args()
    run(agentic=arguments.agentic)
