#!/usr/bin/env python3
"""Run DimOS with Go2 + RFID + semantic particle-filter localization.

Usage (from the repository root):

    cp .env.example .env          # then edit GOOGLE_API_KEY / ROBOT_IP
    uv run python run_semantic_rfid.py --agentic

See SEMANTIC_LOCALIZER.md for how to run, query estimates, and set up MCP/agent.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def _load_dotenv() -> None:
    """Load repo-root ``.env`` into os.environ (does not override existing vars)."""
    env_path = Path(__file__).resolve().parent / ".env"
    if not env_path.is_file():
        return
    try:
        from dotenv import load_dotenv
    except ImportError:
        # Minimal fallback parser if python-dotenv is not installed.
        for line in env_path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, val = line.partition("=")
            key = key.strip()
            val = val.strip().strip("'").strip('"')
            if key and key not in os.environ:
                os.environ[key] = val
        return
    load_dotenv(env_path, override=False)


def _resolve_agent_model(cli_model: str | None) -> str:
    """Pick LLM id for McpClient; prefer Gemini for this project."""
    model = (cli_model or os.environ.get("RFID_AGENT_MODEL", "")).strip()
    if not model:
        # Default agentic stack to Gemini (not OpenAI).
        model = "google_genai:gemini-2.0-flash"
    return model


def main(argv: list[str] | None = None) -> int:
    _load_dotenv()

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
    parser.add_argument(
        "--model",
        default=None,
        help=(
            "LLM for McpClient when using --agentic. "
            "Default: google_genai:gemini-2.0-flash (needs GOOGLE_API_KEY). "
            "Also: gpt-4o (OPENAI_API_KEY), ollama:qwen3:8b."
        ),
    )
    args = parser.parse_args(argv)

    if args.particles is not None:
        os.environ["RFID_PF_PARTICLES"] = str(args.particles)

    robot_ip = os.environ.get("ROBOT_IP", "").strip()
    api_base = os.environ.get("RFID_API_BASE", "").strip()
    if not robot_ip:
        print(
            "Warning: ROBOT_IP is not set. Go2 WebRTC connection will likely fail.\n"
            "  Put it in .env or: export ROBOT_IP=<go2-wifi-ip>",
            file=sys.stderr,
        )
    if not api_base:
        print(
            "Warning: RFID_API_BASE is not set. Defaulting to localhost.\n"
            "  Put it in .env or: export RFID_API_BASE=http://<go2-wifi-ip>:8765/api/v1",
            file=sys.stderr,
        )

    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    if args.agentic:
        from dimos.agents.mcp.mcp_client import McpClient
        from dimos.agents.mcp.mcp_server import McpServer
        from dimos.core.coordination.blueprints import autoconnect

        from dimos_rfid.agentic_skills import rfid_agentic_skills
        from dimos_rfid.semantic_rfid_blueprints import unitree_go2_rfid_semantic

        model = _resolve_agent_model(args.model)
        google_key = os.environ.get("GOOGLE_API_KEY", "").strip()
        openai_key = os.environ.get("OPENAI_API_KEY", "").strip()

        if model.startswith("google_genai:") and not google_key:
            print(
                "Error: Gemini selected but GOOGLE_API_KEY is not set.\n"
                "  1) cp .env.example .env\n"
                "  2) Edit .env and set GOOGLE_API_KEY=…\n"
                "  3) Re-run: uv run python run_semantic_rfid.py --agentic\n"
                "Do not put the key in .env.example (that file can be committed).",
                file=sys.stderr,
            )
            return 2

        if model.startswith(("gpt-", "o1", "o3", "openai:")) and not openai_key:
            print(
                "Error: OpenAI model selected but OPENAI_API_KEY is not set.\n"
                "  Or use Gemini: put GOOGLE_API_KEY in .env and run:\n"
                "  uv run python run_semantic_rfid.py --agentic "
                "--model google_genai:gemini-2.0-flash",
                file=sys.stderr,
            )
            return 2

        # Do not use DimOS `_common_agentic`: it pulls WebInput → missing
        # dimos.web.dimos_interface on the PyPI wheel. CLI text commands use
        # `dimos agent-send` via McpServer.agent_send instead.
        blueprint = autoconnect(
            unitree_go2_rfid_semantic,
            McpServer.blueprint(),
            McpClient.blueprint(model=model),
            rfid_agentic_skills,
        )
        print("Starting DimOS: unitree-go2 + RFID + semantic PF + agentic MCP …")
        print(f"  LLM model: {model}")
        print(f"  GOOGLE_API_KEY set: {bool(google_key)}")
        print(f"  OPENAI_API_KEY set: {bool(openai_key)}")
        print('  Send text:  uv run dimos agent-send "where is RFID tag 8f?"')
        print("  Or skill:   uv run dimos mcp call get_estimated_target_location -a tag_id=8f")
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
