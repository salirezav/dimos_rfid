# Copyright 2026. Call RfidRecorderModule waypoint / path-recording RPCs over LCM.

"""Capture and replay Go2 routes.

Requires a running dataset stack (``go2-dataset`` or ``go2-dataset-rounds``).

Manual single points::

    uv run python -m dimos_rfid.waypoint_cli mark --path dimos_rfid/paths/lab_loop.json
    uv run python -m dimos_rfid.waypoint_cli list --path dimos_rfid/paths/lab_loop.json

Teleop path recording (auto-sample while you drive)::

    uv run python -m dimos_rfid.waypoint_cli record-start --path dimos_rfid/paths/lab_loop.json
    # drive with Keyboard Teleop …
    uv run python -m dimos_rfid.waypoint_cli record-status
    uv run python -m dimos_rfid.waypoint_cli record-stop

Replay later::

    # RFID_COLLECTION_PATH=dimos_rfid/paths/lab_loop.json
    uv run python -m dimos_rfid go2-dataset-rounds
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any


def _call(method: str, **kwargs: Any) -> dict[str, Any]:
    from dimos.protocol.rpc.pubsubrpc import LCMRPC

    from dimos_rfid.recorder import RfidRecorderModule

    # DimOS serves RPCs as ``{ClassName}/{method}`` (PascalCase), not lowercased.
    rpc_name = f"{RfidRecorderModule.__name__}/{method}"
    client = LCMRPC()
    client.start()
    try:
        result, _unsub = client.call_sync(
            rpc_name,
            ((), kwargs),
            rpc_timeout=10.0,
        )
    finally:
        try:
            client.stop()
        except Exception:  # noqa: BLE001
            pass
    if isinstance(result, dict):
        return result
    return {"ok": True, "result": result}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Capture Go2 waypoints / teleop paths into a JSON file"
    )
    parser.add_argument(
        "command",
        choices=[
            "mark",
            "list",
            "clear",
            "record-start",
            "record-stop",
            "record-status",
        ],
        help=(
            "mark=one pose, record-start/stop=auto path while teleoping, "
            "list/clear=inspect file"
        ),
    )
    parser.add_argument(
        "--path",
        default="",
        help="Path JSON file (default: RFID_WAYPOINT_CAPTURE_PATH / captured_path.json)",
    )
    parser.add_argument(
        "--path-id",
        default="",
        help="Optional path_id stored in the JSON (record-start only)",
    )
    parser.add_argument(
        "--append",
        action="store_true",
        help="record-start: continue an existing file instead of clearing it",
    )
    args = parser.parse_args(argv)

    try:
        if args.command == "mark":
            result = _call("mark_waypoint", path_file=args.path)
        elif args.command == "list":
            result = _call("get_waypoints", path_file=args.path)
        elif args.command == "clear":
            result = _call("clear_waypoints", path_file=args.path)
        elif args.command == "record-start":
            result = _call(
                "start_path_recording",
                path_file=args.path,
                path_id=args.path_id,
                clear=not args.append,
            )
        elif args.command == "record-stop":
            result = _call("stop_path_recording")
        else:
            result = _call("get_path_recording_status")
    except Exception as exc:  # noqa: BLE001
        print(
            f"RPC failed: {exc}\n"
            "Is DimOS running? Start with: uv run python -m dimos_rfid go2-dataset",
            file=sys.stderr,
        )
        return 1
    print(json.dumps(result, indent=2))
    return 0 if result.get("ok", True) else 2


if __name__ == "__main__":
    raise SystemExit(main())
