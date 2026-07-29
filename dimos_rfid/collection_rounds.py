# Copyright 2026. RFID DimOS integration — multi-round power-ladder collection.

"""Drive a predetermined waypoint path once per TX power and record a dataset."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

from dimos_rfid.collection_context import write_collection_context
from dimos_rfid.msgs import RfidTagArray
from dimos_rfid.rfid_power import (
    RfidPowerError,
    get_power,
    parse_power_ladder,
    set_read_power,
)
from dimos_rfid.rfid_rerun import COLLECTION_RERUN_ENTITY

logger = setup_logger()

DEFAULT_PATH = str(Path(__file__).resolve().parent / "paths" / "example_loop.json")
DEFAULT_LADDER = "30,25,20,15"


@dataclass
class Waypoint:
    x: float
    y: float
    z: float = 0.0
    yaw: float | None = None
    frame_id: str = "world"

    def to_pose(self) -> PoseStamped:
        if self.yaw is None:
            orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        else:
            orientation = Quaternion.from_euler(Vector3(0.0, 0.0, float(self.yaw)))
        return PoseStamped(
            position=Vector3(float(self.x), float(self.y), float(self.z)),
            orientation=orientation,
            frame_id=self.frame_id,
            ts=time.time(),
        )


def load_waypoints(path: str | Path) -> tuple[str, list[Waypoint]]:
    """Load a path JSON file. Returns (path_id, waypoints)."""
    file_path = Path(path).expanduser().resolve()
    data = json.loads(file_path.read_text(encoding="utf-8"))
    if isinstance(data, list):
        raw_points = data
        path_id = file_path.stem
        frame_id = "world"
    elif isinstance(data, dict):
        raw_points = data.get("waypoints") or data.get("points") or []
        path_id = str(data.get("path_id") or data.get("name") or file_path.stem)
        frame_id = str(data.get("frame_id") or "world")
    else:
        raise ValueError(f"Unsupported path JSON in {file_path}")

    waypoints: list[Waypoint] = []
    for item in raw_points:
        if not isinstance(item, dict):
            raise ValueError(f"Waypoint must be an object, got {item!r}")
        waypoints.append(
            Waypoint(
                x=float(item["x"]),
                y=float(item["y"]),
                z=float(item.get("z", 0.0)),
                yaw=(float(item["yaw"]) if item.get("yaw") is not None else None),
                frame_id=str(item.get("frame_id") or frame_id),
            )
        )
    if not waypoints:
        raise ValueError(f"No waypoints in {file_path}")
    return path_id, waypoints


class RfidCollectionRoundsConfig(ModuleConfig):
    """Configuration for :class:`RfidCollectionRoundsModule`."""

    api_base: str = Field(
        default_factory=lambda: os.environ.get(
            "RFID_API_BASE", "http://localhost:8765/api/v1"
        )
    )
    path_file: str = Field(
        default_factory=lambda: os.environ.get("RFID_COLLECTION_PATH", DEFAULT_PATH),
        description="JSON file with predetermined waypoints.",
    )
    power_ladder: str = Field(
        default_factory=lambda: os.environ.get("RFID_POWER_LADDER", DEFAULT_LADDER),
        description="Comma-separated read_power dBm values, one round each.",
    )
    auto_start: bool = Field(
        default_factory=lambda: os.environ.get(
            "RFID_COLLECTION_AUTO_START", "1"
        ).strip().lower()
        in {"1", "true", "yes", "on"},
    )
    session_name: str = Field(
        default_factory=lambda: os.environ.get("RFID_DATASET_SESSION", ""),
    )
    arrival_tolerance_m: float = Field(default=0.45, gt=0.05)
    waypoint_timeout_s: float = Field(default=90.0, gt=1.0)
    settle_s: float = Field(
        default=2.0,
        ge=0.0,
        description="Pause after setting power before driving the path.",
    )
    log_hz: float = Field(default=1.0, gt=0.0)
    max_log_lines: int = Field(default=40, ge=5)


class RfidCollectionRoundsModule(Module):
    """Multi-round dataset collection: set power → drive path → next power.

    Publishes navigation goals on ``goal_request`` (same stream as the command
    center / patrolling stack). Stamps the recorder with round metadata and
    refreshes a Rerun markdown panel with a live collection log.
    """

    config: RfidCollectionRoundsConfig
    odom: In[PoseStamped]
    rfid_samples: In[RfidTagArray]
    goal_request: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._worker: threading.Thread | None = None
        self._stop = threading.Event()
        self._pause = threading.Event()  # set => paused
        self._skip_round = threading.Event()
        self._advance_waypoint = threading.Event()
        self._running = False
        self._latest_pose: PoseStamped | None = None
        self._path_id = ""
        self._waypoints: list[Waypoint] = []
        self._ladder: list[float] = []
        self._round_index = 0
        self._waypoint_index = 0
        self._current_power: float | None = None
        self._state = "idle"
        self._message = "Idle"
        self._log: deque[str] = deque(maxlen=int(self.config.max_log_lines))
        self._sample_count = 0
        self._last_panel_ts = 0.0
        self._rerun_connected = False
        self._rpc_client: Any = None

    def _api_base(self) -> str:
        return os.environ.get("RFID_API_BASE", self.config.api_base).rstrip("/")

    def _find_recorder(self) -> Any | None:
        return None

    def _recorder_rpc(self, method: str, *args: Any, **kwargs: Any) -> dict[str, Any]:
        """Call RfidRecorderModule RPCs over DimOS LCM RPC (cross-process)."""
        try:
            from dimos.protocol.rpc.pubsubrpc import LCMRPC

            if self._rpc_client is None:
                client = LCMRPC()
                client.start()
                self._rpc_client = client
            result, _unsub = self._rpc_client.call_sync(
                f"RfidRecorderModule/{method}",
                (args, kwargs),
            )
            if isinstance(result, dict):
                return result
            return {"ok": True, "result": result}
        except Exception as exc:  # noqa: BLE001
            logger.debug("Recorder RPC %s failed: %s", method, exc)
            return {"ok": False, "message": str(exc)}

    @rpc
    def set_recorder(self, recorder: Any) -> dict[str, Any]:
        """Compatibility no-op (modules run in separate processes; use LCM RPC)."""
        return {"ok": True, "attached": False, "message": "Use LCM RPC / context file"}

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(
            Disposable(self.rfid_samples.subscribe(self._on_rfid_sample))
        )
        try:
            self._path_id, self._waypoints = load_waypoints(self.config.path_file)
            self._ladder = parse_power_ladder(self.config.power_ladder)
            self._append_log(
                f"Loaded path={self._path_id} "
                f"({len(self._waypoints)} waypoints), ladder={self._ladder}"
            )
        except Exception as exc:  # noqa: BLE001
            self._state = "error"
            self._message = f"Failed to load path/ladder: {exc}"
            logger.error("%s", self._message)
            self._refresh_panel(force=True)
            return

        self._refresh_panel(force=True)
        if self.config.auto_start:
            self.start_rounds(self.config.session_name)

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._worker and self._worker.is_alive():
            self._worker.join(timeout=5.0)
        self._running = False
        super().stop()

    def _on_odom(self, pose: PoseStamped) -> None:
        with self._lock:
            self._latest_pose = pose

    def _on_rfid_sample(self, sample: RfidTagArray) -> None:
        with self._lock:
            self._sample_count += 1
            if not self._running:
                return
            active = sample.active_tags()
            if not active:
                return
            pose = self._latest_pose
            xy = ""
            if pose is not None:
                xy = f" @({pose.position.x:.2f},{pose.position.y:.2f})"
            top = sorted(
                active,
                key=lambda t: t.rssi_dbm if t.rssi_dbm is not None else -999,
                reverse=True,
            )[:3]
            parts = []
            for tag in top:
                epc = tag.epc[-8:] if tag.epc else "?"
                rssi = f"{tag.rssi_dbm:.0f}" if tag.rssi_dbm is not None else "?"
                parts.append(f"{epc}:{rssi}dBm")
            self._append_log(
                f"r{self._round_index + 1} p={self._current_power} "
                f"n={len(active)} [{', '.join(parts)}]{xy}"
            )
        self._refresh_panel()

    def _append_log(self, line: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self._log.appendleft(f"`{stamp}` {line}")

    def _set_state(self, state: str, message: str) -> None:
        with self._lock:
            self._state = state
            self._message = message
            self._append_log(f"{state}: {message}")
        logger.info("Collection %s — %s", state, message)
        self._refresh_panel(force=True)

    def _ensure_rerun(self) -> Any | None:
        try:
            import rerun as rr
            from dimos.core.global_config import global_config
            from dimos.visualization.rerun.bridge import RERUN_GRPC_PORT

            if not self._rerun_connected:
                rr.init("dimos")
                host = (
                    getattr(global_config, "rerun_host", None)
                    or getattr(global_config, "listen_host", None)
                    or "127.0.0.1"
                )
                rr.connect_grpc(f"rerun+http://{host}:{RERUN_GRPC_PORT}/proxy")
                self._rerun_connected = True
            return rr
        except Exception as exc:  # noqa: BLE001
            self._rerun_connected = False
            logger.debug("Collection Rerun connect failed: %s", exc)
            return None

    def _panel_markdown(self) -> str:
        with self._lock:
            lines = [
                "# RFID collection",
                "",
                f"**State:** `{self._state}`  ",
                f"**Message:** {self._message}",
                "",
                f"**Round:** {self._round_index + 1}/{max(len(self._ladder), 1)}  "
                f"· **Power:** {self._current_power if self._current_power is not None else '—'} dBm  ",
                f"**Waypoint:** {self._waypoint_index + 1}/{max(len(self._waypoints), 1)}  "
                f"· **Path:** `{self._path_id or '—'}`  ",
                f"**Samples seen:** {self._sample_count}  "
                f"· **Paused:** {self._pause.is_set()}",
                "",
                "## Ladder",
                "",
                ", ".join(f"**{p}**" if i == self._round_index else str(p)
                          for i, p in enumerate(self._ladder))
                or "_none_",
                "",
                "## Live log",
                "",
            ]
            if self._log:
                lines.extend(f"- {entry}" for entry in list(self._log)[: self.config.max_log_lines])
            else:
                lines.append("_Waiting for RFID samples…_")
            lines.extend(
                [
                    "",
                    "---",
                    "_RPCs:_ `start_rounds`, `pause_rounds`, `resume_rounds`, "
                    "`skip_round`, `advance_waypoint`, `set_power_now`, `get_collection_status`",
                ]
            )
            return "\n".join(lines)

    def _refresh_panel(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_panel_ts) < (1.0 / max(self.config.log_hz, 1e-3)):
            return
        self._last_panel_ts = now
        rr = self._ensure_rerun()
        if rr is None:
            return
        text = self._panel_markdown()
        try:
            rr.log(
                COLLECTION_RERUN_ENTITY,
                rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN),
            )
        except (AttributeError, TypeError):
            rr.log(COLLECTION_RERUN_ENTITY, rr.TextLog(text))
        except Exception as exc:  # noqa: BLE001
            self._rerun_connected = False
            logger.debug("Collection panel log failed: %s", exc)

    def _call_recorder(self, method: str, *args: Any, **kwargs: Any) -> dict[str, Any]:
        return self._recorder_rpc(method, *args, **kwargs)

    def _wait_while_paused(self) -> bool:
        """Block while paused. Returns False if stop requested."""
        while self._pause.is_set():
            if self._stop.is_set():
                return False
            time.sleep(0.1)
        return not self._stop.is_set()

    def _distance_xy(self, pose: PoseStamped, waypoint: Waypoint) -> float:
        return math.hypot(pose.position.x - waypoint.x, pose.position.y - waypoint.y)

    def _drive_waypoint(self, waypoint: Waypoint, index: int) -> bool:
        """Publish a goal and wait for arrival / timeout / skip. False => abort rounds."""
        self._waypoint_index = index
        goal = waypoint.to_pose()
        self._set_state(
            "driving",
            f"Waypoint {index + 1}/{len(self._waypoints)} "
            f"→ ({waypoint.x:.2f}, {waypoint.y:.2f})",
        )
        self.goal_request.publish(goal)
        deadline = time.monotonic() + float(self.config.waypoint_timeout_s)
        while time.monotonic() < deadline:
            if self._stop.is_set():
                return False
            if self._skip_round.is_set():
                return True
            if self._advance_waypoint.is_set():
                self._advance_waypoint.clear()
                self._append_log(f"Manually advanced past waypoint {index + 1}")
                return True
            if not self._wait_while_paused():
                return False
            with self._lock:
                pose = self._latest_pose
            if pose is not None and self._distance_xy(pose, waypoint) <= self.config.arrival_tolerance_m:
                self._append_log(f"Arrived waypoint {index + 1}")
                return True
            time.sleep(0.2)
        self._append_log(
            f"Waypoint {index + 1} timeout after {self.config.waypoint_timeout_s:.0f}s — continuing"
        )
        return True

    def _run_rounds(self, session_name: str) -> None:
        try:
            self._running = True
            self._stop.clear()
            self._skip_round.clear()
            self._pause.clear()

            # Start recording if not already on (dataset blueprint may auto-start).
            status = self._call_recorder("get_recording_status")
            if not status.get("recording"):
                started = self._call_recorder(
                    "start_recording",
                    session_name or self.config.session_name,
                    {
                        "mode": "power_ladder_rounds",
                        "path_id": self._path_id,
                        "power_ladder": list(self._ladder),
                        "path_file": str(self.config.path_file),
                    },
                )
                if not started.get("ok", True) and "already recording" not in str(
                    started.get("message", "")
                ).lower():
                    self._set_state("error", f"Could not start recorder: {started}")
                    return
            else:
                self._call_recorder(
                    "update_user_metadata",
                    {
                        "mode": "power_ladder_rounds",
                        "path_id": self._path_id,
                        "power_ladder": list(self._ladder),
                        "path_file": str(self.config.path_file),
                    },
                )

            for round_i, power in enumerate(self._ladder):
                if self._stop.is_set():
                    break
                self._round_index = round_i
                self._skip_round.clear()
                self._current_power = power
                self._set_state("setting_power", f"Setting read_power={power} dBm")
                try:
                    set_read_power(self._api_base(), power)
                except RfidPowerError as exc:
                    self._set_state("error", f"Power set failed: {exc}")
                    break

                context = {
                    "round_index": round_i,
                    "round_count": len(self._ladder),
                    "read_power_dbm": power,
                    "path_id": self._path_id,
                    "waypoint_count": len(self._waypoints),
                }
                write_collection_context(context)
                self._call_recorder("set_collection_context", context)
                self._call_recorder(
                    "update_user_metadata",
                    {
                        "latest_collection": context,
                        "path_id": self._path_id,
                        "power_ladder": list(self._ladder),
                    },
                )

                if self.config.settle_s > 0:
                    self._set_state("settling", f"Settling {self.config.settle_s:.1f}s at {power} dBm")
                    time.sleep(self.config.settle_s)

                for wp_i, waypoint in enumerate(self._waypoints):
                    if self._stop.is_set() or self._skip_round.is_set():
                        break
                    if not self._drive_waypoint(waypoint, wp_i):
                        self._set_state("stopped", "Collection stopped")
                        return

                if self._skip_round.is_set():
                    self._append_log(f"Skipped remainder of round {round_i + 1}")
                    continue

            if not self._stop.is_set():
                self._set_state("finishing", "Finalizing dataset session")
                write_collection_context(None)
                result = self._call_recorder("stop_recording")
                if result.get("ok"):
                    self._set_state(
                        "done",
                        f"Session ready: {result.get('session_dir') or result.get('archive_path')}",
                    )
                else:
                    self._set_state(
                        "done",
                        "Rounds complete — press Ctrl+C to finalize the ZIP if still recording "
                        f"({result.get('message', 'no recorder RPC')})",
                    )
            else:
                write_collection_context(None)
                self._set_state("stopped", "Collection stopped by operator")
        except Exception as exc:  # noqa: BLE001
            logger.exception("Collection rounds failed")
            self._set_state("error", str(exc))
        finally:
            self._running = False
            self._worker = None
            self._refresh_panel(force=True)

    @rpc
    def start_rounds(self, session_name: str = "") -> dict[str, Any]:
        """Begin the power-ladder rounds (no-op if already running)."""
        if self._worker and self._worker.is_alive():
            return {"ok": False, "message": "Collection rounds already running."}
        if not self._waypoints or not self._ladder:
            try:
                self._path_id, self._waypoints = load_waypoints(self.config.path_file)
                self._ladder = parse_power_ladder(self.config.power_ladder)
            except Exception as exc:  # noqa: BLE001
                return {"ok": False, "message": str(exc)}
        self._worker = threading.Thread(
            target=self._run_rounds,
            args=(session_name,),
            name="rfid-collection-rounds",
            daemon=True,
        )
        self._worker.start()
        return {
            "ok": True,
            "path_id": self._path_id,
            "waypoints": len(self._waypoints),
            "ladder": list(self._ladder),
        }

    @rpc
    def pause_rounds(self) -> dict[str, Any]:
        self._pause.set()
        self._set_state("paused", "Paused — call resume_rounds to continue")
        return {"ok": True, "paused": True}

    @rpc
    def resume_rounds(self) -> dict[str, Any]:
        self._pause.clear()
        self._set_state("running", "Resumed")
        return {"ok": True, "paused": False}

    @rpc
    def skip_round(self) -> dict[str, Any]:
        self._skip_round.set()
        self._advance_waypoint.set()
        self._append_log("Skip round requested")
        self._refresh_panel(force=True)
        return {"ok": True}

    @rpc
    def advance_waypoint(self) -> dict[str, Any]:
        """Manually advance to the next waypoint (arrival fallback)."""
        self._advance_waypoint.set()
        return {"ok": True}

    @rpc
    def set_power_now(self, read_power: float) -> dict[str, Any]:
        """Immediately set reader TX power (does not change ladder schedule)."""
        try:
            payload = set_read_power(self._api_base(), float(read_power))
            self._current_power = float(payload.get("read_power", read_power))
            self._append_log(f"Manual power → {self._current_power} dBm")
            self._refresh_panel(force=True)
            return {"ok": True, **payload}
        except (RfidPowerError, ValueError) as exc:
            return {"ok": False, "message": str(exc)}

    @rpc
    def get_collection_status(self) -> dict[str, Any]:
        with self._lock:
            power_live: dict[str, Any] | None = None
            try:
                power_live = get_power(self._api_base())
            except RfidPowerError:
                power_live = None
            return {
                "ok": True,
                "running": self._running,
                "paused": self._pause.is_set(),
                "state": self._state,
                "message": self._message,
                "round_index": self._round_index,
                "round_count": len(self._ladder),
                "current_power": self._current_power,
                "waypoint_index": self._waypoint_index,
                "waypoint_count": len(self._waypoints),
                "path_id": self._path_id,
                "ladder": list(self._ladder),
                "sample_count": self._sample_count,
                "reader_power": power_live,
                "recorder": self._call_recorder("get_recording_status"),
            }


__all__ = [
    "RfidCollectionRoundsConfig",
    "RfidCollectionRoundsModule",
    "Waypoint",
    "load_waypoints",
]
