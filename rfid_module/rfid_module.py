"""Single-file DimOS module that polls an RFID scanner HTTP API.

Asynchronously polls a local RFID API every `interval` seconds and prints the
active tag count, EPC, and RSSI to the console.

Ways to run it natively (all bypass the `dimos` CLI daemon):

    python rfid_module.py                # in-process  -> RECOMMENDED for the debugger
    python rfid_module.py --coordinator  # via ModuleCoordinator (forks a worker process)
    python rfid_module.py --ui           # standalone: opens the viewer, RFID panel only
    python rfid_module.py --go2          # full Go2 view (camera | 3D lidar | RFID)

`--ui` opens the same viewer `dimos run` uses (`dimos-viewer`) with just the RFID
panel. The module owns the viewer in its own process so it renders reliably.

`--go2` runs the real Go2 stack (camera + 3D lidar) and overrides its Rerun
layout with a camera | 3D | RFID panel. The Go2 modules feed camera/lidar to the
viewer; this module logs its tag text to the RFID panel. Requires the robot to be
reachable (same as `dimos run unitree-go2`).

`--go2` also drops a spatial marker per tag. RFID gives only signal strength, so
a tag can't be located from one reading; as the dog is driven around and re-sees
the same EPC from different positions, the RSSI values are multilaterated into a
world position (gray "estimating" -> blue "refining" -> green "located"). Readings
are only recorded while the dog is stationary; at each stop the module collects
multiple RSSI samples, discards outliers, and uses the median before solving.
In crowded environments, edit ``rfid_focus.txt`` (one EPC/suffix per line) to
focus the UI on selected tags and hide the rest. The marker also appears as a 2D
overlay on the camera image, but only when it projects inside the frame — i.e.
when the dog is looking toward where the tag was found.

Why the default is in-process
-----------------------------
`ModuleCoordinator.build()` deploys each module into a *forkserver child
process* (DimOS always runs a worker pool; there is no in-process worker mode).
On Python 3.12 (this workspace), debugpy has known bugs where stepping
(F10/F11) inside fork/forkserver child processes behaves like "continue", so
breakpoints in the async loop are unreliable there.

Running in-process instead executes the async loop in a background *thread* of
this same process, which the debugger handles perfectly: breakpoints, stepping,
and variable inspection in `_poll_loop` / `_print_tags` all work.

Notes on this DimOS build (see the workspace `.venv`)
----------------------------------------------------
- The lifecycle hook the coordinator calls is `start()`, not `setup()`. To keep
  the requested `setup()` + `self.spawn()` shape, `start()` calls `self.setup()`.
- DimOS builds a module's config from field kwargs (`config_type(**kwargs)` with
  `extra="forbid"`). `__init__` below accepts a `config=` object and expands it
  into fields so `RFIDModule.blueprint(config=RFIDConfig())` works as requested.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import threading
import time
from concurrent.futures import Future
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import httpx
import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_URL = "http://10.42.200.240:8765/api/v1/tags/active"

# Marker quality tiers (see ``_quality_color`` / ``_quality_state``).
QUALITY_BLUE = 0.4
QUALITY_GREEN = 0.85


@dataclass
class _Obs:
    """One finalized RFID sighting at a stationary anchor point."""

    pos: np.ndarray  # dog position (x, y, z) in the world frame at sighting
    rssi: float  # filtered median RSSI in dBm at this anchor
    ts: float
    n_samples: int = 1  # raw RSSI samples that contributed to this anchor


@dataclass
class _Anchor:
    """RSSI samples collected while the dog is stationary at one position."""

    pos: np.ndarray
    rssi_samples: list[float] = field(default_factory=list)
    last_ts: float = 0.0

    def add_sample(self, rssi: float, ts: float) -> None:
        self.rssi_samples.append(float(rssi))
        self.last_ts = ts


def _filter_rssi_outliers(samples: list[float]) -> list[float]:
    """Drop noisy RSSI outliers (IQR rule); keep all if too few to filter."""
    if len(samples) < 4:
        return samples
    arr = np.asarray(samples, dtype=float)
    q1, q3 = np.percentile(arr, [25, 75])
    iqr = float(q3 - q1)
    if iqr < 0.5:
        return samples
    lo, hi = q1 - 1.5 * iqr, q3 + 1.5 * iqr
    filtered = [s for s in samples if lo <= s <= hi]
    return filtered if filtered else samples


@dataclass
class _DogMotionTracker:
    """Tracks whether the dog is stationary enough to record RFID readings."""

    stationary_speed_mps: float = 0.05
    _prev_pos: np.ndarray | None = None
    _prev_ts: float | None = None
    is_stationary: bool = False

    def update(self, pos: np.ndarray, ts: float) -> bool:
        """Update motion state; return True when the dog is stationary."""
        pos = np.asarray(pos, dtype=float)
        if self._prev_pos is None or self._prev_ts is None:
            self._prev_pos = pos
            self._prev_ts = ts
            self.is_stationary = False
            return False

        dt = max(ts - self._prev_ts, 1e-3)
        speed = float(np.linalg.norm(pos - self._prev_pos) / dt)
        self._prev_pos = pos
        self._prev_ts = ts
        self.is_stationary = speed <= self.stationary_speed_mps
        return self.is_stationary


@dataclass
class _FocusFilter:
    """Selects which EPCs to show/localize; empty patterns = show everything.

    Patterns match case-insensitively as substrings, so a short suffix like
    ``8f`` focuses the full EPC ending in ``…8f``. Combine ``config.focus_epcs``
    with a live-watched ``focus_file`` (one EPC/suffix per line).
    """

    config_patterns: list[str] = field(default_factory=list)
    focus_file: str = ""
    _file_patterns: list[str] = field(default_factory=list)
    _file_mtime: float | None = None
    _rpc_patterns: list[str] | None = None  # None = unset; [] = clear via RPC

    def set_rpc_focus(self, patterns: list[str] | None) -> None:
        """Override focus from an RPC call. ``None`` clears the RPC override."""
        self._rpc_patterns = None if patterns is None else [p.strip() for p in patterns if p.strip()]

    def _parse_file(self, path: Path) -> list[str]:
        patterns: list[str] = []
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            return []
        for line in text.splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # Allow comma-separated EPCs on one line.
            for part in line.split(","):
                part = part.strip()
                if part and not part.startswith("#"):
                    patterns.append(part)
        return patterns

    def _reload_file_if_needed(self) -> None:
        if not self.focus_file:
            self._file_patterns = []
            self._file_mtime = None
            return
        path = Path(self.focus_file)
        if not path.is_file():
            self._file_patterns = []
            self._file_mtime = None
            return
        try:
            mtime = path.stat().st_mtime
        except OSError:
            return
        if self._file_mtime is not None and mtime == self._file_mtime:
            return
        self._file_mtime = mtime
        self._file_patterns = self._parse_file(path)

    def patterns(self) -> list[str]:
        self._reload_file_if_needed()
        if self._rpc_patterns is not None:
            return list(self._rpc_patterns)
        out: list[str] = []
        seen: set[str] = set()
        for p in [*self.config_patterns, *self._file_patterns]:
            key = p.lower()
            if key not in seen:
                seen.add(key)
                out.append(p)
        return out

    @property
    def active(self) -> bool:
        return bool(self.patterns())

    def matches(self, epc: str) -> bool:
        """True if this EPC should be shown (always True when filter inactive)."""
        pats = self.patterns()
        if not pats:
            return True
        epc_l = epc.lower()
        return any(p.lower() in epc_l for p in pats)


@dataclass
class _TagLocalizer:
    """Estimates a tag's world position from RSSI seen at multiple dog poses.

    A single UHF-RFID reading gives only signal strength (no bearing, no range),
    so one sighting can't localize a tag. As the dog observes the same EPC from
    several *spatially distinct* stationary positions, each anchor's RSSI samples
    are outlier-filtered and reduced to a median, converted to a rough range via a
    log-distance path-loss model, and the tag position is solved by linear
    least-squares multilateration (in the ground plane; z is taken as the mean
    sighting height since a ground robot barely changes altitude).

    Until there are enough well-spread sightings, ``estimate()`` returns a
    low-quality "placeholder" (the RSSI-weighted centroid of the dog poses).
    """

    rssi_ref_dbm: float = -50.0  # expected RSSI at 1 m (reference power)
    path_loss_n: float = 2.0  # path-loss exponent (2 free space, 2-4 indoor)
    min_baseline_m: float = 0.3  # min dog travel before a new anchor
    min_rssi_samples: int = 3  # prefer this many samples before closing an anchor
    max_range_m: float = 15.0  # reject solutions this far from the sightings
    obs: list[_Obs] = field(default_factory=list)
    _current_anchor: _Anchor | None = None

    def add_stationary_sample(self, pos: np.ndarray, rssi: float, ts: float) -> None:
        """Accumulate RSSI while the dog is stopped at (or near) an anchor point."""
        pos = np.asarray(pos, dtype=float)
        anchor = self._current_anchor
        if anchor is None or np.linalg.norm(pos - anchor.pos) >= self.min_baseline_m:
            self.finalize_current_anchor()
            self._current_anchor = _Anchor(pos=pos.copy(), last_ts=ts)
            anchor = self._current_anchor
        anchor.add_sample(rssi, ts)

    def finalize_current_anchor(self) -> None:
        """Close the open anchor: filter outliers, median RSSI, append to obs."""
        anchor = self._current_anchor
        self._current_anchor = None
        if anchor is None or not anchor.rssi_samples:
            return
        filtered = _filter_rssi_outliers(anchor.rssi_samples)
        if len(filtered) < self.min_rssi_samples and len(anchor.rssi_samples) < self.min_rssi_samples:
            return
        rssi = float(np.median(filtered))
        # Replace the previous obs from the same spot if we re-stopped nearby.
        if self.obs and np.linalg.norm(anchor.pos - self.obs[-1].pos) < self.min_baseline_m:
            self.obs[-1] = _Obs(anchor.pos, rssi, anchor.last_ts, len(filtered))
        else:
            self.obs.append(_Obs(anchor.pos, rssi, anchor.last_ts, len(filtered)))

    def _rssi_to_distance(self, rssi: np.ndarray) -> np.ndarray:
        """Log-distance path-loss model: d = 10 ** ((rssi_ref - rssi) / (10 n))."""
        return 10.0 ** ((self.rssi_ref_dbm - rssi) / (10.0 * self.path_loss_n))

    def estimate(self) -> tuple[np.ndarray, float, int]:
        """Return (position_xyz, quality[0..1], n_observations).

        quality < QUALITY_BLUE: centroid placeholder ("estimating").
        QUALITY_BLUE..QUALITY_GREEN: multilateration ("refining").
        quality >= QUALITY_GREEN: high-confidence multilateration ("located").
        """
        n = len(self.obs)
        pts = np.array([o.pos for o in self.obs])
        rssi = np.array([o.rssi for o in self.obs])

        # RSSI-weighted centroid (linear power weights) — robust fallback/prior.
        weights = 10.0 ** (rssi / 10.0)
        centroid = (weights[:, None] * pts).sum(axis=0) / weights.sum()

        if n < 3:
            return centroid, min(0.35, 0.1 * n), n

        # Linear least-squares multilateration in the ground plane (x, y).
        dist = self._rssi_to_distance(rssi)
        ref = int(np.argmax(rssi))  # anchor on the strongest (closest) sighting
        others = [i for i in range(n) if i != ref]
        a_mat = 2.0 * (pts[others, :2] - pts[ref, :2])
        b_vec = (
            (pts[others, :2] ** 2).sum(axis=1)
            - (pts[ref, :2] ** 2).sum()
            - dist[others] ** 2
            + dist[ref] ** 2
        )
        try:
            sol, _res, rank, _sv = np.linalg.lstsq(a_mat, b_vec, rcond=None)
        except np.linalg.LinAlgError:
            return centroid, 0.25, n
        if rank < 2 or not np.all(np.isfinite(sol)):
            return centroid, 0.25, n

        est = np.array([sol[0], sol[1], float(pts[:, 2].mean())])
        if np.linalg.norm(est[:2] - pts[:, :2].mean(axis=0)) > self.max_range_m:
            return centroid, 0.25, n

        # Residual-based fit quality: lower residual -> higher confidence.
        residuals = []
        for i in range(n):
            predicted = float(np.linalg.norm(est[:2] - pts[i, :2]))
            residuals.append(abs(predicted - dist[i]))
        mean_residual = float(np.mean(residuals)) if residuals else 999.0
        residual_score = max(0.0, 1.0 - mean_residual / 2.0)

        spread = float(np.linalg.norm(pts[:, :2].std(axis=0)))
        spread_score = min(1.0, spread / 1.5)
        count_score = min(1.0, (n - 2) / 5.0)

        quality = 0.2 * count_score + 0.35 * spread_score + 0.45 * residual_score
        quality = float(np.clip(quality, 0.0, 1.0))
        return est, quality, n


class RFIDConfig(ModuleConfig):
    """Configuration for :class:`RFIDModule`.

    `ModuleConfig` is a pydantic model, so these are two plain pydantic fields
    added on top of the framework's config fields.
    """

    url: str = Field(default=DEFAULT_URL, description="RFID active-tags endpoint.")
    interval: float = Field(default=0.5, gt=0, description="Poll interval in seconds.")
    rerun: bool = Field(
        default=False,
        description="Also display the live tag list in the DimOS (Rerun) viewer.",
    )
    rerun_spawn: bool = Field(
        default=True,
        description="Spawn our own viewer (standalone --ui). Set False to log into "
        "an existing DimOS viewer opened by a RerunBridgeModule (e.g. --go2).",
    )
    rerun_entity: str = Field(
        default="rfid",
        description="Rerun entity path for the RFID text panel.",
    )

    # --- Spatial markers (need the robot's pose; enabled by --go2) ---
    spatial: bool = Field(
        default=False,
        description="Place a 3D marker per tag in the world/camera views, refined "
        "over time from RSSI observed at different dog positions. Needs robot pose.",
    )
    world_frame: str = Field(default="world", description="World/map TF frame.")
    base_frame: str = Field(default="base_link", description="Robot body TF frame.")
    camera_frame: str = Field(default="camera_optical", description="Camera optical TF frame.")

    # Go2 color-camera intrinsics (GO2Connection._camera_info_static defaults).
    fx: float = Field(default=819.553492)
    fy: float = Field(default=820.646595)
    cx: float = Field(default=625.284099)
    cy: float = Field(default=336.808987)
    img_width: int = Field(default=1280)
    img_height: int = Field(default=720)

    # RSSI localization tuning.
    rssi_ref_dbm: float = Field(default=-50.0, description="Expected RSSI (dBm) at 1 m.")
    path_loss_n: float = Field(default=2.0, description="Path-loss exponent (2 free space).")
    min_baseline_m: float = Field(
        default=0.3, gt=0, description="Min dog travel before recording a new anchor."
    )
    stationary_speed_mps: float = Field(
        default=0.05,
        ge=0,
        description="Max dog speed (m/s) to count as stationary for RSSI sampling.",
    )
    min_rssi_samples: int = Field(
        default=3,
        ge=1,
        description="Min RSSI samples per anchor before closing it (outliers filtered).",
    )
    quality_blue: float = Field(
        default=QUALITY_BLUE, ge=0, le=1, description="Quality threshold for blue markers."
    )
    quality_green: float = Field(
        default=QUALITY_GREEN, ge=0, le=1, description="Quality threshold for green markers."
    )

    # --- Tag focus / clutter control ---
    focus_epcs: list[str] = Field(
        default_factory=list,
        description="EPC full strings or suffixes to focus on. Empty = show all tags. "
        "Matched case-insensitively as substring (so '8f' matches '...8f').",
    )
    focus_file: str = Field(
        default="",
        description="Optional path to a text file listing EPCs/suffixes (one per line, "
        "# comments OK). Reloaded live when the file changes. Empty string disables.",
    )
    focus_only_localize: bool = Field(
        default=True,
        description="When a focus filter is active, only accumulate localization for "
        "focused tags (ignores clutter). Display always respects focus.",
    )


# Default live-editable focus list beside this module (create on first run if needed).
DEFAULT_FOCUS_FILE = str(Path(__file__).resolve().parent / "rfid_focus.txt")

# Rerun entity path for the standalone RFID text panel.
RERUN_ENTITY = "rfid"

# Where spatial markers live in the viewer's entity tree.
MARKERS_3D_ENTITY = "world/rfid/markers"  # 3D world view
# Camera overlays go *under* the color-image pinhole entity so they land in pixel
# space and only render when the point projects inside the frame.
CAMERA_IMAGE_ENTITY = "world/color_image"

# Go2 Rerun panel origin (must match ``_go2_rfid_rerun_blueprint``).
GO2_RFID_RERUN_ENTITY = "world/rfid/tags"


def _go2_rfid_rerun_blueprint() -> Any:
    """Go2 layout: Camera | 3D map | RFID tag list."""
    import rerun as rr
    import rerun.blueprint as rrb

    if hasattr(rrb, "TextDocumentView"):
        rfid_view = rrb.TextDocumentView(origin=GO2_RFID_RERUN_ENTITY, name="RFID")
    else:
        rfid_view = rrb.TextLogView(origin=GO2_RFID_RERUN_ENTITY, name="RFID")

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                },
            ),
            rfid_view,
            column_shares=[2, 3, 1],
        ),
        rrb.TimePanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
    )


def _rfid_visual_override(_msg: Any) -> Any:
    """No-op visual override for the RFID text panel (must be picklable for workers)."""
    return None


def _go2_rfid_rerun_config() -> dict[str, Any]:
    """Merge Go2 Rerun settings with the RFID panel layout."""
    from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config

    cfg = {**rerun_config}
    cfg["blueprint"] = _go2_rfid_rerun_blueprint

    visual_override = dict(cfg.get("visual_override", {}))
    visual_override[GO2_RFID_RERUN_ENTITY] = _rfid_visual_override
    cfg["visual_override"] = visual_override

    max_hz = dict(cfg.get("max_hz", {}))
    max_hz[GO2_RFID_RERUN_ENTITY] = 1.0
    cfg["max_hz"] = max_hz

    if "pubsubs" not in cfg:
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM

        cfg["pubsubs"] = [LCM()]

    return cfg


class RFIDModule(Module):
    """Polls the RFID API in an async background loop and prints results."""

    config: RFIDConfig

    # `spawn()` schedules onto the module loop via run_coroutine_threadsafe,
    # which returns a concurrent.futures.Future (not an asyncio.Task).
    _poll_task: Future[Any] | None = None
    _stop_flag: threading.Event | None = None
    _rerun_connected: bool = False
    # EPC -> localizer accumulating that tag's sightings.
    _locs: dict[str, _TagLocalizer] | None = None
    _motion: _DogMotionTracker | None = None
    _focus: _FocusFilter | None = None
    # EPCs ever seen (for discovery list in the panel when filtering).
    _seen_epcs: dict[str, float] | None = None  # epc -> last rssi_dbm
    # Markers we've drawn so we can Clear() them when focus hides them.
    _drawn_markers: set[str] | None = None

    def __init__(self, **kwargs: Any) -> None:
        # Support `RFIDModule.blueprint(config=RFIDConfig())` and direct
        # construction with a config object. DimOS builds config from field
        # kwargs, so expand the object into its fields (existing kwargs win).
        cfg = kwargs.pop("config", None)
        if cfg is not None:
            for field_name in type(cfg).model_fields:
                kwargs.setdefault(field_name, getattr(cfg, field_name))
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        self.setup()

    def setup(self) -> None:
        """Kick off the async polling loop on the module event loop."""
        logger.info("RFIDModule polling %s every %.2fs", self.config.url, self.config.interval)
        if self.config.rerun and self.config.rerun_spawn:
            # Standalone: this module opens and owns the DimOS viewer.
            self._start_owned_viewer()
        self._locs = {}
        self._seen_epcs = {}
        self._drawn_markers = set()
        focus_file = self.config.focus_file or DEFAULT_FOCUS_FILE
        self._ensure_focus_file(focus_file)
        self._focus = _FocusFilter(
            config_patterns=list(self.config.focus_epcs),
            focus_file=focus_file,
        )
        self._motion = _DogMotionTracker(stationary_speed_mps=self.config.stationary_speed_mps)
        if self.config.spatial:
            # Touch the TF listener early so /tf is buffering before we look up poses.
            _ = self.tf
        self._stop_flag = threading.Event()
        self._poll_task = self.spawn(self._poll_loop())
        if self._focus is not None and self._focus.patterns():
            logger.info("RFID focus filter active: %s", self._focus.patterns())
        else:
            logger.info(
                "RFID focus: showing all tags. Edit %s (one EPC/suffix per line) to focus.",
                focus_file,
            )

    @staticmethod
    def _ensure_focus_file(path: str) -> None:
        """Create an empty focus file with usage comments if it doesn't exist."""
        p = Path(path)
        if p.exists():
            return
        try:
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(
                "# RFID focus list — one EPC (or suffix) per line.\n"
                "# Empty file = show ALL tags. Edit while running; changes apply next poll.\n"
                "# Examples:\n"
                "#   8f\n"
                "#   E280116060000203B5A908F\n"
                "\n",
                encoding="utf-8",
            )
            logger.info("Created focus file at %s (empty = show all)", path)
        except OSError as exc:
            logger.warning("Could not create focus file %s: %s", path, exc)

    @rpc
    def set_focus(self, epcs: list[str] | None = None) -> dict[str, Any]:
        """Focus the UI on specific EPCs/suffixes.

        - ``set_focus(["8f", "ab"])`` — show only matching tags
        - ``set_focus([])`` — show all (ignore focus file until changed)
        - ``set_focus(None)`` — drop RPC override; use focus file + config again
        """
        assert self._focus is not None
        if epcs is None:
            self._focus.set_rpc_focus(None)
            logger.info("RFID focus RPC override cleared — using file/config")
        else:
            self._focus.set_rpc_focus(list(epcs))
            logger.info("RFID focus set: %s", self._focus.patterns() or "(all)")
        pats = self._focus.patterns()
        return {"focus": pats, "active": bool(pats), "focus_file": self._focus.focus_file}

    @rpc
    def get_focus(self) -> dict[str, Any]:
        """Return current focus patterns and discovered EPCs."""
        assert self._focus is not None
        pats = self._focus.patterns()
        seen = list((self._seen_epcs or {}).keys())
        return {
            "focus": pats,
            "active": bool(pats),
            "discovered": seen,
            "focus_file": self._focus.focus_file,
        }

    def _start_owned_viewer(self) -> None:
        """Open the DimOS (Rerun) viewer and lay out a single RFID text panel."""
        try:
            import rerun as rr
            import rerun.blueprint as rrb
            from dimos.visualization.rerun.bridge import RERUN_GRPC_PORT

            rr.init("dimos")
            try:
                # Prefer the DimOS-branded viewer (same one `dimos run` opens).
                import rerun_bindings

                rerun_bindings.spawn(
                    port=RERUN_GRPC_PORT,
                    executable_name="dimos-viewer",
                    memory_limit="25%",
                )
                rr.connect_grpc(f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy")
            except Exception:
                # Fall back to the stock Rerun viewer if dimos-viewer is absent.
                rr.spawn(connect=True)

            view = (
                rrb.TextDocumentView(origin=self.config.rerun_entity, name="RFID tags")
                if hasattr(rrb, "TextDocumentView")
                else rrb.TextLogView(origin=self.config.rerun_entity, name="RFID tags")
            )
            rr.send_blueprint(rrb.Blueprint(view, rrb.TimePanel(state="collapsed")))
            self._rerun_connected = True
            logger.info("RFIDModule: DimOS viewer ready — tags will appear in the 'RFID tags' panel")
        except Exception as exc:  # noqa: BLE001 - UI is best-effort; keep polling either way
            logger.warning("RFIDModule: could not open the Rerun viewer: %s", exc)
            self._rerun_connected = False

    async def _poll_loop(self) -> None:
        """Continuously fetch active tags without blocking the event loop.

        Good place for a breakpoint: step through a request/response cycle and
        inspect `payload`, `tags`, `epc`, `rssi`.
        """
        stop = self._stop_flag
        assert stop is not None
        async with httpx.AsyncClient(timeout=1.5) as client:
            while not stop.is_set():
                try:
                    response = await client.get(self.config.url)
                    response.raise_for_status()
                    payload = response.json()
                    self._remember_seen(payload)
                    self._print_tags(payload)
                    if self.config.rerun:
                        self._log_rerun(payload)
                        if self.config.spatial and self._rerun_connected:
                            self._log_spatial(payload)
                except httpx.HTTPError as exc:
                    logger.warning("RFID poll failed (%s): %s", self.config.url, exc)
                except Exception as exc:  # noqa: BLE001 - keep the loop alive
                    logger.warning("RFID poll error: %s", exc)
                await asyncio.sleep(self.config.interval)

    def _remember_seen(self, payload: dict) -> None:
        assert self._seen_epcs is not None
        for tag in payload.get("tags", []) or []:
            epc = tag.get("epc")
            if not epc:
                continue
            rssi = tag.get("rssi_dbm")
            self._seen_epcs[epc] = float(rssi) if rssi is not None else float("nan")

    def _is_focused(self, epc: str) -> bool:
        assert self._focus is not None
        return self._focus.matches(epc)

    def _print_tags(self, payload: dict) -> None:
        count = payload.get("count", 0)
        tags = payload.get("tags", []) or []
        focus = self._focus
        active = bool(focus and focus.active)
        if not tags:
            print(f"[RFID] {count} tag(s) in range")
            return
        shown = [t for t in tags if self._is_focused(t.get("epc", ""))] if active else tags
        hidden_n = len(tags) - len(shown)
        hdr = f"[RFID] {count} tag(s) in range"
        if active:
            hdr += f" — focus {focus.patterns()} (showing {len(shown)}, hiding {hidden_n})"
        print(f"{hdr}:")
        for tag in shown:
            epc = tag.get("epc", "?")
            rssi = tag.get("rssi_dbm")
            rssi_s = f"{rssi} dBm" if rssi is not None else "unknown RSSI"
            print(f"    EPC={epc}  RSSI={rssi_s}")
        if active and hidden_n:
            print(f"    … {hidden_n} other tag(s) hidden (edit focus file to show)")

    def _tags_markdown(self, payload: dict) -> str:
        tags = payload.get("tags", []) or []
        focus = self._focus
        assert focus is not None
        assert self._seen_epcs is not None
        pats = focus.patterns()
        active = bool(pats)

        in_range = {t.get("epc"): t.get("rssi_dbm") for t in tags if t.get("epc")}
        # Prefer live RSSI; fall back to last-seen.
        for epc, rssi in in_range.items():
            if rssi is not None:
                self._seen_epcs[epc] = float(rssi)

        focused = sorted(epc for epc in self._seen_epcs if focus.matches(epc))
        others = sorted(epc for epc in self._seen_epcs if not focus.matches(epc)) if active else []

        lines: list[str] = []
        if active:
            lines.append(f"# RFID focus — {len(focused)} selected / {len(self._seen_epcs)} discovered")
            lines.append("")
            lines.append(f"_Filter: `{', '.join(pats)}`_")
            lines.append(f"_Edit `{focus.focus_file}` (one EPC/suffix per line) to change._")
            lines.append("")
            lines.append("## Focused")
            if not focused:
                lines.append("_No matching tags yet — walk near the tag or check the suffix._")
            else:
                lines.append("| EPC | RSSI |")
                lines.append("|-----|------|")
                for epc in focused:
                    rssi = in_range.get(epc, self._seen_epcs.get(epc))
                    mark = "" if epc in in_range else " (stale)"
                    rssi_s = f"{rssi} dBm{mark}" if rssi is not None and rssi == rssi else "—"
                    lines.append(f"| `{epc}` | {rssi_s} |")
            lines.append("")
            lines.append(f"## Hidden ({len(others)})")
            if others:
                lines.append("| EPC | RSSI |")
                lines.append("|-----|------|")
                for epc in others[:40]:  # keep the panel readable
                    rssi = in_range.get(epc, self._seen_epcs.get(epc))
                    mark = "" if epc in in_range else " (stale)"
                    rssi_s = f"{rssi} dBm{mark}" if rssi is not None and rssi == rssi else "—"
                    lines.append(f"| `{epc}` | {rssi_s} |")
                if len(others) > 40:
                    lines.append(f"| … | +{len(others) - 40} more |")
            else:
                lines.append("_None._")
        else:
            n = len(tags)
            lines.append(f"# RFID — {n} tag(s) in range")
            lines.append("")
            lines.append(
                f"_Showing all tags. Add EPC suffixes to `{focus.focus_file}` to focus / hide clutter._"
            )
            lines.append("")
            if not tags:
                lines.append("_No tags in range._")
            else:
                lines.append("| EPC | RSSI |")
                lines.append("|-----|------|")
                for tag in tags:
                    epc = tag.get("epc", "?")
                    rssi = tag.get("rssi_dbm")
                    rssi_s = f"{rssi} dBm" if rssi is not None else "—"
                    lines.append(f"| `{epc}` | {rssi_s} |")
        return "\n".join(lines)

    def _log_rerun(self, payload: dict) -> None:
        """Push the current tag list to the viewer as a text panel.

        In `--go2` mode we don't own the viewer: connect (lazily, with retry) to
        the one the RerunBridgeModule already opened, then log to the entity the
        RFID panel reads.
        """
        try:
            import rerun as rr

            if not self._rerun_connected:
                from dimos.core.global_config import global_config
                from dimos.visualization.rerun.bridge import RERUN_GRPC_PORT

                rr.init("dimos")
                host = getattr(global_config, "listen_host", None) or "127.0.0.1"
                rr.connect_grpc(f"rerun+http://{host}:{RERUN_GRPC_PORT}/proxy")
                self._rerun_connected = True

            md = self._tags_markdown(payload)
            try:
                rr.log(self.config.rerun_entity, rr.TextDocument(md, media_type=rr.MediaType.MARKDOWN))
            except (AttributeError, TypeError):
                rr.log(self.config.rerun_entity, rr.TextLog(md))
        except Exception as exc:  # noqa: BLE001 - never let UI logging break polling
            # Bridge viewer may not be up on the first poll; retry next tick.
            self._rerun_connected = False
            logger.debug("RFIDModule: rerun log failed (will retry): %s", exc)

    def _log_spatial(self, payload: dict) -> None:
        """Refine each tag's world position from RSSI and draw it in 3D + camera.

        - Feeds every current sighting (dog pose + RSSI) into that EPC's localizer.
        - Draws a marker in the 3D world view for every tag we've ever seen.
        - Projects each marker into the live camera image and shows a 2D overlay
          only when it falls inside the frustum (i.e. the dog is looking at it).
        """
        try:
            import rerun as rr
        except Exception:  # noqa: BLE001
            return

        assert self._locs is not None
        assert self._motion is not None

        # Where is the dog right now? Without a pose we can't add sightings.
        dog = self._tf_matrix(self.config.world_frame, self.config.base_frame)
        if dog is not None:
            dog_pos = dog[:3, 3]
            now = time.time()
            stationary = self._motion.update(dog_pos, now)
            if not stationary:
                # Dog is moving — close any open anchors without adding samples.
                for loc in self._locs.values():
                    loc.finalize_current_anchor()
            else:
                for tag in payload.get("tags", []) or []:
                    epc = tag.get("epc")
                    rssi = tag.get("rssi_dbm")
                    if not epc or rssi is None:
                        continue
                    if (
                        self.config.focus_only_localize
                        and self._focus is not None
                        and self._focus.active
                        and not self._is_focused(epc)
                    ):
                        continue
                    loc = self._locs.get(epc)
                    if loc is None:
                        loc = _TagLocalizer(
                            rssi_ref_dbm=self.config.rssi_ref_dbm,
                            path_loss_n=self.config.path_loss_n,
                            min_baseline_m=self.config.min_baseline_m,
                            min_rssi_samples=self.config.min_rssi_samples,
                        )
                        self._locs[epc] = loc
                    loc.add_stationary_sample(dog_pos, float(rssi), now)

        # world -> camera_optical, for projecting markers into the image.
        cam_from_world = self._tf_matrix(self.config.camera_frame, self.config.world_frame)

        assert self._drawn_markers is not None
        visible_now: set[str] = set()
        for epc, loc in self._locs.items():
            if not loc.obs:
                continue
            if not self._is_focused(epc):
                # Focus filter active and this EPC is not selected → hide markers.
                self._clear_marker(rr, epc)
                continue
            est, quality, n_obs = loc.estimate()
            self._log_marker_3d(rr, epc, est, quality, n_obs)
            self._log_marker_camera(rr, epc, est, quality, cam_from_world)
            visible_now.add(epc)

        # Clear markers for EPCs that left focus or were removed.
        for epc in list(self._drawn_markers - visible_now):
            self._clear_marker(rr, epc)
        self._drawn_markers = visible_now

    def _clear_marker(self, rr: Any, epc: str) -> None:
        try:
            rr.log(f"{MARKERS_3D_ENTITY}/{epc}", rr.Clear(recursive=True))
            rr.log(f"{CAMERA_IMAGE_ENTITY}/rfid/{epc}", rr.Clear(recursive=True))
        except Exception:  # noqa: BLE001
            pass
        if self._drawn_markers is not None:
            self._drawn_markers.discard(epc)

    def _tf_matrix(self, parent: str, child: str) -> np.ndarray | None:
        """Latest 4x4 transform mapping points in `child` frame into `parent`."""
        try:
            tf = self.tf.get(parent, child)
        except Exception:  # noqa: BLE001 - TF may not be ready yet
            return None
        if tf is None:
            return None
        try:
            return tf.to_matrix()
        except Exception:  # noqa: BLE001
            return None

    def _quality_color(self, quality: float) -> list[int]:
        """Gray (estimating) -> blue (refining) -> green (located)."""
        if quality >= self.config.quality_green:
            return [40, 200, 90]
        if quality >= self.config.quality_blue:
            return [60, 130, 255]
        return [150, 150, 150]

    def _quality_state(self, quality: float) -> str:
        if quality >= self.config.quality_green:
            return "located"
        if quality >= self.config.quality_blue:
            return "refining"
        return "estimating"

    def _log_marker_3d(
        self, rr: Any, epc: str, est: np.ndarray, quality: float, n_obs: int
    ) -> None:
        state = self._quality_state(quality)
        # Uncertain tags get a bigger, fuzzier dot; confident ones a tight dot.
        radius = float(0.30 * (1.0 - quality) + 0.05)
        label = f"{epc[-8:]} ({state}, n={n_obs})"
        try:
            rr.log(
                f"{MARKERS_3D_ENTITY}/{epc}",
                rr.Points3D(
                    [est.tolist()],
                    radii=[radius],
                    colors=[self._quality_color(quality)],
                    labels=[label],
                    show_labels=True,
                ),
            )
        except Exception as exc:  # noqa: BLE001
            logger.debug("RFIDModule: 3D marker log failed: %s", exc)

    def _log_marker_camera(
        self,
        rr: Any,
        epc: str,
        est: np.ndarray,
        quality: float,
        cam_from_world: np.ndarray | None,
    ) -> None:
        entity = f"{CAMERA_IMAGE_ENTITY}/rfid/{epc}"
        uv = self._project_to_image(est, cam_from_world)
        try:
            if uv is None:
                # Not in view (dog is looking elsewhere / behind) -> hide it.
                rr.log(entity, rr.Clear(recursive=True))
                return
            rr.log(
                entity,
                rr.Points2D(
                    [list(uv)],
                    radii=[8.0],
                    colors=[self._quality_color(quality)],
                    labels=[epc[-8:]],
                    show_labels=True,
                ),
            )
        except Exception as exc:  # noqa: BLE001
            logger.debug("RFIDModule: camera overlay log failed: %s", exc)

    def _project_to_image(
        self, world_pt: np.ndarray, cam_from_world: np.ndarray | None
    ) -> tuple[float, float] | None:
        """Pinhole-project a world point to pixels, or None if outside the frame.

        Returns None when the point is behind the camera or projects outside the
        image bounds — exactly the "is the dog looking at it?" test.
        """
        if cam_from_world is None:
            return None
        p_cam = cam_from_world @ np.array([world_pt[0], world_pt[1], world_pt[2], 1.0])
        z = p_cam[2]
        if z <= 0.05:  # behind (or on top of) the camera
            return None
        u = self.config.fx * p_cam[0] / z + self.config.cx
        v = self.config.fy * p_cam[1] / z + self.config.cy
        if not (0.0 <= u <= self.config.img_width and 0.0 <= v <= self.config.img_height):
            return None
        return float(u), float(v)

    @rpc
    def stop(self) -> None:
        if self._stop_flag is not None:
            self._stop_flag.set()
        if self._locs is not None:
            for loc in self._locs.values():
                loc.finalize_current_anchor()
        task = self._poll_task
        if task is not None:
            # Wait for the loop to exit on its own so the future resolves
            # normally (no CancelledError surfaced at shutdown).
            try:
                task.result(timeout=self.config.interval + 2.0)
            except BaseException:
                pass
            self._poll_task = None
        super().stop()


def run_in_process(config: RFIDConfig) -> None:
    """Run the module in THIS process (no worker fork) for IDE debugging.

    The async loop runs in a background thread of this process, so debugger
    breakpoints and stepping work reliably.
    """
    module = RFIDModule(config=config)
    module.start()
    logger.info("RFIDModule running in-process (pid=%d). Press Ctrl-C to stop.", os.getpid())
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        module.stop()


def run_via_coordinator(config: RFIDConfig) -> None:
    """Run the blueprint via ModuleCoordinator (deploys into a worker process)."""
    blueprint = autoconnect(RFIDModule.blueprint(config=config))
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        coordinator.loop()
    except KeyboardInterrupt:
        coordinator.stop()


def run_with_ui(config: RFIDConfig) -> None:
    """Open the DimOS (Rerun) viewer and show the live tag list in a text panel.

    Runs in-process (no forkserver worker) so it works identically from a plain
    terminal and from the IDE debugger (F5). Under the debugger, a forked worker
    plus the native viewer is unreliable on Python 3.12, so we avoid it here.
    """
    config = config.model_copy(update={"rerun": True, "rerun_spawn": True})
    logger.info("Starting DimOS viewer + RFID module (Ctrl-C to stop)...")
    run_in_process(config)


def run_with_go2(config: RFIDConfig) -> None:
    """Run the full Go2 stack (camera + 3D lidar) with the RFID panel added.

    Reuses DimOS's Go2 blueprint and overrides its Rerun layout with the shipped
    camera | 3D | RFID layout. This module logs its tags to the RFID entity that
    the panel reads; the Go2 modules feed camera + lidar to the same viewer.
    """
    from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    # Log into the bridge's viewer (don't spawn our own) at the panel's entity.
    # Enable spatial markers: the robot pose is available in this mode.
    config = config.model_copy(
        update={
            "rerun": True,
            "rerun_spawn": False,
            "rerun_entity": GO2_RFID_RERUN_ENTITY,
            "spatial": True,
        }
    )

    logger.info("Starting Go2 + RFID with the DimOS viewer (Ctrl-C to stop)...")
    blueprint = autoconnect(
        unitree_go2,
        RFIDModule.blueprint(config=config),
        # Same class as the Go2's own bridge, so this overrides its layout
        # (later module wins) -> camera | 3D | RFID.
        RerunBridgeModule.blueprint(**_go2_rfid_rerun_config()),
    )
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        coordinator.loop()
    except KeyboardInterrupt:
        coordinator.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the RFID DimOS module natively.")
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--ui",
        action="store_true",
        help="Open the DimOS (Rerun) viewer with the RFID panel only (standalone).",
    )
    group.add_argument(
        "--go2",
        action="store_true",
        help="Run the full Go2 stack (camera + 3D lidar) with the RFID panel added.",
    )
    group.add_argument(
        "--coordinator",
        action="store_true",
        help="Run via ModuleCoordinator (forks a worker process; stepping in the "
        "child is unreliable on Python 3.12). Default runs in-process for debugging.",
    )
    args = parser.parse_args()

    if args.ui:
        run_with_ui(RFIDConfig())
    elif args.go2:
        run_with_go2(RFIDConfig())
    elif args.coordinator:
        run_via_coordinator(RFIDConfig())
    else:
        run_in_process(RFIDConfig())
