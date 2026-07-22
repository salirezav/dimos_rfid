"""DimOS module: semantic particle-filter RFID tag localization.

Subscribes to ``rfid_tags``, reads robot pose from TF, and updates an
:class:`~dimos_rfid.rfid_tracker.RFIDTracker` against a semantic occupancy map.

Tag-of-interest selection uses the same ``rfid_focus.txt`` pattern as the
experimental RFID module: put an EPC (or suffix) in the file to localize only
that tag.
"""

from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Any

import numpy as np
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.utils.logging_config import setup_logger

from dimos_rfid.focus_filter import FocusFilter, ensure_focus_file
from dimos_rfid.msgs import RfidTagArray
from dimos_rfid.rfid_tracker import RFIDTracker
from dimos_rfid.semantic_map import MaterialClass, SemanticOccupancyGrid3D

logger = setup_logger()

DEFAULT_FOCUS_FILE = str(Path(__file__).resolve().parent / "rfid_focus.txt")


def _yaw_pitch_from_matrix(rot: np.ndarray) -> tuple[float, float]:
    """Extract yaw (Z) and pitch (Y) from a 3x3 rotation matrix (Z-up)."""
    yaw = float(math.atan2(rot[1, 0], rot[0, 0]))
    pitch = float(math.atan2(-rot[2, 0], math.hypot(rot[0, 0], rot[1, 0])))
    return yaw, pitch


class RfidSemanticLocalizerConfig(ModuleConfig):
    """Configuration for :class:`RfidSemanticLocalizerModule`."""

    world_frame: str = Field(default="world", description="World / map TF frame.")
    antenna_frame: str = Field(
        default="rfid_antenna",
        description="RFID antenna TF frame (falls back to base_link if missing).",
    )
    base_frame: str = Field(default="base_link", description="Robot body TF frame.")
    n_particles: int = Field(default=5000, gt=100, description="Particles per tag.")
    xmin: float = Field(default=-5.0)
    xmax: float = Field(default=15.0)
    ymin: float = Field(default=-5.0)
    ymax: float = Field(default=15.0)
    zmin: float = Field(default=0.0)
    zmax: float = Field(default=3.0)
    map_resolution: float = Field(default=0.2, gt=0.01, description="Voxel size (m).")
    map_npz_path: str = Field(
        default="",
        description="Optional path to a .npz semantic map "
        "(keys: labels, origin, resolution). Empty → blank free-space map.",
    )
    log_estimates_hz: float = Field(
        default=0.5,
        ge=0.0,
        description="How often to log location estimates (0 disables).",
    )
    focus_file: str = Field(
        default="",
        description="Path to rfid_focus.txt (one EPC/suffix per line). "
        "Empty uses dimos_rfid/rfid_focus.txt beside this module.",
    )
    focus_epcs: list[str] = Field(
        default_factory=list,
        description="Extra EPC/suffix patterns to focus (merged with focus_file).",
    )


class RfidSemanticLocalizerModule(Module):
    """Fuse RFID RSSI + TF pose + semantic map via a 3D particle filter.

    **Inputs (automatic when running DimOS):**
      - ``rfid_tags`` stream (from ``RfidModule``)
      - TF ``world ← rfid_antenna`` (or ``base_link``)
      - semantic occupancy map (blank floor by default, or ``RFID_SEMANTIC_MAP``)
      - TOI filter via ``rfid_focus.txt``

    **Outputs:**
      - log lines: ``TOI … @ [x, y, z] m  conf=…``
      - agent skills: ``get_estimated_target_location``, ``get_location_confidence``
      - Python API on the attached ``RFIDTracker``
    """

    config: RfidSemanticLocalizerConfig
    rfid_tags: In[RfidTagArray]

    _tracker: RFIDTracker | None = None
    _semantic_map: SemanticOccupancyGrid3D | None = None
    _focus: FocusFilter | None = None
    _last_log_ts: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        try:
            self.tf.start()
        except Exception as exc:  # noqa: BLE001
            logger.warning("TF start failed (will retry on lookups): %s", exc)

        focus_path = (
            self.config.focus_file
            or os.environ.get("RFID_FOCUS_FILE", "")
            or DEFAULT_FOCUS_FILE
        )
        ensure_focus_file(focus_path)
        self._focus = FocusFilter(
            config_patterns=list(self.config.focus_epcs),
            focus_file=focus_path,
        )
        if self._focus.active:
            logger.info("RFID focus filter active: %s", self._focus.patterns())
        else:
            logger.info(
                "RFID focus: localizing ALL tags. Edit %s (one EPC/suffix per line) to focus.",
                focus_path,
            )

        self._semantic_map = self._build_map()
        bounds = (
            (self.config.xmin, self.config.xmax),
            (self.config.ymin, self.config.ymax),
            (self.config.zmin, self.config.zmax),
        )
        self._tracker = RFIDTracker(
            bounds=bounds,
            n_particles=self.config.n_particles,
        )
        self.rfid_tags.subscribe(self._on_tags)
        logger.info(
            "RfidSemanticLocalizerModule started "
            "(particles=%d, bounds=%s, map=%s)",
            self.config.n_particles,
            bounds,
            "npz" if self.config.map_npz_path else "empty-free-space",
        )

    def _build_map(self) -> SemanticOccupancyGrid3D:
        path = (self.config.map_npz_path or os.environ.get("RFID_SEMANTIC_MAP", "")).strip()
        if path:
            return self._load_map_npz(path)

        origin = np.array(
            [self.config.xmin, self.config.ymin, self.config.zmin],
            dtype=np.float64,
        )
        res = float(self.config.map_resolution)
        shape = (
            max(1, int(math.ceil((self.config.xmax - self.config.xmin) / res))),
            max(1, int(math.ceil((self.config.ymax - self.config.ymin) / res))),
            max(1, int(math.ceil((self.config.zmax - self.config.zmin) / res))),
        )
        grid = SemanticOccupancyGrid3D(origin=origin, resolution=res, shape=shape)
        grid.set_box(
            (self.config.xmin, self.config.ymin, self.config.zmin),
            (self.config.xmax, self.config.ymax, self.config.zmin + res),
            MaterialClass.STRUCTURAL,
        )
        return grid

    @staticmethod
    def _load_map_npz(path: str) -> SemanticOccupancyGrid3D:
        data = np.load(path, allow_pickle=False)
        labels = np.asarray(data["labels"], dtype=np.int8)
        origin = np.asarray(data["origin"], dtype=np.float64).reshape(3)
        resolution = float(np.asarray(data["resolution"]).reshape(()))
        grid = SemanticOccupancyGrid3D(origin=origin, resolution=resolution, shape=labels.shape)
        grid.labels[...] = labels
        logger.info("Loaded semantic map from %s shape=%s", path, labels.shape)
        return grid

    def _dog_pose(self) -> tuple[np.ndarray, float, float] | None:
        """Return (xyz, yaw, pitch) of the antenna in the world frame."""
        for child in (self.config.antenna_frame, self.config.base_frame):
            try:
                tf = self.tf.get(self.config.world_frame, child)
            except Exception:  # noqa: BLE001
                tf = None
            if tf is None:
                continue
            try:
                mat = tf.to_matrix()
            except Exception:  # noqa: BLE001
                continue
            pos = mat[:3, 3].astype(np.float64)
            yaw, pitch = _yaw_pitch_from_matrix(mat[:3, :3])
            return pos, yaw, pitch
        return None

    def _on_tags(self, msg: RfidTagArray) -> None:
        if self._tracker is None or self._semantic_map is None:
            return
        pose = self._dog_pose()
        if pose is None:
            logger.debug("No TF pose yet; skipping particle-filter update")
            return
        dog_pos, yaw, pitch = pose
        now = time.time()
        for tag in msg.active_tags():
            if tag.rssi_dbm is None or not tag.epc:
                continue
            if self._focus is not None and not self._focus.matches(tag.epc):
                continue
            self._tracker.ingest(
                float(dog_pos[0]),
                float(dog_pos[1]),
                float(dog_pos[2]),
                yaw,
                pitch,
                tag.epc,
                float(tag.rssi_dbm),
                self._semantic_map,
                timestamp=now,
            )

        hz = float(self.config.log_estimates_hz)
        if hz > 0 and (now - self._last_log_ts) >= (1.0 / hz):
            self._last_log_ts = now
            self._log_estimates()

    def _log_estimates(self) -> None:
        assert self._tracker is not None
        for tag_id in self._tracker.known_tags():
            if self._focus is not None and not self._focus.matches(tag_id):
                continue
            loc = self._tracker.get_estimated_target_location(tag_id)
            conf = self._tracker.get_location_confidence(tag_id)
            if loc is None:
                continue
            short = tag_id[-8:] if len(tag_id) > 8 else tag_id
            logger.info(
                "TOI %s @ [%.2f, %.2f, %.2f] m  conf=%.2f",
                short,
                loc[0],
                loc[1],
                loc[2],
                conf,
            )

    def get_tracker(self) -> RFIDTracker | None:
        return self._tracker

    def get_semantic_map(self) -> SemanticOccupancyGrid3D | None:
        return self._semantic_map

    def set_semantic_map(self, semantic_map: SemanticOccupancyGrid3D) -> None:
        """Replace the live semantic map (e.g. after LiDAR+vision fusion)."""
        self._semantic_map = semantic_map

    @rpc
    def set_focus(self, epcs: list[str] | None = None) -> dict[str, Any]:
        """Set focus patterns at runtime.

        - ``set_focus(["8f"])`` — localize only matching tags
        - ``set_focus([])`` — localize all (ignore focus file until changed)
        - ``set_focus(None)`` — drop RPC override; use focus file again
        """
        assert self._focus is not None
        self._focus.set_rpc_focus(epcs)
        pats = self._focus.patterns()
        return {"focus": pats, "active": bool(pats)}

    @skill
    def get_estimated_target_location(self, tag_id: str) -> str:
        """Return the estimated 3D world location of an RFID tag (particle-filter mean).

        Args:
            tag_id: Full or partial EPC hex string.
        """
        if self._tracker is None:
            return "Semantic RFID localizer not started."
        key = self._resolve_tag_id(tag_id)
        if key is None:
            return f"No location estimate yet for tag {tag_id!r}."
        loc = self._tracker.get_estimated_target_location(key)
        if loc is None:
            return f"No location estimate yet for tag {tag_id!r}."
        return f"{key}: [{loc[0]:.3f}, {loc[1]:.3f}, {loc[2]:.3f}] m"

    @skill
    def get_location_confidence(self, tag_id: str) -> str:
        """Return localization confidence in [0, 1] for an RFID tag.

        Args:
            tag_id: Full or partial EPC hex string.
        """
        if self._tracker is None:
            return "Semantic RFID localizer not started."
        key = self._resolve_tag_id(tag_id)
        if key is None:
            return f"No confidence estimate yet for tag {tag_id!r}."
        conf = self._tracker.get_location_confidence(key)
        return f"{key}: confidence={conf:.3f}"

    def _resolve_tag_id(self, tag_id: str) -> str | None:
        assert self._tracker is not None
        needle = tag_id.strip().lower()
        known = self._tracker.known_tags()
        if needle in known:
            return needle
        matches = [k for k in known if needle in k.lower()]
        if len(matches) == 1:
            return matches[0]
        if not matches:
            return None
        return matches[0]

    @rpc
    def stop(self) -> None:
        self._tracker = None
        self._semantic_map = None
        self._focus = None
        super().stop()
