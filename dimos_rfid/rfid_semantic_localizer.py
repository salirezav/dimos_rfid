"""DimOS module: semantic particle-filter RFID tag localization.

Subscribes to ``rfid_tags``, reads robot pose from TF, and updates an
:class:`~dimos_rfid.rfid_tracker.RFIDTracker` against a semantic occupancy map.

Tag-of-interest selection uses the same ``rfid_focus.txt`` pattern as the
experimental RFID module: put an EPC (or suffix) in the file to localize only
that tag.

Estimated tag poses are drawn in Rerun as 3D points (world / LiDAR view) and
as 2D overlays on the dog camera when the tag projects into the image.
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

# Rerun entity paths (must sit under the Go2 Spatial2D / Spatial3D view origins).
MARKERS_3D_ENTITY = "world/rfid/markers"
CAMERA_IMAGE_ENTITY = "world/color_image"

QUALITY_BLUE = 0.45
QUALITY_GREEN = 0.75


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
    camera_frame: str = Field(
        default="camera_optical",
        description="Camera optical TF frame for 2D image overlays.",
    )
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
        description="How often to log / refresh Rerun markers (0 = every tag batch).",
    )
    visualize: bool = Field(
        default=True,
        description="Draw estimated tag poses in Rerun 3D + camera views.",
    )
    # Go2 color-camera intrinsics (GO2Connection._camera_info_static defaults).
    fx: float = Field(default=819.553492)
    fy: float = Field(default=820.646595)
    cx: float = Field(default=625.284099)
    cy: float = Field(default=336.808987)
    img_width: int = Field(default=1280)
    img_height: int = Field(default=720)
    quality_blue: float = Field(default=QUALITY_BLUE, ge=0, le=1)
    quality_green: float = Field(default=QUALITY_GREEN, ge=0, le=1)
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
      - Rerun 3D markers + camera overlays (when ``visualize=True``)
      - agent skills: ``get_estimated_target_location``, ``get_location_confidence``
    """

    config: RfidSemanticLocalizerConfig
    rfid_tags: In[RfidTagArray]

    _tracker: RFIDTracker | None = None
    _semantic_map: SemanticOccupancyGrid3D | None = None
    _focus: FocusFilter | None = None
    _last_log_ts: float = 0.0
    _rerun_connected: bool = False
    _drawn_markers: set[str] | None = None

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
        self._drawn_markers = set()
        self.rfid_tags.subscribe(self._on_tags)
        logger.info(
            "RfidSemanticLocalizerModule started "
            "(particles=%d, bounds=%s, map=%s, visualize=%s)",
            self.config.n_particles,
            bounds,
            "npz" if self.config.map_npz_path else "empty-free-space",
            self.config.visualize,
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

    def _tf_matrix(self, parent: str, child: str) -> np.ndarray | None:
        try:
            tf = self.tf.get(parent, child)
        except Exception:  # noqa: BLE001
            return None
        if tf is None:
            return None
        try:
            return tf.to_matrix()
        except Exception:  # noqa: BLE001
            return None

    def _dog_pose(self) -> tuple[np.ndarray, float, float] | None:
        """Return (xyz, yaw, pitch) of the antenna in the world frame."""
        for child in (self.config.antenna_frame, self.config.base_frame):
            mat = self._tf_matrix(self.config.world_frame, child)
            if mat is None:
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
        should_emit = hz <= 0 or (now - self._last_log_ts) >= (1.0 / max(hz, 1e-6))
        if should_emit:
            self._last_log_ts = now
            if hz > 0:
                self._log_estimates()
            if self.config.visualize:
                self._visualize_estimates()

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

    # ------------------------------------------------------------------
    # Rerun visualization (3D world + camera image)
    # ------------------------------------------------------------------

    def _ensure_rerun(self) -> Any | None:
        """Connect to the DimOS Rerun gRPC bridge; return ``rerun`` or None."""
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
            logger.debug("Rerun connect failed (will retry): %s", exc)
            return None

    def _quality_color(self, quality: float) -> list[int]:
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

    def _visualize_estimates(self) -> None:
        assert self._tracker is not None
        rr = self._ensure_rerun()
        if rr is None or self._drawn_markers is None:
            return

        cam_from_world = self._tf_matrix(self.config.camera_frame, self.config.world_frame)
        visible_now: set[str] = set()

        for tag_id in self._tracker.known_tags():
            if self._focus is not None and not self._focus.matches(tag_id):
                self._clear_marker(rr, tag_id)
                continue
            loc = self._tracker.get_estimated_target_location(tag_id)
            conf = self._tracker.get_location_confidence(tag_id)
            if loc is None or not np.all(np.isfinite(loc)):
                continue
            self._log_marker_3d(rr, tag_id, loc, conf)
            self._log_marker_camera(rr, tag_id, loc, conf, cam_from_world)
            visible_now.add(tag_id)

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

    def _log_marker_3d(self, rr: Any, epc: str, est: np.ndarray, quality: float) -> None:
        state = self._quality_state(quality)
        radius = float(0.30 * (1.0 - quality) + 0.05)
        label = f"{epc[-8:]} ({state}, conf={quality:.2f})"
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
            logger.debug("3D marker log failed: %s", exc)

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
            logger.debug("Camera overlay log failed: %s", exc)

    def _project_to_image(
        self, world_pt: np.ndarray, cam_from_world: np.ndarray | None
    ) -> tuple[float, float] | None:
        if cam_from_world is None:
            return None
        p_cam = cam_from_world @ np.array(
            [world_pt[0], world_pt[1], world_pt[2], 1.0], dtype=np.float64
        )
        z = float(p_cam[2])
        if z <= 0.05:
            return None
        u = self.config.fx * float(p_cam[0]) / z + self.config.cx
        v = self.config.fy * float(p_cam[1]) / z + self.config.cy
        if not (0.0 <= u <= self.config.img_width and 0.0 <= v <= self.config.img_height):
            return None
        return float(u), float(v)

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
        if self._drawn_markers:
            rr = self._ensure_rerun()
            if rr is not None:
                for epc in list(self._drawn_markers):
                    self._clear_marker(rr, epc)
        self._tracker = None
        self._semantic_map = None
        self._focus = None
        self._drawn_markers = None
        super().stop()
