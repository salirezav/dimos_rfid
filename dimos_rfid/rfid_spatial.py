# Copyright 2026. RSSI multilateration + Rerun spatial markers for RFID tags.
#
# Ported from rfid_module/rfid_module.py — gray (estimating) → blue (refining) →
# green (located) markers on the SLAM map, with dot size scaling by confidence.

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

import numpy as np
from pydantic import BaseModel, Field

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

QUALITY_BLUE = 0.4
QUALITY_GREEN = 0.85

MARKERS_3D_ENTITY = "world/rfid/markers"
CAMERA_IMAGE_ENTITY = "world/color_image"


@dataclass
class _Obs:
    pos: np.ndarray
    rssi: float
    ts: float
    n_samples: int = 1


@dataclass
class _Anchor:
    pos: np.ndarray
    rssi_samples: list[float] = field(default_factory=list)
    last_ts: float = 0.0

    def add_sample(self, rssi: float, ts: float) -> None:
        self.rssi_samples.append(float(rssi))
        self.last_ts = ts


def _filter_rssi_outliers(samples: list[float]) -> list[float]:
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
    stationary_speed_mps: float = 0.05
    _prev_pos: np.ndarray | None = None
    _prev_ts: float | None = None
    is_stationary: bool = False

    def update(self, pos: np.ndarray, ts: float) -> bool:
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
    config_patterns: list[str] = field(default_factory=list)
    focus_file: str = ""
    _file_patterns: list[str] = field(default_factory=list)
    _file_mtime: float | None = None

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
        pats = self.patterns()
        if not pats:
            return True
        epc_l = epc.lower()
        return any(p.lower() in epc_l for p in pats)


@dataclass
class _TagLocalizer:
    rssi_ref_dbm: float = -50.0
    path_loss_n: float = 2.0
    min_baseline_m: float = 0.3
    min_rssi_samples: int = 3
    min_observations: int = 5
    max_observations: int = 10
    max_history_observations: int = 50
    robust_loss_scale_m: float = 1.0
    max_range_m: float = 15.0
    obs: list[_Obs] = field(default_factory=list)
    _current_anchor: _Anchor | None = None

    def add_stationary_sample(self, pos: np.ndarray, rssi: float, ts: float) -> None:
        pos = np.asarray(pos, dtype=float)
        anchor = self._current_anchor
        if anchor is None or np.linalg.norm(pos - anchor.pos) >= self.min_baseline_m:
            self.finalize_current_anchor()
            self._current_anchor = _Anchor(pos=pos.copy(), last_ts=ts)
            anchor = self._current_anchor
        anchor.add_sample(rssi, ts)

    def finalize_current_anchor(self) -> None:
        anchor = self._current_anchor
        self._current_anchor = None
        if anchor is None or not anchor.rssi_samples:
            return
        filtered = _filter_rssi_outliers(anchor.rssi_samples)
        if len(filtered) < self.min_rssi_samples and len(anchor.rssi_samples) < self.min_rssi_samples:
            return
        rssi = float(np.median(filtered))
        if self.obs and np.linalg.norm(anchor.pos - self.obs[-1].pos) < self.min_baseline_m:
            self.obs[-1] = _Obs(anchor.pos, rssi, anchor.last_ts, len(filtered))
        else:
            self.obs.append(_Obs(anchor.pos, rssi, anchor.last_ts, len(filtered)))
            if len(self.obs) > self.max_history_observations:
                del self.obs[: len(self.obs) - self.max_history_observations]

    def _rssi_to_distance(self, rssi: np.ndarray) -> np.ndarray:
        return 10.0 ** ((self.rssi_ref_dbm - rssi) / (10.0 * self.path_loss_n))

    def _select_diverse_observations(self) -> list[_Obs]:
        if len(self.obs) <= self.max_observations:
            return list(self.obs)
        chosen = [len(self.obs) - 1]
        remaining = set(range(len(self.obs) - 1))
        while remaining and len(chosen) < self.max_observations:
            next_index = max(
                remaining,
                key=lambda index: min(
                    float(np.linalg.norm(self.obs[index].pos[:2] - self.obs[j].pos[:2]))
                    for j in chosen
                ),
            )
            chosen.append(next_index)
            remaining.remove(next_index)
        return [self.obs[index] for index in sorted(chosen)]

    @staticmethod
    def _linear_seed(pts: np.ndarray, dist: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        ref = int(np.argmin(dist))
        others = [index for index in range(len(pts)) if index != ref]
        a_mat = 2.0 * (pts[others, :2] - pts[ref, :2])
        b_vec = (
            (pts[others, :2] ** 2).sum(axis=1)
            - (pts[ref, :2] ** 2).sum()
            - dist[others] ** 2
            + dist[ref] ** 2
        )
        try:
            solution, _residuals, rank, _singular = np.linalg.lstsq(a_mat, b_vec, rcond=None)
        except np.linalg.LinAlgError:
            return fallback.copy()
        if rank < 2 or not np.all(np.isfinite(solution)):
            return fallback.copy()
        return np.asarray(solution, dtype=float)

    def _robust_joint_solve(
        self, pts: np.ndarray, dist: np.ndarray, sample_weights: np.ndarray, seed: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        estimate = np.asarray(seed, dtype=float).copy()
        scale = max(float(self.robust_loss_scale_m), 0.05)
        for _ in range(30):
            delta = estimate[None, :] - pts[:, :2]
            predicted = np.maximum(np.linalg.norm(delta, axis=1), 1e-6)
            residual = predicted - dist
            robust_weights = 1.0 / (1.0 + (residual / scale) ** 2)
            weights = np.maximum(sample_weights * robust_weights, 1e-6)
            jacobian = delta / predicted[:, None]
            normal = jacobian.T @ (weights[:, None] * jacobian) + np.eye(2) * 1e-6
            gradient = jacobian.T @ (weights * residual)
            try:
                step = np.linalg.solve(normal, gradient)
            except np.linalg.LinAlgError:
                break
            if not np.all(np.isfinite(step)):
                break
            estimate -= step
            if float(np.linalg.norm(step)) < 1e-4:
                break
        residual = np.linalg.norm(estimate[None, :] - pts[:, :2], axis=1) - dist
        return estimate, residual

    def estimate(self) -> tuple[np.ndarray, float, int]:
        selected = self._select_diverse_observations()
        n = len(selected)
        pts = np.array([observation.pos for observation in selected])
        rssi = np.array([observation.rssi for observation in selected])

        weights = 10.0 ** (rssi / 10.0)
        centroid = (weights[:, None] * pts).sum(axis=0) / weights.sum()

        if n < self.min_observations:
            return centroid, min(0.35, 0.07 * n), n

        dist = self._rssi_to_distance(rssi)
        sample_weights = np.sqrt(
            np.asarray([max(1, observation.n_samples) for observation in selected], dtype=float)
        )
        sample_weights /= max(float(sample_weights.max()), 1.0)
        seed = self._linear_seed(pts, dist, centroid[:2])

        candidates = []
        for candidate_seed in (seed, centroid[:2]):
            candidate, residual = self._robust_joint_solve(
                pts, dist, sample_weights, candidate_seed
            )
            objective = float(
                np.sum(sample_weights * np.log1p((residual / self.robust_loss_scale_m) ** 2))
            )
            candidates.append((objective, candidate, residual))
        _objective, solution, residuals = min(candidates, key=lambda candidate: candidate[0])

        est = np.array([solution[0], solution[1], float(pts[:, 2].mean())])
        if np.linalg.norm(est[:2] - pts[:, :2].mean(axis=0)) > self.max_range_m:
            return centroid, 0.25, n

        median_residual = float(np.median(np.abs(residuals)))
        residual_score = max(0.0, 1.0 - median_residual / 2.0)
        spread = float(np.linalg.norm(pts[:, :2].std(axis=0)))
        spread_score = min(1.0, spread / 1.5)
        centered = pts[:, :2] - pts[:, :2].mean(axis=0)
        singular = np.linalg.svd(centered, compute_uv=False)
        geometry_score = (
            min(1.0, float(singular[1] / singular[0]) / 0.35)
            if len(singular) > 1 and singular[0] > 1e-6
            else 0.0
        )
        count_score = min(1.0, (n - self.min_observations + 1) / 4.0)
        quality = (
            0.15 * count_score
            + 0.25 * spread_score
            + 0.20 * geometry_score
            + 0.40 * residual_score
        )
        quality = float(np.clip(quality, 0.0, 1.0))
        return est, quality, n


class RfidSpatialConfig(BaseModel):
    world_frame: str = "world"
    base_frame: str = "base_link"
    camera_frame: str = "camera_optical"
    fx: float = 819.553492
    fy: float = 820.646595
    cx: float = 625.284099
    cy: float = 336.808987
    img_width: int = 1280
    img_height: int = 720
    rssi_ref_dbm: float = -50.0
    path_loss_n: float = 2.0
    min_baseline_m: float = 0.3
    stationary_speed_mps: float = 0.05
    min_rssi_samples: int = 3
    min_observations: int = 5
    max_observations: int = 10
    robust_loss_scale_m: float = 1.0
    quality_blue: float = Field(default=QUALITY_BLUE, ge=0, le=1)
    quality_green: float = Field(default=QUALITY_GREEN, ge=0, le=1)
    focus_epcs: list[str] = Field(default_factory=list)
    focus_file: str = ""
    focus_only_localize: bool = True


class _TfLookup(Protocol):
    def get(self, parent: str, child: str) -> Any: ...


class RfidSpatialTracker:
    """Accumulate RSSI at stationary poses and draw quality-colored 3D markers."""

    def __init__(self, tf: _TfLookup, config: RfidSpatialConfig) -> None:
        self._tf = tf
        self._config = config
        self._locs: dict[str, _TagLocalizer] = {}
        self._motion = _DogMotionTracker(stationary_speed_mps=config.stationary_speed_mps)
        self._focus = _FocusFilter(
            config_patterns=list(config.focus_epcs),
            focus_file=config.focus_file,
        )
        self._drawn_markers: set[str] = set()
        _ = self._tf

    def update(self, tags: list[Any]) -> None:
        """Ingest tag readings and log estimated positions to Rerun."""
        try:
            import rerun as rr
        except Exception:
            return

        dog = self._tf_matrix(self._config.world_frame, self._config.base_frame)
        if dog is not None:
            dog_pos = dog[:3, 3]
            now = time.time()
            stationary = self._motion.update(dog_pos, now)
            if not stationary:
                for loc in self._locs.values():
                    loc.finalize_current_anchor()
            else:
                for tag in tags:
                    epc = getattr(tag, "epc", None) or (tag.get("epc") if isinstance(tag, dict) else None)
                    rssi = getattr(tag, "rssi_dbm", None)
                    if isinstance(tag, dict):
                        rssi = tag.get("rssi_dbm")
                    in_range = getattr(tag, "in_range", True)
                    if isinstance(tag, dict):
                        in_range = tag.get("in_range", True)
                    if not epc or rssi is None or not in_range:
                        continue
                    if (
                        self._config.focus_only_localize
                        and self._focus.active
                        and not self._focus.matches(epc)
                    ):
                        continue
                    loc = self._locs.get(epc)
                    if loc is None:
                        loc = _TagLocalizer(
                            rssi_ref_dbm=self._config.rssi_ref_dbm,
                            path_loss_n=self._config.path_loss_n,
                            min_baseline_m=self._config.min_baseline_m,
                            min_rssi_samples=self._config.min_rssi_samples,
                            min_observations=self._config.min_observations,
                            max_observations=self._config.max_observations,
                            robust_loss_scale_m=self._config.robust_loss_scale_m,
                        )
                        self._locs[epc] = loc
                    loc.add_stationary_sample(dog_pos, float(rssi), now)

        cam_from_world = self._tf_matrix(self._config.camera_frame, self._config.world_frame)
        visible_now: set[str] = set()
        for epc, loc in self._locs.items():
            if not loc.obs:
                continue
            if not self._focus.matches(epc):
                self._clear_marker(rr, epc)
                continue
            est, quality, n_obs = loc.estimate()
            self._log_marker_3d(rr, epc, est, quality, n_obs)
            self._log_marker_camera(rr, epc, est, quality, cam_from_world)
            visible_now.add(epc)

        for epc in list(self._drawn_markers - visible_now):
            self._clear_marker(rr, epc)
        self._drawn_markers = visible_now

    def finalize(self) -> None:
        for loc in self._locs.values():
            loc.finalize_current_anchor()

    def _tf_matrix(self, parent: str, child: str) -> np.ndarray | None:
        try:
            tf = self._tf.get(parent, child)
        except Exception:
            return None
        if tf is None:
            return None
        try:
            return tf.to_matrix()
        except Exception:
            return None

    def _quality_color(self, quality: float) -> list[int]:
        if quality >= self._config.quality_green:
            return [40, 200, 90]
        if quality >= self._config.quality_blue:
            return [60, 130, 255]
        return [150, 150, 150]

    def _quality_state(self, quality: float) -> str:
        if quality >= self._config.quality_green:
            return "located"
        if quality >= self._config.quality_blue:
            return "refining"
        return "estimating"

    def _log_marker_3d(
        self, rr: Any, epc: str, est: np.ndarray, quality: float, n_obs: int
    ) -> None:
        state = self._quality_state(quality)
        radius = float(0.30 * (1.0 - quality) + 0.05)
        label = f"{epc[-8:]} ({state}, {int(round(quality * 100))}%, n={n_obs})"
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
        except Exception as exc:
            logger.debug("RFID spatial: 3D marker log failed: %s", exc)

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
        except Exception as exc:
            logger.debug("RFID spatial: camera overlay log failed: %s", exc)

    def _project_to_image(
        self, world_pt: np.ndarray, cam_from_world: np.ndarray | None
    ) -> tuple[float, float] | None:
        if cam_from_world is None:
            return None
        p_cam = cam_from_world @ np.array([world_pt[0], world_pt[1], world_pt[2], 1.0])
        z = p_cam[2]
        if z <= 0.05:
            return None
        u = self._config.fx * p_cam[0] / z + self._config.cx
        v = self._config.fy * p_cam[1] / z + self._config.cy
        if not (0.0 <= u <= self._config.img_width and 0.0 <= v <= self._config.img_height):
            return None
        return float(u), float(v)

    def _clear_marker(self, rr: Any, epc: str) -> None:
        try:
            rr.log(f"{MARKERS_3D_ENTITY}/{epc}", rr.Clear(recursive=True))
            rr.log(f"{CAMERA_IMAGE_ENTITY}/rfid/{epc}", rr.Clear(recursive=True))
        except Exception:
            pass
        self._drawn_markers.discard(epc)
