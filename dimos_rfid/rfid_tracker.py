"""Orchestrator that fuses robot telemetry into per-tag semantic particle filters."""

from __future__ import annotations

from typing import Any

import numpy as np

from dimos_rfid.semantic_map import SemanticMapProtocol, SemanticOccupancyGrid3D
from dimos_rfid.semantic_particle_filter import (
    DEFAULT_N_PARTICLES,
    SemanticParticleFilter3D,
    evaluate_multipath_reading,
)


class RFIDTracker:
    """Ingests real-time dog pose + RSSI and maintains a PF per tag id.

    Agent-facing query methods:
      - ``get_estimated_target_location(tag_id)``
      - ``get_location_confidence(tag_id)``
    """

    def __init__(
        self,
        bounds: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
        *,
        n_particles: int = DEFAULT_N_PARTICLES,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.bounds = bounds
        self.n_particles = int(n_particles)
        self.rng = rng if rng is not None else np.random.default_rng()
        self._filters: dict[str, SemanticParticleFilter3D] = {}
        self._last_ts: dict[str, float] = {}
        self._update_count: dict[str, int] = {}

    def set_bounds(
        self,
        bounds: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
        *,
        reinitialize: bool = True,
    ) -> None:
        """Update spatial bounds; optionally re-scatter existing filters."""
        self.bounds = bounds
        if reinitialize:
            for tag_id, pf in self._filters.items():
                pf.initialize(bounds)

    def reset(self, tag_id: str | None = None) -> None:
        """Reset one tag filter, or all filters if ``tag_id`` is None."""
        if tag_id is None:
            self._filters.clear()
            self._last_ts.clear()
            self._update_count.clear()
            return
        key = str(tag_id)
        self._filters.pop(key, None)
        self._last_ts.pop(key, None)
        self._update_count.pop(key, None)

    def _get_or_create(self, tag_id: str) -> SemanticParticleFilter3D:
        key = str(tag_id)
        pf = self._filters.get(key)
        if pf is None:
            pf = SemanticParticleFilter3D(
                n_particles=self.n_particles,
                bounds=self.bounds,
                rng=self.rng,
            )
            self._filters[key] = pf
            self._update_count[key] = 0
        return pf

    def ingest(
        self,
        dog_x: float,
        dog_y: float,
        dog_z: float,
        dog_yaw: float,
        dog_pitch: float,
        tag_id: str,
        rssi: float,
        semantic_lidar_map: SemanticMapProtocol | SemanticOccupancyGrid3D,
        *,
        timestamp: float | None = None,
        dt: float | None = None,
    ) -> dict[str, Any]:
        """Ingest one unidirectional RFID reading and update the tag's filter.

        Parameters
        ----------
        dog_x, dog_y, dog_z:
            Antenna / robot position in the world frame (meters).
        dog_yaw, dog_pitch:
            Antenna orientation in radians (Z-up yaw, pitch about Y).
        tag_id:
            EPC / tag identifier.
        rssi:
            Received signal strength in dBm.
        semantic_lidar_map:
            Semantic occupancy map implementing ``SemanticMapProtocol``.
        timestamp:
            Optional monotonic time (seconds); used to derive ``dt`` if given.
        dt:
            Explicit predict timestep; overrides timestamp delta when set.

        Returns
        -------
        dict with keys ``multipath``, ``location``, ``confidence``, ``updates``.
        """
        key = str(tag_id)
        dog_pos = np.array([dog_x, dog_y, dog_z], dtype=np.float64)
        pf = self._get_or_create(key)

        # Predict step.
        if dt is None:
            if timestamp is not None and key in self._last_ts:
                dt = max(float(timestamp) - self._last_ts[key], 1e-3)
            else:
                dt = 1.0
        if timestamp is not None:
            self._last_ts[key] = float(timestamp)

        pf.predict(dt=float(dt))

        is_multipath, weight_scale = evaluate_multipath_reading(
            semantic_lidar_map,
            dog_pos,
            float(dog_yaw),
            float(dog_pitch),
            float(rssi),
        )

        pf.update(
            dog_pos,
            float(dog_yaw),
            float(dog_pitch),
            float(rssi),
            semantic_lidar_map,
            multipath_discount=weight_scale,
        )

        self._update_count[key] = self._update_count.get(key, 0) + 1
        location, confidence = pf.estimate()
        return {
            "multipath": bool(is_multipath),
            "location": location,
            "confidence": float(confidence),
            "updates": self._update_count[key],
        }

    def get_estimated_target_location(self, tag_id: str) -> np.ndarray | None:
        """Weighted mean ``[x, y, z]`` of the particle cluster, or None if unknown."""
        pf = self._filters.get(str(tag_id))
        if pf is None:
            return None
        mean, _ = pf.estimate()
        if not np.all(np.isfinite(mean)):
            return None
        return mean

    def get_location_confidence(self, tag_id: str) -> float:
        """Confidence in ``[0, 1]`` from cluster variance; ``0.0`` if unknown."""
        pf = self._filters.get(str(tag_id))
        if pf is None:
            return 0.0
        _, confidence = pf.estimate()
        return float(confidence)

    def get_particles(self, tag_id: str) -> tuple[np.ndarray, np.ndarray] | None:
        """Return ``(particles, weights)`` copies, or None if tag unseen."""
        pf = self._filters.get(str(tag_id))
        if pf is None:
            return None
        return pf.get_particles()

    def known_tags(self) -> list[str]:
        return list(self._filters.keys())
