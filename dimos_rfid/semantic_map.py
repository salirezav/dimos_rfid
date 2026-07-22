"""Semantic 3D occupancy grid for RFID localization.

Class A (STRUCTURAL) blocks RF origin hypotheses. Class B (INVENTORY) is
penetrable with attenuation. Free space is unrestricted.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Protocol, runtime_checkable

import numpy as np


class MaterialClass(IntEnum):
    """Semantic material labels for voxels."""

    FREE = 0
    STRUCTURAL = 1  # Class A: walls, floors, metal pillars
    INVENTORY = 2  # Class B: cardboard boxes, pallets, plastic totes


@dataclass(frozen=True)
class RaycastHit:
    """Result of a raycast through a semantic occupancy grid."""

    hit: bool
    distance_m: float
    material: MaterialClass
    inventory_penetration_m: float
    hit_point: np.ndarray | None = None


@runtime_checkable
class SemanticMapProtocol(Protocol):
    """Minimal map interface so alternate backends can replace the grid."""

    def class_at(self, xyz: np.ndarray) -> MaterialClass: ...

    def is_inside_bounds(self, xyz: np.ndarray) -> bool: ...

    def raycast(
        self,
        origin: np.ndarray,
        direction: np.ndarray,
        max_range: float,
        step: float | None = None,
    ) -> RaycastHit: ...


class SemanticOccupancyGrid3D:
    """Dense voxel grid with per-cell material labels.

    Parameters
    ----------
    origin:
        World-frame XYZ of the *minimum* corner of the grid (meters).
    resolution:
        Voxel edge length in meters.
    shape:
        Grid dimensions ``(nx, ny, nz)`` in voxels.
    """

    def __init__(
        self,
        origin: np.ndarray | tuple[float, float, float],
        resolution: float,
        shape: tuple[int, int, int],
    ) -> None:
        if resolution <= 0:
            raise ValueError(f"resolution must be > 0, got {resolution}")
        nx, ny, nz = shape
        if nx < 1 or ny < 1 or nz < 1:
            raise ValueError(f"shape must be positive, got {shape}")

        self.origin = np.asarray(origin, dtype=np.float64).reshape(3)
        self.resolution = float(resolution)
        self.shape = (int(nx), int(ny), int(nz))
        self.labels = np.zeros(self.shape, dtype=np.int8)

    @property
    def bounds_max(self) -> np.ndarray:
        extents = np.array(self.shape, dtype=np.float64) * self.resolution
        return self.origin + extents

    def world_to_index(self, xyz: np.ndarray) -> tuple[int, int, int] | None:
        """Map a world point to a voxel index, or None if out of bounds."""
        xyz = np.asarray(xyz, dtype=np.float64).reshape(3)
        idx = np.floor((xyz - self.origin) / self.resolution).astype(np.int64)
        if np.any(idx < 0) or np.any(idx >= np.array(self.shape)):
            return None
        return int(idx[0]), int(idx[1]), int(idx[2])

    def is_inside_bounds(self, xyz: np.ndarray) -> bool:
        return self.world_to_index(xyz) is not None

    def class_at(self, xyz: np.ndarray) -> MaterialClass:
        idx = self.world_to_index(xyz)
        if idx is None:
            return MaterialClass.FREE
        return MaterialClass(int(self.labels[idx]))

    def set_voxel(self, ijk: tuple[int, int, int], material: MaterialClass) -> None:
        i, j, k = ijk
        if not (0 <= i < self.shape[0] and 0 <= j < self.shape[1] and 0 <= k < self.shape[2]):
            raise IndexError(f"voxel index {ijk} out of range for shape {self.shape}")
        self.labels[i, j, k] = int(material)

    def set_box(
        self,
        min_xyz: np.ndarray | tuple[float, float, float],
        max_xyz: np.ndarray | tuple[float, float, float],
        material: MaterialClass,
    ) -> None:
        """Fill all voxels whose centers lie inside the axis-aligned box."""
        min_xyz = np.asarray(min_xyz, dtype=np.float64).reshape(3)
        max_xyz = np.asarray(max_xyz, dtype=np.float64).reshape(3)
        i0 = max(0, int(np.floor((min_xyz[0] - self.origin[0]) / self.resolution)))
        j0 = max(0, int(np.floor((min_xyz[1] - self.origin[1]) / self.resolution)))
        k0 = max(0, int(np.floor((min_xyz[2] - self.origin[2]) / self.resolution)))
        i1 = min(self.shape[0], int(np.floor((max_xyz[0] - self.origin[0]) / self.resolution)) + 1)
        j1 = min(self.shape[1], int(np.floor((max_xyz[1] - self.origin[1]) / self.resolution)) + 1)
        k1 = min(self.shape[2], int(np.floor((max_xyz[2] - self.origin[2]) / self.resolution)) + 1)

        for i in range(i0, i1):
            for j in range(j0, j1):
                for k in range(k0, k1):
                    center = self.origin + (np.array([i, j, k], dtype=np.float64) + 0.5) * self.resolution
                    if np.all(center >= min_xyz) and np.all(center <= max_xyz):
                        self.labels[i, j, k] = int(material)

    def fill_from_points(
        self,
        points: np.ndarray,
        material: MaterialClass,
    ) -> None:
        """Label voxels that contain any of the given world points."""
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"points must be (N, 3), got {points.shape}")
        for p in points:
            idx = self.world_to_index(p)
            if idx is not None:
                self.labels[idx] = int(material)

    def raycast(
        self,
        origin: np.ndarray,
        direction: np.ndarray,
        max_range: float,
        step: float | None = None,
    ) -> RaycastHit:
        """Step along a ray and report first non-FREE hit plus inventory depth.

        Inventory voxels along the ray (before a structural hit, or up to
        ``max_range``) accumulate into ``inventory_penetration_m``.
        """
        origin = np.asarray(origin, dtype=np.float64).reshape(3)
        direction = np.asarray(direction, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(direction))
        if norm < 1e-12:
            return RaycastHit(
                hit=False,
                distance_m=0.0,
                material=MaterialClass.FREE,
                inventory_penetration_m=0.0,
                hit_point=None,
            )
        direction = direction / norm
        max_range = float(max_range)
        if max_range <= 0:
            return RaycastHit(
                hit=False,
                distance_m=0.0,
                material=MaterialClass.FREE,
                inventory_penetration_m=0.0,
                hit_point=None,
            )

        ds = float(step) if step is not None else max(self.resolution * 0.5, 0.05)
        inventory_m = 0.0
        prev_in_inventory = False
        t = 0.0
        hit_material = MaterialClass.FREE
        hit_point: np.ndarray | None = None
        hit = False

        while t <= max_range + 1e-9:
            point = origin + direction * t
            material = self.class_at(point)

            if material == MaterialClass.INVENTORY:
                if prev_in_inventory:
                    inventory_m += ds
                else:
                    # Entering inventory: count half-step from previous free.
                    inventory_m += ds
                prev_in_inventory = True
            else:
                prev_in_inventory = False

            if material == MaterialClass.STRUCTURAL:
                hit = True
                hit_material = MaterialClass.STRUCTURAL
                hit_point = point.copy()
                break

            if material == MaterialClass.INVENTORY and not hit:
                # First inventory surface counts as a soft hit for callers that
                # care about occlusion class, but we keep walking for depth.
                if hit_material == MaterialClass.FREE:
                    hit = True
                    hit_material = MaterialClass.INVENTORY
                    hit_point = point.copy()

            t += ds

        if hit and hit_material == MaterialClass.STRUCTURAL:
            distance = float(np.linalg.norm(hit_point - origin)) if hit_point is not None else t
            return RaycastHit(
                hit=True,
                distance_m=distance,
                material=MaterialClass.STRUCTURAL,
                inventory_penetration_m=inventory_m,
                hit_point=hit_point,
            )

        if hit and hit_material == MaterialClass.INVENTORY:
            # Soft hit: continue reported distance is first inventory entry,
            # penetration is total inventory along the full ray to max_range.
            distance = float(np.linalg.norm(hit_point - origin)) if hit_point is not None else 0.0
            return RaycastHit(
                hit=True,
                distance_m=distance,
                material=MaterialClass.INVENTORY,
                inventory_penetration_m=inventory_m,
                hit_point=hit_point,
            )

        return RaycastHit(
            hit=False,
            distance_m=max_range,
            material=MaterialClass.FREE,
            inventory_penetration_m=inventory_m,
            hit_point=None,
        )

    def inventory_penetration_along_segment(
        self,
        start: np.ndarray,
        end: np.ndarray,
        step: float | None = None,
    ) -> float:
        """Meters of Class-B material along the segment from start to end."""
        start = np.asarray(start, dtype=np.float64).reshape(3)
        end = np.asarray(end, dtype=np.float64).reshape(3)
        delta = end - start
        dist = float(np.linalg.norm(delta))
        if dist < 1e-12:
            return 0.0
        hit = self.raycast(start, delta / dist, dist, step=step)
        # Recompute penetration only up to the segment (raycast already capped).
        return float(hit.inventory_penetration_m)

    def particle_occupancy_mask(
        self,
        particles: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (is_structural, is_inventory) boolean masks for (N, 3) particles."""
        particles = np.asarray(particles, dtype=np.float64)
        n = particles.shape[0]
        is_structural = np.zeros(n, dtype=bool)
        is_inventory = np.zeros(n, dtype=bool)
        for i, p in enumerate(particles):
            material = self.class_at(p)
            if material == MaterialClass.STRUCTURAL:
                is_structural[i] = True
            elif material == MaterialClass.INVENTORY:
                is_inventory[i] = True
        return is_structural, is_inventory
