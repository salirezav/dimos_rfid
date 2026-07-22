"""Synthetic warehouse tests for the semantic RFID particle filter."""

from __future__ import annotations

import numpy as np
import pytest

from dimos_rfid.rfid_tracker import RFIDTracker
from dimos_rfid.semantic_map import MaterialClass, SemanticOccupancyGrid3D
from dimos_rfid.semantic_particle_filter import (
    CARDBOARD_ATTENUATION_PENALTY,
    MULTIPATH_WEIGHT_SCALE,
    SemanticParticleFilter3D,
    distance_to_rssi,
    evaluate_multipath_reading,
    raycast_semantic_check,
    rssi_to_distance,
)


def _empty_map(
    extent: float = 10.0,
    resolution: float = 0.2,
    z_extent: float = 3.0,
) -> SemanticOccupancyGrid3D:
    nx = int(extent / resolution)
    ny = int(extent / resolution)
    nz = int(z_extent / resolution)
    return SemanticOccupancyGrid3D(
        origin=np.array([0.0, 0.0, 0.0]),
        resolution=resolution,
        shape=(nx, ny, nz),
    )


def test_class_at_and_set_box() -> None:
    grid = _empty_map()
    assert grid.class_at([1.0, 1.0, 1.0]) == MaterialClass.FREE
    grid.set_box([2.0, 2.0, 0.0], [3.0, 3.0, 2.0], MaterialClass.STRUCTURAL)
    assert grid.class_at([2.5, 2.5, 1.0]) == MaterialClass.STRUCTURAL
    assert grid.class_at([1.0, 1.0, 1.0]) == MaterialClass.FREE


def test_raycast_structural_hit() -> None:
    grid = _empty_map()
    # Wall at x ∈ [4, 4.4]
    grid.set_box([4.0, 0.0, 0.0], [4.4, 10.0, 3.0], MaterialClass.STRUCTURAL)
    hit = grid.raycast(np.array([1.0, 5.0, 1.0]), np.array([1.0, 0.0, 0.0]), max_range=8.0)
    assert hit.hit
    assert hit.material == MaterialClass.STRUCTURAL
    assert 2.5 < hit.distance_m < 4.0


def test_raycast_inventory_penetration() -> None:
    grid = _empty_map(resolution=0.1)
    # Inventory slab from x=2..4 along the ray.
    grid.set_box([2.0, 4.5, 0.5], [4.0, 5.5, 1.5], MaterialClass.INVENTORY)
    hit = grid.raycast(np.array([0.5, 5.0, 1.0]), np.array([1.0, 0.0, 0.0]), max_range=6.0)
    assert hit.hit
    assert hit.material == MaterialClass.INVENTORY
    assert hit.inventory_penetration_m > 1.5  # ~2 m of cardboard


def test_structural_blocks_particles_behind_wall() -> None:
    """Particles behind a Class-A wall get zero weight; estimate stays in front."""
    grid = _empty_map()
    grid.set_box([4.0, 0.0, 0.0], [4.4, 10.0, 3.0], MaterialClass.STRUCTURAL)

    rng = np.random.default_rng(0)
    pf = SemanticParticleFilter3D(
        n_particles=2000,
        bounds=((0.5, 9.5), (0.5, 9.5), (0.3, 2.0)),
        rng=rng,
    )
    dog = np.array([1.0, 5.0, 1.0])
    # True tag in front of wall.
    true_tag = np.array([3.0, 5.0, 1.0])
    rssi = distance_to_rssi(float(np.linalg.norm(true_tag - dog)))

    for _ in range(8):
        pf.predict(dt=0.2)
        pf.update(dog, yaw=0.0, pitch=0.0, rssi_dbm=rssi, semantic_map=grid)

    particles, weights = pf.get_particles()
    behind = particles[:, 0] > 4.4
    if behind.any():
        assert float(weights[behind].max()) < 1e-6

    mean, confidence = pf.estimate()
    assert mean[0] < 4.0  # estimate on the near side of the wall
    assert confidence > 0.0


def test_inventory_particles_remain_valid() -> None:
    """Class-B occupancy does not zero weights; attenuation increases with depth."""
    grid = _empty_map(resolution=0.15)
    grid.set_box([2.0, 4.0, 0.2], [5.0, 6.0, 2.0], MaterialClass.INVENTORY)

    dog = np.array([0.5, 5.0, 1.0])
    inside = np.array([3.5, 5.0, 1.0])
    behind = np.array([5.5, 5.0, 1.0])

    sem_inside = raycast_semantic_check(grid, dog, inside)
    assert not sem_inside.blocked_by_structural
    assert sem_inside.inside_inventory
    assert sem_inside.inventory_penetration_m > 0.5

    sem_behind = raycast_semantic_check(grid, dog, behind)
    assert not sem_behind.blocked_by_structural
    assert not sem_behind.inside_structural
    assert sem_behind.inventory_penetration_m > sem_inside.inventory_penetration_m

    # Attenuation shifts expected RSSI downward.
    free_expected = distance_to_rssi(float(np.linalg.norm(inside - dog)))
    attenuated = free_expected - CARDBOARD_ATTENUATION_PENALTY * sem_inside.inventory_penetration_m
    assert attenuated < free_expected - 1.0


def test_multipath_structural_discount() -> None:
    grid = _empty_map()
    grid.set_box([3.0, 0.0, 0.0], [3.4, 10.0, 3.0], MaterialClass.STRUCTURAL)
    dog = np.array([1.0, 5.0, 1.0])
    # RSSI implies ~8 m range, but wall is at ~2 m along +X.
    rssi = distance_to_rssi(8.0)
    is_mp, scale = evaluate_multipath_reading(grid, dog, yaw=0.0, pitch=0.0, rssi_dbm=rssi)
    assert is_mp
    assert scale == pytest.approx(MULTIPATH_WEIGHT_SCALE)

    # Clear LOS: no structural obstacle.
    empty = _empty_map()
    is_mp2, scale2 = evaluate_multipath_reading(empty, dog, yaw=0.0, pitch=0.0, rssi_dbm=rssi)
    assert not is_mp2
    assert scale2 == pytest.approx(1.0)


def test_multipath_inventory_is_valid_occluded() -> None:
    grid = _empty_map(resolution=0.15)
    grid.set_box([2.0, 4.0, 0.2], [4.0, 6.0, 2.0], MaterialClass.INVENTORY)
    dog = np.array([0.5, 5.0, 1.0])
    rssi = distance_to_rssi(6.0)
    is_mp, scale = evaluate_multipath_reading(grid, dog, yaw=0.0, pitch=0.0, rssi_dbm=rssi)
    assert not is_mp
    assert scale == pytest.approx(1.0)


def test_tracker_api_converges_on_los_target() -> None:
    grid = _empty_map()
    bounds = ((0.5, 9.5), (0.5, 9.5), (0.3, 2.0))
    tracker = RFIDTracker(bounds=bounds, n_particles=2500, rng=np.random.default_rng(1))
    tag_id = "e280toi0001"
    dog = (1.0, 5.0, 1.0)
    true_tag = np.array([4.0, 5.0, 1.0])
    rssi = distance_to_rssi(float(np.linalg.norm(true_tag - np.array(dog))))

    assert tracker.get_estimated_target_location(tag_id) is None
    assert tracker.get_location_confidence(tag_id) == 0.0

    for i in range(12):
        tracker.ingest(
            *dog,
            dog_yaw=0.0,
            dog_pitch=0.0,
            tag_id=tag_id,
            rssi=rssi,
            semantic_lidar_map=grid,
            timestamp=float(i),
        )

    loc = tracker.get_estimated_target_location(tag_id)
    conf = tracker.get_location_confidence(tag_id)
    assert loc is not None
    assert conf > 0.2
    # Should be pulled toward the true tag along the beam.
    assert abs(loc[1] - 5.0) < 1.5
    assert loc[0] > 2.0


def test_rssi_distance_roundtrip() -> None:
    d = 3.5
    rssi = distance_to_rssi(d)
    assert rssi_to_distance(rssi) == pytest.approx(d, rel=1e-6)
