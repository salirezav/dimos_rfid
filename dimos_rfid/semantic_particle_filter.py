"""3D semantic particle filter for RFID tag-of-interest localization.

Fuses unidirectional RSSI, robot pose (beam cone), and a semantic occupancy
map that distinguishes structural obstacles from penetrable inventory.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dimos_rfid.semantic_map import MaterialClass, SemanticMapProtocol

# ---------------------------------------------------------------------------
# Tunable constants (exposed at module top for easy calibration)
# ---------------------------------------------------------------------------

DEFAULT_N_PARTICLES = 5000

# dBm reduction in expected RSSI per meter of Class-B (cardboard / pallet) penetration.
CARDBOARD_ATTENUATION_PENALTY = 8.0

# Weight assigned to particles inside / behind Class-A structure.
STRUCTURAL_WEIGHT = 0.0

# Scale applied to all particle weight updates when a reading is flagged multipath.
MULTIPATH_WEIGHT_SCALE = 0.05

# Unidirectional scanner beam half-angle (degrees) around antenna forward axis.
BEAM_HALF_ANGLE_DEG = 45.0

# Soft floor for cone likelihood outside the hard beam (avoids total collapse).
BEAM_OUTSIDE_LIKELIHOOD = 1e-3

# Log-distance path-loss model: RSSI(d) = RSSI_REF - 10 * n * log10(d).
RSSI_REF_DBM = -50.0
PATH_LOSS_N = 2.2
RSSI_SIGMA_DBM = 6.0  # observation noise for likelihood

# Process noise for predict() random walk (meters, per sqrt(second) scale via dt).
PROCESS_NOISE_STD_M = 0.05

# Resample when N_eff / N drops below this fraction.
RESAMPLE_ESS_FRACTION = 0.5

# Confidence maps RMS spatial std into [0, 1]; this std (m) → confidence ~0.
CONFIDENCE_STD_REF_M = 2.0


@dataclass(frozen=True)
class SemanticRayResult:
    """Outcome of a dog→particle (or dog→range) semantic ray check."""

    blocked_by_structural: bool
    behind_structural: bool
    inside_structural: bool
    inside_inventory: bool
    inventory_penetration_m: float
    first_hit_material: MaterialClass
    first_hit_distance_m: float
    path_length_m: float


def rssi_to_distance(rssi_dbm: float, rssi_ref_dbm: float = RSSI_REF_DBM, n: float = PATH_LOSS_N) -> float:
    """Invert the log-distance path-loss model to an approximate range (m)."""
    return float(10.0 ** ((rssi_ref_dbm - rssi_dbm) / (10.0 * n)))


def distance_to_rssi(distance_m: float, rssi_ref_dbm: float = RSSI_REF_DBM, n: float = PATH_LOSS_N) -> float:
    """Expected free-space RSSI (dBm) at a given range."""
    d = max(float(distance_m), 0.05)
    return float(rssi_ref_dbm - 10.0 * n * np.log10(d))


def antenna_forward_axis(yaw: float, pitch: float = 0.0) -> np.ndarray:
    """Unit forward vector from robot yaw/pitch (radians, ROS-like Z-up)."""
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    # Yaw about Z, then pitch about Y (nose up positive pitch).
    return np.array([cp * cy, cp * sy, -sp], dtype=np.float64)


def raycast_semantic_check(
    semantic_map: SemanticMapProtocol,
    dog_pos: np.ndarray,
    target_pos: np.ndarray,
    *,
    step: float | None = None,
) -> SemanticRayResult:
    """Evaluate material class between the dog and a point (particle or hypothesis).

    - Particles *inside* Class A → physically impossible.
    - If a Class A surface is hit *before* reaching the target, the target is
      *behind* structure (also impossible for a true LOS origin).
    - Class B along the path contributes inventory penetration length.
    """
    dog_pos = np.asarray(dog_pos, dtype=np.float64).reshape(3)
    target_pos = np.asarray(target_pos, dtype=np.float64).reshape(3)
    delta = target_pos - dog_pos
    path_length = float(np.linalg.norm(delta))

    material_at_target = semantic_map.class_at(target_pos)
    inside_structural = material_at_target == MaterialClass.STRUCTURAL
    inside_inventory = material_at_target == MaterialClass.INVENTORY

    if path_length < 1e-9:
        return SemanticRayResult(
            blocked_by_structural=inside_structural,
            behind_structural=False,
            inside_structural=inside_structural,
            inside_inventory=inside_inventory,
            inventory_penetration_m=0.0,
            first_hit_material=material_at_target,
            first_hit_distance_m=0.0,
            path_length_m=0.0,
        )

    direction = delta / path_length
    hit = semantic_map.raycast(dog_pos, direction, path_length, step=step)

    behind_structural = False
    blocked = inside_structural
    first_material = MaterialClass.FREE
    first_dist = path_length

    if hit.hit and hit.material == MaterialClass.STRUCTURAL:
        # Structural surface before (or at) the target.
        if hit.distance_m < path_length - 1e-6:
            behind_structural = True
            blocked = True
        first_material = MaterialClass.STRUCTURAL
        first_dist = hit.distance_m
    elif hit.hit and hit.material == MaterialClass.INVENTORY:
        first_material = MaterialClass.INVENTORY
        first_dist = hit.distance_m

    return SemanticRayResult(
        blocked_by_structural=blocked,
        behind_structural=behind_structural,
        inside_structural=inside_structural,
        inside_inventory=inside_inventory,
        inventory_penetration_m=float(hit.inventory_penetration_m),
        first_hit_material=first_material,
        first_hit_distance_m=float(first_dist),
        path_length_m=path_length,
    )


def evaluate_multipath_reading(
    semantic_map: SemanticMapProtocol,
    dog_pos: np.ndarray,
    yaw: float,
    pitch: float,
    rssi_dbm: float,
    *,
    step: float | None = None,
) -> tuple[bool, float]:
    """LOS gate along the scanner boresight to the RSSI-implied range.

    Returns ``(is_probable_multipath, weight_scale)``.
    Class A hit before the estimated distance → multipath (heavy discount).
    Class B hit → valid occluded reading (scale 1.0; attenuation handled in PF).
    """
    dog_pos = np.asarray(dog_pos, dtype=np.float64).reshape(3)
    forward = antenna_forward_axis(yaw, pitch)
    est_range = rssi_to_distance(rssi_dbm)
    hit = semantic_map.raycast(dog_pos, forward, est_range, step=step)

    if hit.hit and hit.material == MaterialClass.STRUCTURAL and hit.distance_m < est_range - 1e-3:
        return True, MULTIPATH_WEIGHT_SCALE
    return False, 1.0


class SemanticParticleFilter3D:
    """Monte Carlo localization of a single RFID tag in 3D."""

    def __init__(
        self,
        n_particles: int = DEFAULT_N_PARTICLES,
        bounds: tuple[tuple[float, float], tuple[float, float], tuple[float, float]] | None = None,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.n_particles = int(n_particles)
        self.rng = rng if rng is not None else np.random.default_rng()
        self.particles = np.zeros((self.n_particles, 3), dtype=np.float64)
        self.weights = np.full(self.n_particles, 1.0 / self.n_particles, dtype=np.float64)
        self.bounds = bounds
        self._initialized = False
        if bounds is not None:
            self.initialize(bounds)

    def initialize(
        self,
        bounds: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    ) -> None:
        """Scatter particles uniformly across ``((xmin,xmax),(ymin,ymax),(zmin,zmax))``."""
        self.bounds = bounds
        (xmin, xmax), (ymin, ymax), (zmin, zmax) = bounds
        self.particles[:, 0] = self.rng.uniform(xmin, xmax, self.n_particles)
        self.particles[:, 1] = self.rng.uniform(ymin, ymax, self.n_particles)
        self.particles[:, 2] = self.rng.uniform(zmin, zmax, self.n_particles)
        self.weights[:] = 1.0 / self.n_particles
        self._initialized = True

    def predict(self, dt: float = 1.0) -> None:
        """Diffuse particles with isotropic Gaussian process noise."""
        if not self._initialized:
            return
        sigma = PROCESS_NOISE_STD_M * np.sqrt(max(float(dt), 1e-6))
        self.particles += self.rng.normal(0.0, sigma, size=self.particles.shape)
        if self.bounds is not None:
            (xmin, xmax), (ymin, ymax), (zmin, zmax) = self.bounds
            self.particles[:, 0] = np.clip(self.particles[:, 0], xmin, xmax)
            self.particles[:, 1] = np.clip(self.particles[:, 1], ymin, ymax)
            self.particles[:, 2] = np.clip(self.particles[:, 2], zmin, zmax)

    def update(
        self,
        dog_pos: np.ndarray,
        yaw: float,
        pitch: float,
        rssi_dbm: float,
        semantic_map: SemanticMapProtocol,
        *,
        multipath_discount: float = 1.0,
    ) -> None:
        """Reweight particles from RSSI, beam cone, and semantic constraints."""
        if not self._initialized:
            raise RuntimeError("Particle filter not initialized; call initialize() first.")

        dog_pos = np.asarray(dog_pos, dtype=np.float64).reshape(3)
        forward = antenna_forward_axis(yaw, pitch)
        half_angle = np.deg2rad(BEAM_HALF_ANGLE_DEG)
        cos_half = float(np.cos(half_angle))

        deltas = self.particles - dog_pos[None, :]
        distances = np.linalg.norm(deltas, axis=1)
        distances_safe = np.maximum(distances, 0.05)

        # Directional cone likelihood.
        unit = deltas / distances_safe[:, None]
        cos_angles = unit @ forward
        in_cone = cos_angles >= cos_half
        # Soft angular falloff outside the cone.
        cone_like = np.where(
            in_cone,
            0.5 + 0.5 * np.clip((cos_angles - cos_half) / max(1.0 - cos_half, 1e-6), 0.0, 1.0),
            BEAM_OUTSIDE_LIKELIHOOD * np.clip(cos_angles + 1.0, 0.0, 1.0),
        )

        # Per-particle semantic checks + inventory attenuation.
        likelihood = np.zeros(self.n_particles, dtype=np.float64)
        for i in range(self.n_particles):
            sem = raycast_semantic_check(semantic_map, dog_pos, self.particles[i])
            if sem.blocked_by_structural or sem.behind_structural or sem.inside_structural:
                likelihood[i] = STRUCTURAL_WEIGHT
                continue

            expected = distance_to_rssi(distances_safe[i])
            expected -= CARDBOARD_ATTENUATION_PENALTY * sem.inventory_penetration_m
            # Gaussian RSSI likelihood in dBm space.
            resid = float(rssi_dbm) - expected
            rssi_like = float(np.exp(-0.5 * (resid / RSSI_SIGMA_DBM) ** 2))
            likelihood[i] = rssi_like * float(cone_like[i])

        likelihood *= float(multipath_discount)
        self.weights *= likelihood

        w_sum = float(self.weights.sum())
        if w_sum <= 0.0 or not np.isfinite(w_sum):
            # Degenerate: reset to uniform (observation rejected / all blocked).
            self.weights[:] = 1.0 / self.n_particles
        else:
            self.weights /= w_sum

        if self.effective_sample_size() < RESAMPLE_ESS_FRACTION * self.n_particles:
            self.resample()

    def effective_sample_size(self) -> float:
        return float(1.0 / np.sum(self.weights**2))

    def resample(self) -> None:
        """Systematic resampling."""
        n = self.n_particles
        positions = (self.rng.random() + np.arange(n)) / n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0  # numerical safety
        indexes = np.searchsorted(cumulative, positions)
        self.particles = self.particles[indexes].copy()
        self.weights[:] = 1.0 / n

    def estimate(self) -> tuple[np.ndarray, float]:
        """Return (weighted_mean_xyz, confidence in [0, 1])."""
        if not self._initialized:
            return np.full(3, np.nan), 0.0

        mean = (self.weights[:, None] * self.particles).sum(axis=0)
        centered = self.particles - mean[None, :]
        # Weighted covariance trace → RMS-like spatial std.
        var = (self.weights[:, None] * (centered**2)).sum(axis=0)
        rms_std = float(np.sqrt(var.sum() / 3.0))
        confidence = float(np.clip(1.0 - rms_std / CONFIDENCE_STD_REF_M, 0.0, 1.0))
        return mean.astype(np.float64), confidence

    def get_particles(self) -> tuple[np.ndarray, np.ndarray]:
        """Return copies of (particles, weights)."""
        return self.particles.copy(), self.weights.copy()
