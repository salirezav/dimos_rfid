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
world position (gray "estimating" dot -> green "located" dot). The marker also
appears as a 2D overlay on the camera image, but only when it projects inside the
frame — i.e. when the dog is looking toward where the tag was found.

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


@dataclass
class _Obs:
    """One RFID sighting: where the dog was and how strong the signal was."""

    pos: np.ndarray  # dog position (x, y, z) in the world frame at sighting
    rssi: float  # RSSI in dBm (less negative == stronger == closer)
    ts: float


@dataclass
class _TagLocalizer:
    """Estimates a tag's world position from RSSI seen at multiple dog poses.

    A single UHF-RFID reading gives only signal strength (no bearing, no range),
    so one sighting can't localize a tag. As the dog observes the same EPC from
    several *spatially distinct* positions, each RSSI is converted to a rough
    range via a log-distance path-loss model, and the tag position is solved by
    linear least-squares multilateration (in the ground plane; z is taken as the
    mean sighting height since a ground robot barely changes altitude).

    Until there are enough well-spread sightings, ``estimate()`` returns a
    low-quality "placeholder" (the RSSI-weighted centroid of the dog poses).
    """

    rssi_ref_dbm: float = -50.0  # expected RSSI at 1 m (reference power)
    path_loss_n: float = 2.0  # path-loss exponent (2 free space, 2-4 indoor)
    min_baseline_m: float = 0.3  # min dog travel before logging a new sighting
    max_range_m: float = 15.0  # reject solutions this far from the sightings
    obs: list[_Obs] = field(default_factory=list)

    def add(self, pos: np.ndarray, rssi: float, ts: float) -> None:
        """Record a sighting, but only from a meaningfully new dog position.

        Re-detections from ~the same spot don't add spatial information; we keep
        the strongest RSSI seen there instead of piling up duplicates.
        """
        pos = np.asarray(pos, dtype=float)
        if self.obs and np.linalg.norm(pos - self.obs[-1].pos) < self.min_baseline_m:
            if rssi > self.obs[-1].rssi:
                self.obs[-1] = _Obs(self.obs[-1].pos, float(rssi), ts)
            return
        self.obs.append(_Obs(pos, float(rssi), ts))

    def _rssi_to_distance(self, rssi: np.ndarray) -> np.ndarray:
        """Log-distance path-loss model: d = 10 ** ((rssi_ref - rssi) / (10 n))."""
        return 10.0 ** ((self.rssi_ref_dbm - rssi) / (10.0 * self.path_loss_n))

    def estimate(self) -> tuple[np.ndarray, float, int]:
        """Return (position_xyz, quality[0..1], n_observations).

        quality < ~0.4 means "still estimating" (placeholder); higher means a
        multilateration fit backed by several spread-out sightings.
        """
        n = len(self.obs)
        pts = np.array([o.pos for o in self.obs])
        rssi = np.array([o.rssi for o in self.obs])

        # RSSI-weighted centroid (linear power weights) — robust fallback/prior.
        weights = 10.0 ** (rssi / 10.0)
        centroid = (weights[:, None] * pts).sum(axis=0) / weights.sum()

        if n < 3:
            # Not enough baselines to trilaterate yet: placeholder only.
            return centroid, min(0.3, 0.1 * n), n

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
            return centroid, 0.3, n
        if rank < 2 or not np.all(np.isfinite(sol)):
            return centroid, 0.3, n

        est = np.array([sol[0], sol[1], float(pts[:, 2].mean())])
        if np.linalg.norm(est[:2] - pts[:, :2].mean(axis=0)) > self.max_range_m:
            # Degenerate/noisy solve flew off; trust the centroid instead.
            return centroid, 0.3, n

        # More sightings + wider spatial spread -> higher confidence.
        spread = float(np.linalg.norm(pts[:, :2].std(axis=0)))
        quality = min(1.0, 0.4 + 0.1 * n) * min(1.0, spread / 1.0)
        return est, max(0.4, quality), n


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
        default=0.3, gt=0, description="Min dog travel before recording a new sighting."
    )


# Rerun entity path for the standalone RFID text panel.
RERUN_ENTITY = "rfid"

# Where spatial markers live in the viewer's entity tree.
MARKERS_3D_ENTITY = "world/rfid/markers"  # 3D world view
# Camera overlays go *under* the color-image pinhole entity so they land in pixel
# space and only render when the point projects inside the frame.
CAMERA_IMAGE_ENTITY = "world/color_image"


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
        if self.config.spatial:
            # Touch the TF listener early so /tf is buffering before we look up poses.
            _ = self.tf
        self._stop_flag = threading.Event()
        self._poll_task = self.spawn(self._poll_loop())

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

    @staticmethod
    def _print_tags(payload: dict) -> None:
        count = payload.get("count", 0)
        tags = payload.get("tags", []) or []
        if not tags:
            print(f"[RFID] {count} tag(s) in range")
            return
        print(f"[RFID] {count} tag(s) in range:")
        for tag in tags:
            epc = tag.get("epc", "?")
            rssi = tag.get("rssi_dbm")
            rssi_s = f"{rssi} dBm" if rssi is not None else "unknown RSSI"
            print(f"    EPC={epc}  RSSI={rssi_s}")

    @staticmethod
    def _tags_markdown(payload: dict) -> str:
        count = payload.get("count", 0)
        tags = payload.get("tags", []) or []
        lines = [f"# RFID — {count} tag(s) in range", ""]
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

        # Where is the dog right now? Without a pose we can't add sightings.
        dog = self._tf_matrix(self.config.world_frame, self.config.base_frame)
        if dog is not None:
            dog_pos = dog[:3, 3]
            now = time.time()
            for tag in payload.get("tags", []) or []:
                epc = tag.get("epc")
                rssi = tag.get("rssi_dbm")
                if not epc or rssi is None:
                    continue
                loc = self._locs.get(epc)
                if loc is None:
                    loc = _TagLocalizer(
                        rssi_ref_dbm=self.config.rssi_ref_dbm,
                        path_loss_n=self.config.path_loss_n,
                        min_baseline_m=self.config.min_baseline_m,
                    )
                    self._locs[epc] = loc
                loc.add(dog_pos, float(rssi), now)

        # world -> camera_optical, for projecting markers into the image.
        cam_from_world = self._tf_matrix(self.config.camera_frame, self.config.world_frame)

        for epc, loc in self._locs.items():
            if not loc.obs:
                continue
            est, quality, n_obs = loc.estimate()
            self._log_marker_3d(rr, epc, est, quality, n_obs)
            self._log_marker_camera(rr, epc, est, quality, cam_from_world)

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

    @staticmethod
    def _quality_color(quality: float) -> list[int]:
        """Gray while still estimating, green once well localized."""
        if quality < 0.4:
            return [150, 150, 150]
        return [40, 200, 90]

    def _log_marker_3d(
        self, rr: Any, epc: str, est: np.ndarray, quality: float, n_obs: int
    ) -> None:
        state = "estimating" if quality < 0.4 else "located"
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
    try:
        from dimos.hardware.sensors.rfid.bridge import RFID_RERUN_ENTITY
        from dimos.hardware.sensors.rfid.rfid_rerun import go2_rfid_rerun_config
    except ModuleNotFoundError:
        # Local development: the package lives at dimos_rfid/ in this repo.
        from dimos_rfid.rfid_rerun import RFID_RERUN_ENTITY, go2_rfid_rerun_config
    try:
        from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
    except ModuleNotFoundError as exc:
        logger.error(
            "Go2/Unitree dependencies are missing: %s.\n"
            "Install the Unitree extras in your environment, e.g. `pip install 'dimos[unitree]'`",
            exc,
        )
        return
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    # Log into the bridge's viewer (don't spawn our own) at the panel's entity.
    # Enable spatial markers: the robot pose is available in this mode.
    config = config.model_copy(
        update={
            "rerun": True,
            "rerun_spawn": False,
            "rerun_entity": RFID_RERUN_ENTITY,
            "spatial": True,
        }
    )

    logger.info("Starting Go2 + RFID with the DimOS viewer (Ctrl-C to stop)...")
    blueprint = autoconnect(
        unitree_go2,
        RFIDModule.blueprint(config=config),
        # Same class as the Go2's own bridge, so this overrides its layout
        # (later module wins) -> camera | 3D | RFID.
        RerunBridgeModule.blueprint(**go2_rfid_rerun_config()),
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
