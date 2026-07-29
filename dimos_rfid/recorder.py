# Copyright 2026. RFID DimOS integration.
#
# Offline recorder for synchronized RFID, Go2 camera, and odometry data.

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import csv
import json
import math
import os
from pathlib import Path
import queue
import re
import shutil
import threading
import time
from typing import Any

import cv2
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

from dimos_rfid.collection_context import read_collection_context
from dimos_rfid.msgs import RfidTagArray

logger = setup_logger()

SCHEMA_VERSION = "1.0"
_STOP = object()


def _utc_iso(timestamp: float) -> str:
    return datetime.fromtimestamp(timestamp, timezone.utc).isoformat()


def _safe_session_name(value: str) -> str:
    """Return a filesystem-safe session name, or a UTC timestamp when empty."""
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip()).strip("._-")
    return value or datetime.now(timezone.utc).strftime("rfid_%Y%m%dT%H%M%SZ")


def _pose_to_dict(pose: PoseStamped, *, sample_ts: float | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "timestamp": float(pose.ts),
        "timestamp_iso": _utc_iso(float(pose.ts)),
        "frame_id": pose.frame_id,
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation_xyzw": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }
    if sample_ts is not None:
        result["age_seconds"] = float(sample_ts - pose.ts)
    return result


def _rfid_to_dict(sample: RfidTagArray) -> dict[str, Any]:
    return {
        "timestamp": float(sample.ts),
        "timestamp_iso": _utc_iso(float(sample.ts)),
        "frame_id": sample.frame_id,
        "active_count": int(sample.active_count),
        "total_count": int(sample.total_count),
        "connection_status": sample.connection_status,
        "reader_host": sample.reader_host,
        "reader_device_id": sample.reader_device_id,
        "reader_started": sample.reader_started,
        "stale_seconds": sample.stale_seconds,
        "source_updated_at": sample.source_updated_at,
        "scanner_status": sample.scanner_status,
        "tags": [asdict(tag) for tag in sample.tags],
    }


@dataclass
class _CapturedSample:
    sequence: int
    received_at: float
    monotonic_ns: int
    rfid: dict[str, Any]
    robot_pose: dict[str, Any] | None
    image_data: np.ndarray[Any, np.dtype[Any]] | None
    image_timestamp: float | None
    image_frame_id: str
    image_format: str
    collection: dict[str, Any] | None = None
    motion: dict[str, Any] | None = None


@dataclass
class _CapturedPointCloudMap:
    points: np.ndarray[Any, np.dtype[Any]]
    colors: np.ndarray[Any, np.dtype[Any]] | None
    timestamp: float | None
    frame_id: str
    update_count: int = 0


class _SessionWriter:
    """Disk writer kept separate from DimOS so it can be tested and reused."""

    def __init__(
        self,
        output_root: Path,
        session_name: str,
        *,
        jpeg_quality: int,
        user_metadata: dict[str, Any] | None = None,
    ) -> None:
        self.output_root = output_root
        self.session_id = _safe_session_name(session_name)
        self.jpeg_quality = jpeg_quality
        self.user_metadata = dict(user_metadata or {})
        self.started_at = time.time()
        self.observation_count = 0
        self.image_count = 0
        self.tag_read_count = 0
        self.image_errors = 0

        output_root.mkdir(parents=True, exist_ok=True)
        session_dir = output_root / self.session_id
        if session_dir.exists():
            suffix = datetime.now(timezone.utc).strftime("%H%M%S_%f")
            self.session_id = f"{self.session_id}_{suffix}"
            session_dir = output_root / self.session_id
        self.session_dir = session_dir
        self.images_dir = session_dir / "images"
        self.images_dir.mkdir(parents=True)
        self._observations_file = (session_dir / "observations.jsonl").open(
            "w", encoding="utf-8", buffering=1
        )

    def write_sample(self, sample: _CapturedSample) -> None:
        image_info: dict[str, Any] | None = None
        if sample.image_data is not None and sample.image_timestamp is not None:
            relative_path = Path("images") / f"{sample.sequence:08d}.jpg"
            target = self.session_dir / relative_path
            try:
                success, encoded = cv2.imencode(
                    ".jpg",
                    sample.image_data,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )
                if not success:
                    raise ValueError("OpenCV JPEG encoder returned false")
                target.write_bytes(encoded.tobytes())
                height, width = sample.image_data.shape[:2]
                image_info = {
                    "path": relative_path.as_posix(),
                    "timestamp": sample.image_timestamp,
                    "timestamp_iso": _utc_iso(sample.image_timestamp),
                    "age_seconds": sample.received_at - sample.image_timestamp,
                    "frame_id": sample.image_frame_id,
                    "source_format": sample.image_format,
                    "width": int(width),
                    "height": int(height),
                }
                self.image_count += 1
            except Exception as exc:  # noqa: BLE001 - retain RFID/pose even if JPEG fails
                self.image_errors += 1
                image_info = {"error": str(exc)}

        record = {
            "schema_version": SCHEMA_VERSION,
            "sequence": sample.sequence,
            "received_at": sample.received_at,
            "received_at_iso": _utc_iso(sample.received_at),
            "monotonic_ns": sample.monotonic_ns,
            "image": image_info,
            "robot_pose": sample.robot_pose,
            "rfid": sample.rfid,
        }
        if sample.motion:
            record["motion"] = sample.motion
        if sample.collection:
            record["collection"] = sample.collection
        self._observations_file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.observation_count += 1
        self.tag_read_count += len(sample.rfid["tags"])

    @staticmethod
    def _path_length(trajectory: list[dict[str, Any]]) -> float:
        total = 0.0
        for previous, current in zip(trajectory, trajectory[1:]):
            p1 = previous["position"]
            p2 = current["position"]
            total += math.dist((p1["x"], p1["y"], p1["z"]), (p2["x"], p2["y"], p2["z"]))
        return total

    def _write_trajectory(self, trajectory: list[dict[str, Any]]) -> float:
        path_length = self._path_length(trajectory)
        (self.session_dir / "trajectory.json").write_text(
            json.dumps(
                {
                    "schema_version": SCHEMA_VERSION,
                    "frame_id": trajectory[0]["frame_id"] if trajectory else "",
                    "path_length_m": path_length,
                    "poses": trajectory,
                },
                indent=2,
            ),
            encoding="utf-8",
        )

        with (self.session_dir / "trajectory.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["timestamp", "frame_id", "x", "y", "z", "qx", "qy", "qz", "qw"])
            for pose in trajectory:
                p = pose["position"]
                q = pose["orientation_xyzw"]
                writer.writerow(
                    [
                        pose["timestamp"],
                        pose["frame_id"],
                        p["x"],
                        p["y"],
                        p["z"],
                        q["x"],
                        q["y"],
                        q["z"],
                        q["w"],
                    ]
                )
        self._write_trajectory_image(trajectory, path_length)
        return path_length

    def _write_trajectory_image(
        self, trajectory: list[dict[str, Any]], path_length: float
    ) -> None:
        canvas_size = 1000
        margin = 80
        canvas = np.full((canvas_size, canvas_size, 3), 255, dtype=np.uint8)
        if trajectory:
            xy = np.asarray(
                [[pose["position"]["x"], pose["position"]["y"]] for pose in trajectory],
                dtype=float,
            )
            minimum = xy.min(axis=0)
            maximum = xy.max(axis=0)
            span = np.maximum(maximum - minimum, 1e-6)
            scale = min((canvas_size - 2 * margin) / span[0], (canvas_size - 2 * margin) / span[1])
            pixels = (xy - minimum) * scale
            pixels[:, 0] += margin
            pixels[:, 1] = canvas_size - margin - pixels[:, 1]
            points = np.rint(pixels).astype(np.int32).reshape((-1, 1, 2))
            if len(points) > 1:
                cv2.polylines(canvas, [points], False, (220, 90, 30), 4, cv2.LINE_AA)
            cv2.circle(canvas, tuple(points[0, 0]), 10, (30, 180, 30), -1, cv2.LINE_AA)
            cv2.circle(canvas, tuple(points[-1, 0]), 10, (30, 30, 220), -1, cv2.LINE_AA)
        cv2.putText(
            canvas,
            f"Go2 trajectory: {path_length:.2f} m  (green=start, red=end)",
            (30, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (30, 30, 30),
            2,
            cv2.LINE_AA,
        )
        cv2.imwrite(str(self.session_dir / "trajectory.png"), canvas)

    def _write_pointcloud_map(
        self, pointcloud_map: _CapturedPointCloudMap | None
    ) -> dict[str, Any]:
        """Write the final accumulated point-cloud map as NPZ and binary PLY."""
        if pointcloud_map is None:
            return {
                "available": False,
                "reason": "No global_map message was received during this session.",
            }

        points = np.asarray(pointcloud_map.points, dtype=np.float32).reshape((-1, 3))
        finite = np.all(np.isfinite(points), axis=1)
        points = points[finite]
        colors = pointcloud_map.colors
        if colors is not None:
            colors = np.asarray(colors).reshape((-1, 3))
            if len(colors) == len(finite):
                colors = colors[finite]
            else:
                colors = None

        if colors is not None:
            if np.issubdtype(colors.dtype, np.floating) and len(colors) and colors.max() <= 1.0:
                colors = colors * 255.0
            colors = np.clip(colors, 0, 255).astype(np.uint8)

        npz_path = self.session_dir / "pointcloud_map.npz"
        np.savez_compressed(
            npz_path,
            points=points,
            colors=(colors if colors is not None else np.empty((0, 3), dtype=np.uint8)),
            frame_id=np.asarray(pointcloud_map.frame_id),
            timestamp=np.asarray(
                pointcloud_map.timestamp if pointcloud_map.timestamp is not None else np.nan,
                dtype=np.float64,
            ),
        )
        ply_path = self.session_dir / "pointcloud_map.ply"
        self._write_binary_ply(ply_path, points, colors)

        bounds = None
        if len(points):
            bounds = {
                "min_xyz": points.min(axis=0).astype(float).tolist(),
                "max_xyz": points.max(axis=0).astype(float).tolist(),
            }
        return {
            "available": True,
            "source_stream": "global_map",
            "frame_id": pointcloud_map.frame_id,
            "timestamp": pointcloud_map.timestamp,
            "timestamp_iso": (
                _utc_iso(pointcloud_map.timestamp)
                if pointcloud_map.timestamp is not None
                else None
            ),
            "point_count": int(len(points)),
            "update_count": pointcloud_map.update_count,
            "has_colors": colors is not None,
            "bounds": bounds,
            "npz_path": npz_path.name,
            "ply_path": ply_path.name,
        }

    @staticmethod
    def _write_binary_ply(
        target: Path,
        points: np.ndarray[Any, np.dtype[Any]],
        colors: np.ndarray[Any, np.dtype[Any]] | None,
    ) -> None:
        """Write a little-endian binary PLY without requiring an Open3D writer."""
        has_colors = colors is not None
        fields: list[tuple[str, str]] = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
        if has_colors:
            fields.extend([("red", "u1"), ("green", "u1"), ("blue", "u1")])
        vertices = np.empty(len(points), dtype=np.dtype(fields))
        vertices["x"], vertices["y"], vertices["z"] = points.T
        if colors is not None:
            vertices["red"], vertices["green"], vertices["blue"] = colors.T

        header_lines = [
            "ply",
            "format binary_little_endian 1.0",
            "comment generated by dimos_rfid RfidRecorderModule",
            f"element vertex {len(points)}",
            "property float x",
            "property float y",
            "property float z",
        ]
        if has_colors:
            header_lines.extend(
                ["property uchar red", "property uchar green", "property uchar blue"]
            )
        header_lines.extend(["end_header", ""])
        with target.open("wb") as file:
            file.write("\n".join(header_lines).encode("ascii"))
            file.write(vertices.tobytes())

    def finish(
        self,
        trajectory: list[dict[str, Any]],
        pointcloud_map: _CapturedPointCloudMap | None = None,
        *,
        dropped_samples: int,
        create_archive: bool,
    ) -> dict[str, Any]:
        self._observations_file.flush()
        self._observations_file.close()
        completed_at = time.time()
        path_length = self._write_trajectory(trajectory)
        pointcloud_metadata = self._write_pointcloud_map(pointcloud_map)
        metadata = {
            "schema_version": SCHEMA_VERSION,
            "session_id": self.session_id,
            "started_at": self.started_at,
            "started_at_iso": _utc_iso(self.started_at),
            "completed_at": completed_at,
            "completed_at_iso": _utc_iso(completed_at),
            "duration_seconds": completed_at - self.started_at,
            "observation_count": self.observation_count,
            "image_count": self.image_count,
            "image_error_count": self.image_errors,
            "tag_read_count": self.tag_read_count,
            "trajectory_pose_count": len(trajectory),
            "path_length_m": path_length,
            "pointcloud_map": pointcloud_metadata,
            "dropped_sample_count": dropped_samples,
            "files": {
                "observations": "observations.jsonl",
                "images": "images/",
                "trajectory_json": "trajectory.json",
                "trajectory_csv": "trajectory.csv",
                "trajectory_preview": "trajectory.png",
                "pointcloud_map_npz": (
                    "pointcloud_map.npz" if pointcloud_metadata["available"] else None
                ),
                "pointcloud_map_ply": (
                    "pointcloud_map.ply" if pointcloud_metadata["available"] else None
                ),
            },
            "user_metadata": self.user_metadata,
        }
        (self.session_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )

        archive_path: Path | None = None
        if create_archive:
            archive_path = Path(
                shutil.make_archive(
                    str(self.output_root / self.session_id),
                    "zip",
                    root_dir=self.output_root,
                    base_dir=self.session_id,
                )
            )
        return {
            "session_id": self.session_id,
            "session_dir": str(self.session_dir),
            "archive_path": str(archive_path) if archive_path else None,
            **metadata,
        }


class RfidRecorderConfig(ModuleConfig):
    """Configuration for :class:`RfidRecorderModule`."""

    output_dir: str = Field(
        default_factory=lambda: os.environ.get(
            "RFID_DATASET_DIR",
            str(Path.home() / "Downloads" / "dimos_rfid_datasets"),
        ),
        description="Folder on the DimOS host (normally the user's laptop) receiving sessions.",
    )
    auto_start: bool = Field(
        default=False,
        description="Begin a session when the module starts; useful for the go2-dataset blueprint.",
    )
    session_name: str = Field(
        default="", description="Optional name used for auto-started sessions."
    )
    jpeg_quality: int = Field(default=90, ge=1, le=100)
    max_pending_samples: int = Field(default=128, ge=1)
    trajectory_min_distance_m: float = Field(default=0.02, ge=0)
    trajectory_max_interval_s: float = Field(default=0.5, gt=0)
    create_archive_on_stop: bool = True
    # Skip JPEGs until the dog moves this far since the last saved image.
    min_image_move_m: float = Field(
        default_factory=lambda: float(
            os.environ.get("RFID_DATASET_MIN_IMAGE_MOVE_M", "0.25")
        ),
        ge=0.0,
        description="Minimum XY travel (m) before saving another camera JPEG.",
    )
    # Classify each observation as moving vs stationary from recent odom.
    stationary_speed_mps: float = Field(
        default_factory=lambda: float(
            os.environ.get("RFID_DATASET_STATIONARY_SPEED_MPS", "0.05")
        ),
        ge=0.0,
        description="Speeds at or below this (m/s) count as stationary.",
    )
    waypoint_path_file: str = Field(
        default_factory=lambda: os.environ.get(
            "RFID_WAYPOINT_CAPTURE_PATH",
            str(Path(__file__).resolve().parent / "paths" / "captured_path.json"),
        ),
        description="JSON path file updated by mark_waypoint / path recording.",
    )
    path_record_min_distance_m: float = Field(
        default_factory=lambda: float(
            os.environ.get("RFID_PATH_RECORD_MIN_DISTANCE_M", "0.35")
        ),
        ge=0.05,
        description="While path-recording, add a waypoint after this much XY travel.",
    )
    path_record_max_interval_s: float = Field(
        default_factory=lambda: float(
            os.environ.get("RFID_PATH_RECORD_MAX_INTERVAL_S", "2.0")
        ),
        gt=0.0,
        description="While path-recording, also add a waypoint at least this often while moving.",
    )


class RfidRecorderModule(Module):
    """Record RFID, camera, pose, walked path, and the accumulated point-cloud map.

    ``rfid_samples`` is the trigger stream: every RFID poll records tags + pose.
    Camera JPEGs are saved only after the dog moves ``min_image_move_m``.
    Odometry is also accumulated independently into the complete robot trajectory.
    """

    config: RfidRecorderConfig
    rfid_samples: In[RfidTagArray]
    color_image: In[Image]
    odom: In[PoseStamped]
    global_map: In[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._state_lock = threading.RLock()
        self._latest_image: Image | None = None
        self._latest_pose: PoseStamped | None = None
        self._latest_global_map: PointCloud2 | None = None
        self._global_map_updates = 0
        self._trajectory: list[dict[str, Any]] = []
        self._writer: _SessionWriter | None = None
        self._write_queue: queue.Queue[_CapturedSample | object] | None = None
        self._writer_thread: threading.Thread | None = None
        self._recording = False
        self._sequence = 0
        self._dropped_samples = 0
        self._last_result: dict[str, Any] | None = None
        self._collection_context: dict[str, Any] = {}
        self._prev_sample_pose: PoseStamped | None = None
        self._prev_sample_ts: float | None = None
        self._last_image_xy: tuple[float, float] | None = None
        self._images_skipped_stationary = 0
        # Teleop path-recording mode (independent of RFID dataset session).
        self._path_recording = False
        self._path_record_data: dict[str, Any] = {}
        self._path_record_last_xy: tuple[float, float] | None = None
        self._path_record_last_ts: float | None = None
        self._path_record_dirty = False
        self._path_record_last_flush_ts = 0.0

    @property
    def _output_root(self) -> Path:
        return Path(os.path.expandvars(self.config.output_dir)).expanduser().resolve()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_image)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.global_map.subscribe(self._on_global_map)))
        self.register_disposable(Disposable(self.rfid_samples.subscribe(self._on_rfid_sample)))
        if self.config.auto_start:
            self._begin_recording(self.config.session_name, {})

    def _on_image(self, image: Image) -> None:
        with self._state_lock:
            self._latest_image = image

    def _on_global_map(self, pointcloud: PointCloud2) -> None:
        """Keep the newest accumulated voxel map; copy it only when finalizing."""
        with self._state_lock:
            if self._recording:
                self._latest_global_map = pointcloud
                self._global_map_updates += 1

    def _on_odom(self, pose: PoseStamped) -> None:
        with self._state_lock:
            self._latest_pose = pose
            self._maybe_sample_path_recording(pose)
            if not self._recording:
                return
            pose_dict = _pose_to_dict(pose)
            if not self._trajectory:
                self._trajectory.append(pose_dict)
                return
            previous = self._trajectory[-1]
            p1 = previous["position"]
            p2 = pose_dict["position"]
            distance = math.dist((p1["x"], p1["y"], p1["z"]), (p2["x"], p2["y"], p2["z"]))
            elapsed = pose_dict["timestamp"] - previous["timestamp"]
            if (
                distance >= self.config.trajectory_min_distance_m
                or elapsed >= self.config.trajectory_max_interval_s
            ):
                self._trajectory.append(pose_dict)

    def _on_rfid_sample(self, sample: RfidTagArray) -> None:
        received_at = time.time()
        with self._state_lock:
            if not self._recording or self._write_queue is None:
                return
            self._sequence += 1
            image = self._latest_image
            pose = self._latest_pose
            collection = dict(self._collection_context) if self._collection_context else None
            if not collection:
                collection = read_collection_context() or None

            motion = self._motion_from_pose(pose, received_at)
            save_image = self._should_save_image(pose, motion)
            if not save_image and image is not None:
                self._images_skipped_stationary += 1
            motion["image_saved"] = bool(save_image and image is not None)

            captured = _CapturedSample(
                sequence=self._sequence,
                received_at=received_at,
                monotonic_ns=time.monotonic_ns(),
                rfid=_rfid_to_dict(sample),
                robot_pose=_pose_to_dict(pose, sample_ts=received_at) if pose is not None else None,
                image_data=(
                    image.to_opencv().copy() if save_image and image is not None else None
                ),
                image_timestamp=(
                    float(image.ts) if save_image and image is not None else None
                ),
                image_frame_id=image.frame_id if save_image and image is not None else "",
                image_format=(
                    image.format.value if save_image and image is not None else ""
                ),
                collection=collection,
                motion=motion,
            )
            if pose is not None:
                self._prev_sample_pose = pose
                self._prev_sample_ts = received_at
            # Queue while holding the state lock so stop_recording() cannot put
            # the worker sentinel between capture and enqueue.
            try:
                self._write_queue.put_nowait(captured)
            except queue.Full:
                self._dropped_samples += 1
                logger.warning("RFID dataset queue full; dropping sample %d", captured.sequence)

    def _motion_from_pose(
        self, pose: PoseStamped | None, received_at: float
    ) -> dict[str, Any]:
        """Estimate instantaneous speed vs the previous RFID sample pose."""
        speed_mps = 0.0
        distance_m = 0.0
        dt_s = 0.0
        prev = self._prev_sample_pose
        prev_ts = self._prev_sample_ts
        if pose is not None and prev is not None and prev_ts is not None:
            distance_m = math.hypot(
                float(pose.position.x) - float(prev.position.x),
                float(pose.position.y) - float(prev.position.y),
            )
            dt_s = max(received_at - prev_ts, 1e-3)
            speed_mps = distance_m / dt_s
        state = (
            "stationary"
            if speed_mps <= float(self.config.stationary_speed_mps)
            else "moving"
        )
        return {
            "state": state,
            "speed_mps": round(speed_mps, 4),
            "distance_from_previous_m": round(distance_m, 4),
            "dt_s": round(dt_s, 4),
            "stationary_speed_threshold_mps": float(self.config.stationary_speed_mps),
        }

    def _should_save_image(
        self, pose: PoseStamped | None, motion: dict[str, Any]
    ) -> bool:
        """Save a JPEG on the first sample, then only after enough XY travel."""
        min_move = float(self.config.min_image_move_m)
        if min_move <= 0.0:
            return True
        if pose is None:
            # No pose → keep previous behavior (allow image) but do not advance gate.
            return True
        xy = (float(pose.position.x), float(pose.position.y))
        if self._last_image_xy is None:
            self._last_image_xy = xy
            return True
        traveled = math.hypot(xy[0] - self._last_image_xy[0], xy[1] - self._last_image_xy[1])
        if traveled >= min_move:
            self._last_image_xy = xy
            return True
        return False

    def _writer_loop(self, writer: _SessionWriter, write_queue: queue.Queue[Any]) -> None:
        while True:
            item = write_queue.get()
            try:
                if item is _STOP:
                    return
                writer.write_sample(item)
            except Exception:  # noqa: BLE001 - keep later samples recordable
                logger.exception("Failed to write RFID dataset sample")
            finally:
                write_queue.task_done()

    def _begin_recording(
        self, session_name: str, metadata: dict[str, Any] | None
    ) -> dict[str, Any]:
        with self._state_lock:
            if self._recording:
                assert self._writer is not None
                return {
                    "ok": False,
                    "message": "A dataset session is already recording.",
                    "session_id": self._writer.session_id,
                    "session_dir": str(self._writer.session_dir),
                }
            writer = _SessionWriter(
                self._output_root,
                session_name,
                jpeg_quality=self.config.jpeg_quality,
                user_metadata=metadata,
            )
            write_queue: queue.Queue[Any] = queue.Queue(self.config.max_pending_samples)
            writer_thread = threading.Thread(
                target=self._writer_loop,
                args=(writer, write_queue),
                name="rfid-dataset-writer",
                daemon=True,
            )
            self._writer = writer
            self._write_queue = write_queue
            self._writer_thread = writer_thread
            self._trajectory = []
            self._latest_global_map = None
            self._global_map_updates = 0
            self._sequence = 0
            self._dropped_samples = 0
            self._prev_sample_pose = None
            self._prev_sample_ts = None
            self._last_image_xy = None
            self._images_skipped_stationary = 0
            self._recording = True
            writer_thread.start()
            logger.info(
                "RFID dataset recording started: %s (min_image_move_m=%.2f)",
                writer.session_dir,
                float(self.config.min_image_move_m),
            )
            return {
                "ok": True,
                "session_id": writer.session_id,
                "session_dir": str(writer.session_dir),
                "output_root": str(self._output_root),
            }

    @rpc
    def start_recording(
        self, session_name: str = "", metadata: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        """Start a new dataset session.

        Args:
            session_name: Human-readable run name; unsafe path characters are replaced.
            metadata: Optional experiment information such as room, tag layout, or operator.
        """
        return self._begin_recording(session_name, metadata)

    def _finish_recording(self, create_archive: bool | None = None) -> dict[str, Any]:
        with self._state_lock:
            if not self._recording:
                return self._last_result or {
                    "ok": False,
                    "message": "No dataset session is recording.",
                }
            self._recording = False
            writer = self._writer
            write_queue = self._write_queue
            writer_thread = self._writer_thread
            trajectory = list(self._trajectory)
            latest_global_map = self._latest_global_map
            global_map_updates = self._global_map_updates
            dropped = self._dropped_samples
        assert writer is not None and write_queue is not None and writer_thread is not None

        write_queue.join()
        write_queue.put(_STOP)
        writer_thread.join()
        pointcloud_map = self._capture_pointcloud_map(
            latest_global_map, update_count=global_map_updates
        )
        result = writer.finish(
            trajectory,
            pointcloud_map,
            dropped_samples=dropped,
            create_archive=(
                self.config.create_archive_on_stop
                if create_archive is None
                else create_archive
            ),
        )
        result["ok"] = True
        with self._state_lock:
            self._last_result = result
            self._writer = None
            self._write_queue = None
            self._writer_thread = None
        logger.info("RFID dataset ready: %s", result.get("archive_path") or result["session_dir"])
        return result

    @rpc
    def stop_recording(self, create_archive: bool | None = None) -> dict[str, Any]:
        """Finish the session and return the local directory and ZIP paths."""
        return self._finish_recording(create_archive)

    @rpc
    def get_recording_status(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "recording": self._recording,
                "session_id": self._writer.session_id if self._writer else None,
                "session_dir": str(self._writer.session_dir) if self._writer else None,
                "queued_samples": self._write_queue.qsize() if self._write_queue else 0,
                "captured_samples": self._sequence,
                "trajectory_poses": len(self._trajectory),
                "global_map_updates": self._global_map_updates,
                "global_map_available": self._latest_global_map is not None,
                "dropped_samples": self._dropped_samples,
                "images_skipped_stationary": self._images_skipped_stationary,
                "min_image_move_m": float(self.config.min_image_move_m),
                "collection": dict(self._collection_context),
                "last_result": self._last_result,
                "waypoint_path_file": str(self.config.waypoint_path_file),
                "path_recording": self._path_recording,
                "path_waypoint_count": len(self._path_record_data.get("waypoints") or []),
            }

    def _waypoint_file(self) -> Path:
        return Path(os.path.expandvars(self.config.waypoint_path_file)).expanduser().resolve()

    def _read_waypoint_file(self) -> dict[str, Any]:
        path = self._waypoint_file()
        if not path.is_file():
            return {
                "path_id": path.stem,
                "frame_id": "world",
                "description": "Captured with RfidRecorderModule.mark_waypoint",
                "waypoints": [],
            }
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, list):
            return {
                "path_id": path.stem,
                "frame_id": "world",
                "waypoints": data,
            }
        if not isinstance(data, dict):
            raise ValueError(f"Unsupported waypoint file format: {path}")
        data.setdefault("path_id", path.stem)
        data.setdefault("frame_id", "world")
        data.setdefault("waypoints", [])
        return data

    def _write_waypoint_file(self, data: dict[str, Any]) -> Path:
        path = self._waypoint_file()
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        tmp.replace(path)
        return path

    @staticmethod
    def _yaw_from_pose(pose: PoseStamped) -> float:
        euler = pose.orientation.to_euler()
        # Vector3 yaw is z in DimOS convention.
        return float(getattr(euler, "z", euler[2] if hasattr(euler, "__getitem__") else 0.0))

    def _pose_to_waypoint(self, pose: PoseStamped) -> dict[str, Any]:
        return {
            "x": round(float(pose.position.x), 3),
            "y": round(float(pose.position.y), 3),
            "z": round(float(pose.position.z), 3),
            "yaw": round(self._yaw_from_pose(pose), 4),
            "frame_id": pose.frame_id or "world",
        }

    def _maybe_sample_path_recording(self, pose: PoseStamped) -> None:
        """Append a waypoint while teleop path-recording is active (caller holds lock)."""
        if not self._path_recording:
            return
        now = float(pose.ts) if pose.ts is not None else time.time()
        xy = (float(pose.position.x), float(pose.position.y))
        waypoints = self._path_record_data.setdefault("waypoints", [])
        if not waypoints:
            waypoints.append(self._pose_to_waypoint(pose))
            self._path_record_last_xy = xy
            self._path_record_last_ts = now
            self._path_record_dirty = True
            self._flush_path_recording(force=False)
            return

        assert self._path_record_last_xy is not None
        traveled = math.hypot(
            xy[0] - self._path_record_last_xy[0],
            xy[1] - self._path_record_last_xy[1],
        )
        elapsed = now - (self._path_record_last_ts or now)
        min_dist = float(self.config.path_record_min_distance_m)
        max_dt = float(self.config.path_record_max_interval_s)
        # Only use time-based samples while actually moving (avoid spam when stopped).
        moving = traveled >= 0.02
        if traveled >= min_dist or (moving and elapsed >= max_dt):
            waypoints.append(self._pose_to_waypoint(pose))
            self._path_record_last_xy = xy
            self._path_record_last_ts = now
            self._path_record_dirty = True
            self._flush_path_recording(force=False)

    def _flush_path_recording(self, *, force: bool) -> Path | None:
        """Write in-memory path to disk (caller holds lock)."""
        if not self._path_record_data:
            return None
        if not force and not self._path_record_dirty:
            return self._waypoint_file()
        now = time.time()
        # Throttle disk writes unless forced (stop / explicit save).
        if not force and (now - self._path_record_last_flush_ts) < 1.0:
            return self._waypoint_file()
        path = self._write_waypoint_file(self._path_record_data)
        self._path_record_dirty = False
        self._path_record_last_flush_ts = now
        return path

    @rpc
    def start_path_recording(
        self,
        path_file: str = "",
        path_id: str = "",
        clear: bool = True,
    ) -> dict[str, Any]:
        """Start teleop path recording: auto-sample poses while a human drives.

        Drive with Keyboard Teleop. Waypoints are written to ``path_file`` (or
        ``RFID_WAYPOINT_CAPTURE_PATH``). Later set ``RFID_COLLECTION_PATH`` to that
        file and run ``go2-dataset-rounds`` to replay the same route.
        """
        with self._state_lock:
            if path_file.strip():
                self.config.waypoint_path_file = path_file.strip()
            path = self._waypoint_file()
            if clear or not path.is_file():
                data = {
                    "path_id": (path_id.strip() or path.stem),
                    "frame_id": "world",
                    "description": "Recorded by teleop path-recording mode",
                    "recorded_at": _utc_iso(time.time()),
                    "waypoints": [],
                }
            else:
                data = self._read_waypoint_file()
                if path_id.strip():
                    data["path_id"] = path_id.strip()
            self._path_record_data = data
            self._path_recording = True
            self._path_record_last_xy = None
            self._path_record_last_ts = None
            self._path_record_dirty = True
            # Seed with current pose if available.
            if self._latest_pose is not None:
                self._maybe_sample_path_recording(self._latest_pose)
            saved = self._flush_path_recording(force=True)
            logger.info(
                "Path recording started → %s (min_dist=%.2fm, max_dt=%.1fs)",
                saved,
                float(self.config.path_record_min_distance_m),
                float(self.config.path_record_max_interval_s),
            )
            return {
                "ok": True,
                "recording": True,
                "path_file": str(saved or path),
                "path_id": data.get("path_id"),
                "waypoint_count": len(data.get("waypoints") or []),
                "min_distance_m": float(self.config.path_record_min_distance_m),
                "max_interval_s": float(self.config.path_record_max_interval_s),
            }

    @rpc
    def stop_path_recording(self) -> dict[str, Any]:
        """Stop teleop path recording and flush the waypoint JSON to disk."""
        with self._state_lock:
            if not self._path_recording and not self._path_record_data:
                return {
                    "ok": True,
                    "recording": False,
                    "message": "Path recording was not active.",
                    "path_file": str(self._waypoint_file()),
                    "waypoint_count": 0,
                }
            # Always finalize with the latest pose so the path ends where the dog is.
            if self._latest_pose is not None:
                waypoints = self._path_record_data.setdefault("waypoints", [])
                last = waypoints[-1] if waypoints else None
                current = self._pose_to_waypoint(self._latest_pose)
                if last != current:
                    waypoints.append(current)
                    self._path_record_dirty = True
            self._path_recording = False
            path = self._flush_path_recording(force=True)
            count = len(self._path_record_data.get("waypoints") or [])
            path_id = self._path_record_data.get("path_id")
            logger.info("Path recording stopped → %s (%d waypoints)", path, count)
            return {
                "ok": True,
                "recording": False,
                "path_file": str(path or self._waypoint_file()),
                "path_id": path_id,
                "waypoint_count": count,
                "message": (
                    f"Saved {count} waypoints. Replay with "
                    f"RFID_COLLECTION_PATH={path} and go2-dataset-rounds."
                ),
            }

    @rpc
    def get_path_recording_status(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "ok": True,
                "recording": self._path_recording,
                "path_file": str(self._waypoint_file()),
                "path_id": self._path_record_data.get("path_id"),
                "waypoint_count": len(self._path_record_data.get("waypoints") or []),
                "min_distance_m": float(self.config.path_record_min_distance_m),
                "max_interval_s": float(self.config.path_record_max_interval_s),
            }

    @rpc
    def mark_waypoint(self, path_file: str = "") -> dict[str, Any]:
        """Append the current robot pose as a waypoint to the capture JSON path.

        Drive the dog to a spot (teleop), then call this RPC. Set
        ``RFID_WAYPOINT_CAPTURE_PATH`` or pass ``path_file`` to choose the file.
        Later point ``RFID_COLLECTION_PATH`` at that file for ``go2-dataset-rounds``.
        """
        with self._state_lock:
            pose = self._latest_pose
            if path_file.strip():
                self.config.waypoint_path_file = path_file.strip()
        if pose is None:
            return {"ok": False, "message": "No odometry pose yet — wait for localization."}
        try:
            data = self._read_waypoint_file()
            waypoint = self._pose_to_waypoint(pose)
            data.setdefault("waypoints", []).append(waypoint)
            # Keep in-memory path-recording buffer in sync if active on same file.
            with self._state_lock:
                if self._path_recording:
                    self._path_record_data = data
                    self._path_record_last_xy = (waypoint["x"], waypoint["y"])
                    self._path_record_last_ts = time.time()
            path = self._write_waypoint_file(data)
            logger.info("Marked waypoint #%d → %s %s", len(data["waypoints"]), path, waypoint)
            return {
                "ok": True,
                "path_file": str(path),
                "waypoint": waypoint,
                "waypoint_count": len(data["waypoints"]),
            }
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "message": str(exc)}

    @rpc
    def clear_waypoints(self, path_file: str = "") -> dict[str, Any]:
        """Clear all waypoints in the capture path file (keeps path_id/metadata)."""
        with self._state_lock:
            if path_file.strip():
                self.config.waypoint_path_file = path_file.strip()
        try:
            data = self._read_waypoint_file()
            data["waypoints"] = []
            path = self._write_waypoint_file(data)
            return {"ok": True, "path_file": str(path), "waypoint_count": 0}
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "message": str(exc)}

    @rpc
    def get_waypoints(self, path_file: str = "") -> dict[str, Any]:
        """Return the current capture path JSON contents."""
        with self._state_lock:
            if path_file.strip():
                self.config.waypoint_path_file = path_file.strip()
        try:
            data = self._read_waypoint_file()
            return {
                "ok": True,
                "path_file": str(self._waypoint_file()),
                "path_id": data.get("path_id"),
                "waypoint_count": len(data.get("waypoints") or []),
                "waypoints": data.get("waypoints") or [],
            }
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "message": str(exc)}

    @rpc
    def set_collection_context(self, context: dict[str, Any] | None = None) -> dict[str, Any]:
        """Stamp each subsequent observation with round / power metadata.

        Args:
            context: Mapping merged into ``observations.jsonl`` as ``collection``.
                Pass ``None`` or ``{}`` to clear.
        """
        with self._state_lock:
            self._collection_context = dict(context or {})
            try:
                from dimos_rfid.collection_context import write_collection_context

                write_collection_context(self._collection_context or None)
            except Exception:  # noqa: BLE001 - file sync is best-effort
                pass
            if self._writer is not None and self._collection_context:
                self._writer.user_metadata.setdefault("collection_rounds", [])
                # Keep latest context visible in session metadata.
                self._writer.user_metadata["latest_collection"] = dict(self._collection_context)
            return {"ok": True, "collection": dict(self._collection_context)}

    @rpc
    def update_user_metadata(self, metadata: dict[str, Any] | None = None) -> dict[str, Any]:
        """Merge keys into the active session's ``user_metadata`` (no-op if idle)."""
        with self._state_lock:
            if self._writer is None:
                return {"ok": False, "message": "No dataset session is recording."}
            self._writer.user_metadata.update(dict(metadata or {}))
            return {"ok": True, "user_metadata": dict(self._writer.user_metadata)}

    @staticmethod
    def _capture_pointcloud_map(
        pointcloud: PointCloud2 | None,
        *,
        update_count: int = 0,
    ) -> _CapturedPointCloudMap | None:
        if pointcloud is None:
            return None
        points, colors = pointcloud.as_numpy()
        return _CapturedPointCloudMap(
            points=np.asarray(points).copy(),
            colors=np.asarray(colors).copy() if colors is not None else None,
            timestamp=float(pointcloud.ts) if pointcloud.ts is not None else None,
            frame_id=pointcloud.frame_id,
            update_count=update_count,
        )

    @skill
    def begin_rfid_dataset(self, session_name: str = "") -> str:
        """Begin recording synchronized RFID, camera, robot-pose, and trajectory data."""
        result = self._begin_recording(session_name, {})
        if result["ok"]:
            return f"RFID dataset recording started: {result['session_dir']}"
        return result["message"]

    @skill
    def finish_rfid_dataset(self) -> str:
        """Stop dataset recording, create its ZIP archive, and report its laptop path."""
        result = self._finish_recording()
        if not result.get("ok"):
            return result["message"]
        return f"RFID dataset saved to {result.get('archive_path') or result['session_dir']}"

    @skill
    def mark_rfid_waypoint(self, path_file: str = "") -> str:
        """Save the dog's current pose as the next waypoint in the capture path JSON."""
        result = self.mark_waypoint(path_file)
        if not result.get("ok"):
            return result.get("message", "Failed to mark waypoint")
        wp = result["waypoint"]
        return (
            f"Waypoint #{result['waypoint_count']} saved to {result['path_file']}: "
            f"({wp['x']}, {wp['y']}, yaw={wp['yaw']})"
        )

    @skill
    def start_rfid_path_recording(self, path_file: str = "", path_id: str = "") -> str:
        """Start recording the teleop route as waypoints for later replay."""
        result = self.start_path_recording(path_file=path_file, path_id=path_id, clear=True)
        if not result.get("ok"):
            return result.get("message", "Failed to start path recording")
        return (
            f"Path recording ON → {result['path_file']} "
            f"(sample every {result['min_distance_m']} m). Drive the dog, then stop recording."
        )

    @skill
    def stop_rfid_path_recording(self) -> str:
        """Stop teleop path recording and save the waypoint JSON."""
        result = self.stop_path_recording()
        if not result.get("ok"):
            return result.get("message", "Failed to stop path recording")
        return result.get("message") or (
            f"Saved {result.get('waypoint_count', 0)} waypoints to {result.get('path_file')}"
        )

    @rpc
    def stop(self) -> None:
        if self._path_recording:
            try:
                self.stop_path_recording()
            except Exception:  # noqa: BLE001 - module shutdown must continue
                logger.exception("Failed to finalize path recording during shutdown")
        if self._recording:
            try:
                self._finish_recording()
            except Exception:  # noqa: BLE001 - module shutdown must continue
                logger.exception("Failed to finalize RFID dataset during shutdown")
        super().stop()


__all__ = [
    "RfidRecorderConfig",
    "RfidRecorderModule",
]
