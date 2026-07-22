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
from dimos.utils.logging_config import setup_logger

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

    def finish(
        self,
        trajectory: list[dict[str, Any]],
        *,
        dropped_samples: int,
        create_archive: bool,
    ) -> dict[str, Any]:
        self._observations_file.flush()
        self._observations_file.close()
        completed_at = time.time()
        path_length = self._write_trajectory(trajectory)
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
            "dropped_sample_count": dropped_samples,
            "files": {
                "observations": "observations.jsonl",
                "images": "images/",
                "trajectory_json": "trajectory.json",
                "trajectory_csv": "trajectory.csv",
                "trajectory_preview": "trajectory.png",
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


class RfidRecorderModule(Module):
    """Record synchronized RFID samples, camera frames, pose, and walked path.

    ``rfid_samples`` is the trigger stream: every RFID sample captures the latest
    camera frame and robot pose. Odometry is also accumulated independently into
    the complete robot trajectory.
    """

    config: RfidRecorderConfig
    rfid_samples: In[RfidTagArray]
    color_image: In[Image]
    odom: In[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._state_lock = threading.RLock()
        self._latest_image: Image | None = None
        self._latest_pose: PoseStamped | None = None
        self._trajectory: list[dict[str, Any]] = []
        self._writer: _SessionWriter | None = None
        self._write_queue: queue.Queue[_CapturedSample | object] | None = None
        self._writer_thread: threading.Thread | None = None
        self._recording = False
        self._sequence = 0
        self._dropped_samples = 0
        self._last_result: dict[str, Any] | None = None

    @property
    def _output_root(self) -> Path:
        return Path(os.path.expandvars(self.config.output_dir)).expanduser().resolve()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_image)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.rfid_samples.subscribe(self._on_rfid_sample)))
        if self.config.auto_start:
            self._begin_recording(self.config.session_name, {})

    def _on_image(self, image: Image) -> None:
        with self._state_lock:
            self._latest_image = image

    def _on_odom(self, pose: PoseStamped) -> None:
        with self._state_lock:
            self._latest_pose = pose
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
            captured = _CapturedSample(
                sequence=self._sequence,
                received_at=received_at,
                monotonic_ns=time.monotonic_ns(),
                rfid=_rfid_to_dict(sample),
                robot_pose=_pose_to_dict(pose, sample_ts=received_at) if pose is not None else None,
                image_data=image.to_opencv().copy() if image is not None else None,
                image_timestamp=float(image.ts) if image is not None else None,
                image_frame_id=image.frame_id if image is not None else "",
                image_format=image.format.value if image is not None else "",
            )
            # Queue while holding the state lock so stop_recording() cannot put
            # the worker sentinel between capture and enqueue.
            try:
                self._write_queue.put_nowait(captured)
            except queue.Full:
                self._dropped_samples += 1
                logger.warning("RFID dataset queue full; dropping sample %d", captured.sequence)

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
            self._sequence = 0
            self._dropped_samples = 0
            self._recording = True
            writer_thread.start()
            logger.info("RFID dataset recording started: %s", writer.session_dir)
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
            dropped = self._dropped_samples
        assert writer is not None and write_queue is not None and writer_thread is not None

        write_queue.join()
        write_queue.put(_STOP)
        writer_thread.join()
        result = writer.finish(
            trajectory,
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
                "dropped_samples": self._dropped_samples,
                "last_result": self._last_result,
            }

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

    @rpc
    def stop(self) -> None:
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
