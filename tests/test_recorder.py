from __future__ import annotations

import json
from pathlib import Path
import zipfile

import numpy as np

from dimos.core.coordination.blueprints import BlueprintAtom
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos_rfid.recorder import (
    _CapturedSample,
    _CapturedPointCloudMap,
    _SessionWriter,
    RfidRecorderModule,
)
from dimos_rfid.msgs import RfidTag, RfidTagArray


def _pose(timestamp: float, x: float, y: float) -> dict:
    return {
        "timestamp": timestamp,
        "timestamp_iso": "2026-01-01T00:00:00+00:00",
        "frame_id": "world",
        "position": {"x": x, "y": y, "z": 0.3},
        "orientation_xyzw": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }


def test_session_writer_exports_synchronized_dataset(tmp_path: Path) -> None:
    writer = _SessionWriter(
        tmp_path,
        "warehouse run/1",
        jpeg_quality=90,
        user_metadata={"room": "warehouse"},
    )
    writer.write_sample(
        _CapturedSample(
            sequence=1,
            received_at=1_800_000_000.0,
            monotonic_ns=123,
            rfid={
                "timestamp": 1_800_000_000.0,
                "timestamp_iso": "2027-01-15T08:00:00+00:00",
                "frame_id": "rfid_antenna",
                "active_count": 1,
                "total_count": 1,
                "connection_status": "Connected",
                "tags": [
                    {
                        "epc": "E28001",
                        "rssi_dbm": -61.0,
                        "antenna": 1,
                        "frequency_khz": 915250,
                        "read_count": 3,
                        "in_range": True,
                        "last_seen": 1_800_000_000.0,
                        "name": "test-tag",
                    }
                ],
            },
            robot_pose=_pose(1_800_000_000.0, 0.0, 0.0),
            image_data=np.full((24, 32, 3), 127, dtype=np.uint8),
            image_timestamp=1_800_000_000.0,
            image_frame_id="camera_optical",
            image_format="BGR",
        )
    )

    result = writer.finish(
        [_pose(1.0, 0.0, 0.0), _pose(2.0, 3.0, 4.0)],
        _CapturedPointCloudMap(
            points=np.asarray([[0.0, 1.0, 2.0], [3.0, 4.0, 5.0]], dtype=np.float32),
            colors=None,
            timestamp=2.0,
            frame_id="world",
        ),
        dropped_samples=0,
        create_archive=True,
    )

    session_dir = Path(result["session_dir"])
    archive = Path(result["archive_path"])
    assert session_dir.name == "warehouse_run_1"
    assert (session_dir / "images/00000001.jpg").is_file()
    assert (session_dir / "trajectory.csv").is_file()
    assert (session_dir / "trajectory.png").is_file()
    assert (session_dir / "pointcloud_map.npz").is_file()
    assert (session_dir / "pointcloud_map.ply").is_file()
    assert archive.is_file()

    observation = json.loads((session_dir / "observations.jsonl").read_text().strip())
    assert observation["rfid"]["tags"][0]["rssi_dbm"] == -61.0
    assert observation["robot_pose"]["position"]["x"] == 0.0
    assert observation["image"]["path"] == "images/00000001.jpg"

    metadata = json.loads((session_dir / "metadata.json").read_text())
    assert metadata["observation_count"] == 1
    assert metadata["image_count"] == 1
    assert metadata["path_length_m"] == 5.0
    assert metadata["pointcloud_map"]["available"] is True
    assert metadata["pointcloud_map"]["point_count"] == 2
    assert metadata["user_metadata"] == {"room": "warehouse"}
    with zipfile.ZipFile(archive) as zipped:
        assert f"{session_dir.name}/metadata.json" in zipped.namelist()
        assert f"{session_dir.name}/pointcloud_map.ply" in zipped.namelist()


def test_recorder_declares_matching_dimos_inputs() -> None:
    atom = BlueprintAtom.create(RfidRecorderModule, {})
    streams = {(stream.name, stream.direction) for stream in atom.streams}
    assert streams == {
        ("rfid_samples", "in"),
        ("color_image", "in"),
        ("odom", "in"),
        ("global_map", "in"),
    }


def test_rfid_message_preserves_reader_metadata() -> None:
    sample = RfidTagArray.from_api_payload(
        {
            "reader_host": "192.168.123.2",
            "device_id": "reader-1",
            "reader_started": True,
            "stale_seconds": 5,
            "updated_at": "2027-01-15T08:00:00+00:00",
            "scanner": {"connected": True},
            "tags": [
                {
                    "epc": "ABC",
                    "rssi_dbm": -52,
                    "phase": "20",
                    "first_seen": 99.0,
                    "device_id": "reader-1",
                    "in_range": True,
                }
            ],
        }
    )
    assert sample.reader_host == "192.168.123.2"
    assert sample.reader_device_id == "reader-1"
    assert sample.scanner_status == {"connected": True}
    assert sample.tags[0].phase == "20"
    assert sample.tags[0].first_seen == 99.0


def test_module_synchronizes_latest_image_pose_and_rfid(tmp_path: Path) -> None:
    module = RfidRecorderModule(
        output_dir=str(tmp_path),
        create_archive_on_stop=False,
        trajectory_min_distance_m=0.0,
    )
    try:
        assert module.start_recording("sync-test")["ok"]
        module._on_image(
            Image(
                data=np.zeros((10, 12, 3), dtype=np.uint8),
                format=ImageFormat.BGR,
                frame_id="camera_optical",
                ts=100.0,
            )
        )
        module._on_odom(
            PoseStamped(
                ts=100.1,
                frame_id="world",
                position=[1.0, 2.0, 0.3],
                orientation=[0.0, 0.0, 0.0, 1.0],
            )
        )
        module._on_global_map(
            PointCloud2.from_numpy(
                np.asarray([[1.0, 2.0, 0.1], [2.0, 3.0, 0.2]], dtype=np.float32),
                frame_id="world",
                timestamp=100.15,
            )
        )
        module._on_rfid_sample(
            RfidTagArray(
                tags=[RfidTag(epc="ABC", rssi_dbm=-55.0, phase="21", in_range=True)],
                active_count=1,
                total_count=1,
                ts=100.2,
            )
        )
        result = module.stop_recording(False)
        observation = json.loads(
            (Path(result["session_dir"]) / "observations.jsonl").read_text().strip()
        )
        assert observation["image"]["timestamp"] == 100.0
        assert observation["robot_pose"]["timestamp"] == 100.1
        assert observation["rfid"]["timestamp"] == 100.2
        assert observation["rfid"]["tags"][0]["rssi_dbm"] == -55.0
        assert observation["rfid"]["tags"][0]["phase"] == "21"
        assert result["trajectory_pose_count"] == 1
        assert result["pointcloud_map"]["point_count"] == 2
        assert result["pointcloud_map"]["update_count"] == 1
        pointcloud = np.load(Path(result["session_dir"]) / "pointcloud_map.npz")
        assert pointcloud["points"].shape == (2, 3)
        assert pointcloud["frame_id"].item() == "world"
    finally:
        module.stop()
