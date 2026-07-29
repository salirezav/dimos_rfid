# Copyright 2026. RFID DimOS integration.

from __future__ import annotations

import json
from pathlib import Path

import pytest

from dimos_rfid.collection_context import (
    read_collection_context,
    write_collection_context,
)
from dimos_rfid.collection_rounds import load_waypoints
from dimos_rfid.recorder import _CapturedSample, _SessionWriter


def test_load_waypoints_from_example_path() -> None:
    path = (
        Path(__file__).resolve().parents[1]
        / "dimos_rfid"
        / "paths"
        / "example_loop.json"
    )
    path_id, waypoints = load_waypoints(path)
    assert path_id == "example_loop"
    assert len(waypoints) == 5
    assert waypoints[0].x == 0.0
    assert waypoints[1].yaw == pytest.approx(1.5708)


def test_load_waypoints_list_form(tmp_path: Path) -> None:
    path = tmp_path / "simple.json"
    path.write_text(
        json.dumps([{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0, "yaw": 0.5}]),
        encoding="utf-8",
    )
    path_id, waypoints = load_waypoints(path)
    assert path_id == "simple"
    assert len(waypoints) == 2
    assert waypoints[1].yaw == 0.5


def test_collection_context_roundtrip(tmp_path: Path, monkeypatch) -> None:
    ctx_file = tmp_path / "ctx.json"
    monkeypatch.setenv("RFID_COLLECTION_CONTEXT_FILE", str(ctx_file))
    write_collection_context(
        {
            "round_index": 1,
            "read_power_dbm": 20.0,
            "path_id": "example_loop",
        }
    )
    loaded = read_collection_context()
    assert loaded["round_index"] == 1
    assert loaded["read_power_dbm"] == 20.0
    write_collection_context(None)
    assert read_collection_context() == {}


def test_observation_includes_collection_metadata(tmp_path: Path) -> None:
    writer = _SessionWriter(tmp_path, "power-ladder", jpeg_quality=90)
    writer.write_sample(
        _CapturedSample(
            sequence=1,
            received_at=1.0,
            monotonic_ns=1,
            rfid={
                "timestamp": 1.0,
                "tags": [{"epc": "E280", "rssi_dbm": -50.0}],
            },
            robot_pose=None,
            image_data=None,
            image_timestamp=None,
            image_frame_id="",
            image_format="",
            collection={
                "round_index": 2,
                "round_count": 4,
                "read_power_dbm": 20.0,
                "path_id": "example_loop",
                "waypoint_count": 5,
            },
            motion={"state": "stationary", "speed_mps": 0.0, "image_saved": False},
        )
    )
    result = writer.finish([], None, dropped_samples=0, create_archive=False)
    observation = json.loads(
        (Path(result["session_dir"]) / "observations.jsonl").read_text().strip()
    )
    assert observation["collection"]["read_power_dbm"] == 20.0
    assert observation["collection"]["round_index"] == 2
    assert observation["motion"]["state"] == "stationary"
    assert observation["image"] is None


def test_path_recording_samples_by_distance(tmp_path: Path) -> None:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    from dimos_rfid.recorder import RfidRecorderModule

    path_file = tmp_path / "lab_loop.json"
    module = RfidRecorderModule(
        waypoint_path_file=str(path_file),
        path_record_min_distance_m=0.5,
        path_record_max_interval_s=10.0,
        auto_start=False,
    )
    try:
        started = module.start_path_recording(path_file=str(path_file), path_id="lab_loop")
        assert started["ok"]
        # Inject poses along X without going through DimOS subscriptions.
        for i, x in enumerate([0.0, 0.2, 0.55, 1.1, 1.1]):
            pose = PoseStamped(
                position=Vector3(x, 0.0, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
                ts=100.0 + i,
            )
            with module._state_lock:
                module._latest_pose = pose
                module._maybe_sample_path_recording(pose)
        stopped = module.stop_path_recording()
        assert stopped["ok"]
        assert stopped["waypoint_count"] >= 3
        data = json.loads(path_file.read_text(encoding="utf-8"))
        assert data["path_id"] == "lab_loop"
        xs = [wp["x"] for wp in data["waypoints"]]
        assert xs[0] == 0.0
        assert max(xs) >= 1.0
    finally:
        module.stop()
