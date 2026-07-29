# RFID Dataset Collection — Feature Guide

Guide to the **dataset recording**, **teleop path capture**, and **power-ladder
replay** features added for synchronized Go2 + Vulcan RFID experiments.

No MCP agent is required. Everything is driven by DimOS blueprints, a Rerun UI,
Keyboard Teleop, and a small waypoint CLI.

---

## What you get

| Capability | Entry point |
|------------|-------------|
| Manual drive + record RFID/camera/pose | `uv run python -m dimos_rfid go2-dataset` |
| Auto-sample a teleop route to JSON | `waypoint_cli record-start` / `record-stop` |
| Replay a saved path (+ optional TX power ladder) | `uv run python -m dimos_rfid go2-dataset-rounds` |
| Live Collection status in Rerun | panel **Collection** (rounds mode) |
| Persistent config | repo-root `.env` (gitignored) |

**Data lands on the laptop** (not the dog):

`~/Downloads/dimos_rfid_datasets/<session>/`

---

## RFID read frequency

Default poll rate is **1 Hz** (one HTTP tag snapshot per second).

- Set by `RFID_POLL_HZ` (blueprint passes it into `RfidModule.poll_hz`; default `"1"`).
- Each poll: DimOS `GET`s `{RFID_API_BASE}/…tags` on the dog and publishes `/rfid/samples`.
- The recorder treats **every poll** as one observation while a session is active — about **1 sample/second**, whether the dog is walking or standing still.
- Motion does **not** gate RFID reads. It only:
  - labels each observation (`motion.state` = `moving` | `stationary`)
  - optionally skips saving a new JPEG until the dog has moved ≥ `RFID_DATASET_MIN_IMAGE_MOVE_M`

To poll faster or slower:

```bash
# .env or shell
RFID_POLL_HZ=2    # 2 samples/sec
RFID_POLL_HZ=0.5  # one sample every 2 seconds
```

Restart the DimOS stack after changing `RFID_POLL_HZ`.

---

## Prerequisites

### On the Go2

```bash
python3 rfid_scanner_server.py
curl http://<go2-wifi-ip>:8765/api/v1/power
```

Power can also be set from the dog web UI: `http://<go2-wifi-ip>:8765/settings`

### On the laptop

```bash
cp .env.example .env   # once
# edit ROBOT_IP and RFID_API_BASE
uv sync --extra unitree
```

`python -m dimos_rfid …` and `run_semantic_rfid.py` **load `.env` automatically**.
Shell `export`s always override `.env`.

---

## Configuration (`.env`)

Typical values:

```bash
ROBOT_IP=10.42.x.x
RFID_API_BASE=http://10.42.x.x:8765/api/v1

RFID_POWER_LADDER=30,25,20,15
RFID_COLLECTION_PATH=dimos_rfid/paths/captured_path.json
RFID_DATASET_DIR=~/Downloads/dimos_rfid_datasets

# Fewer JPEGs while standing still (meters of XY travel between images)
RFID_DATASET_MIN_IMAGE_MOVE_M=0.25
# Label observations moving vs stationary
RFID_DATASET_STATIONARY_SPEED_MPS=0.05

# Teleop path capture defaults
RFID_WAYPOINT_CAPTURE_PATH=dimos_rfid/paths/captured_path.json
RFID_PATH_RECORD_MIN_DISTANCE_M=0.35
RFID_PATH_RECORD_MAX_INTERVAL_S=2.0
```

| Variable | Role |
|----------|------|
| `ROBOT_IP` | Go2 WebRTC |
| `RFID_API_BASE` | Scanner HTTP API on the dog |
| `RFID_POWER_LADDER` | dBm list for rounds (0–31.5, 0.5 steps) |
| `RFID_COLLECTION_PATH` | Waypoint JSON to **replay** |
| `RFID_WAYPOINT_CAPTURE_PATH` | Default file for **record/mark** |
| `RFID_DATASET_DIR` | Session output root |
| `RFID_DATASET_MIN_IMAGE_MOVE_M` | Min travel before next JPEG |
| `RFID_DATASET_STATIONARY_SPEED_MPS` | Speed ≤ this ⇒ `motion.state=stationary` |
| `RFID_PATH_RECORD_MIN_DISTANCE_M` | Spacing while path-recording |
| `RFID_PATH_RECORD_MAX_INTERVAL_S` | Also sample this often while moving |
| `RFID_POLL_HZ` | RFID HTTP poll / observation rate in Hz (**default 1** = once per second; applies moving and stationary) |
| `RFID_COLLECTION_AUTO_START` | `1` = start rounds when stack comes up |

---

## Workflow A — Manual teleop recording

Best for exploratory walks and first datasets.

```bash
uv run python -m dimos_rfid go2-dataset
```

1. Wait for Rerun (Camera + 3D map) and `RFID dataset recording started`.
2. Drive with **Keyboard Teleop** in the DimOS viewer  
   (`W/S` forward/back, `A/D` strafe, `Q/E` turn, `Shift` faster, **STOP**).
3. Optional: open [http://localhost:7779/command-center](http://localhost:7779/command-center) for the 2D map / click goals / Explore.
4. **Ctrl+C** when done → session finalized + ZIP.

### What is recorded

At the default **1 Hz** RFID poll (`RFID_POLL_HZ=1`), every second writes one line
to `observations.jsonl` — **on the move and while stationary**:

- RFID tags (EPC, RSSI, phase, …)
- robot pose
- `motion`: `{ state, speed_mps, image_saved, … }`
- camera JPEG **only** if the dog moved ≥ `RFID_DATASET_MIN_IMAGE_MOVE_M` since the last image

Also at session end: `trajectory.*`, `pointcloud_map.*`, `metadata.json`, `<session>.zip`.

RFID sampling is continuous for the whole recording session; only images are
motion-gated so standing still does not flood `images/`.

---

## Workflow B — Record a path, then replay it

### 1. Capture the route (human drives)

Keep `go2-dataset` running in terminal 1. In terminal 2:

```bash
uv run python -m dimos_rfid.waypoint_cli record-start \
  --path dimos_rfid/paths/captured_path.json --path-id my_route

# drive the dog with Keyboard Teleop …

uv run python -m dimos_rfid.waypoint_cli record-status
uv run python -m dimos_rfid.waypoint_cli record-stop
```

Waypoints are auto-sampled about every **0.35 m** into a JSON file like:

```json
{
  "path_id": "my_route",
  "frame_id": "world",
  "waypoints": [
    {"x": -1.22, "y": 0.17, "z": 0.29, "yaw": -1.96, "frame_id": "world"}
  ]
}
```

**Discrete marks** (one pose per command) instead of continuous recording:

```bash
uv run python -m dimos_rfid.waypoint_cli mark --path dimos_rfid/paths/lab_loop.json
uv run python -m dimos_rfid.waypoint_cli list  --path dimos_rfid/paths/lab_loop.json
uv run python -m dimos_rfid.waypoint_cli clear --path dimos_rfid/paths/lab_loop.json
```

### 2. Replay the route (dog drives itself)

Stop the manual stack, then:

```bash
# .env
RFID_COLLECTION_PATH=dimos_rfid/paths/captured_path.json
# optional: single power instead of full ladder
# RFID_POWER_LADDER=30

uv run python -m dimos_rfid go2-dataset-rounds
```

For each power in `RFID_POWER_LADDER`:

1. Set reader `read_power` over HTTP  
2. Settle briefly  
3. Drive all waypoints via DimOS `goal_request` (A* nav)  
4. Stamp observations with `collection: { round_index, read_power_dbm, path_id, … }`

Watch the Rerun **Collection** panel for state / power / waypoint progress.

**Important:** waypoints are in the SLAM `world` frame from recording time. Replay in the **same mapped session** when possible; a cold restart with a different origin will misplace the path.

---

## Rerun layout

| Panel | Content |
|-------|---------|
| Camera | Go2 color image |
| 3D | SLAM `global_map` (+ robot TF) |
| RFID | Live tag list from `RfidModule` |
| Collection | Round / power / live log (`go2-dataset-rounds`) |

Keyboard Teleop is a DimOS viewer overlay (not part of RFID code).

If the 3D map is blank, check that **rerun-sdk** and **dimos-viewer** are on the same 0.32.x line (this repo pins that in `pyproject.toml`). Long runs may log Rerun memory-limit messages as the map history grows — expected, not a crash.

---

## Output layout

```text
~/Downloads/dimos_rfid_datasets/<session>/
├── metadata.json
├── observations.jsonl      # one RFID-triggered sample per line
├── images/                 # JPEGs (motion-gated)
├── trajectory.json|.csv|.png
├── pointcloud_map.npz|.ply
└── <session>.zip           # after stop / Ctrl+C
```

Example observation fields:

```json
{
  "rfid": { "tags": [ { "epc": "…", "rssi_dbm": -45 } ] },
  "robot_pose": { "position": { "x": 1.2, "y": 0.4, "z": 0.3 } },
  "image": { "path": "images/00000012.jpg" },
  "motion": { "state": "moving", "speed_mps": 0.35, "image_saved": true },
  "collection": {
    "round_index": 1,
    "read_power_dbm": 25.0,
    "path_id": "captured_path"
  }
}
```

(`collection` appears during power-ladder rounds.)

---

## Operator commands (CLI)

Requires a **running** dataset stack in another terminal.

```bash
# Path recording
uv run python -m dimos_rfid.waypoint_cli record-start --path … --path-id …
uv run python -m dimos_rfid.waypoint_cli record-status
uv run python -m dimos_rfid.waypoint_cli record-stop

# Single waypoints
uv run python -m dimos_rfid.waypoint_cli mark  --path …
uv run python -m dimos_rfid.waypoint_cli list  --path …
uv run python -m dimos_rfid.waypoint_cli clear --path …
```

RPC timeouts usually mean either DimOS is not running, or an old CLI used the wrong RPC name. Current CLI targets `RfidRecorderModule/…` (PascalCase).

---

## Modules (architecture)

| Module | Role |
|--------|------|
| `RfidModule` | Poll dog HTTP API; publish `/rfid/tags` + `/rfid/samples`; power RPCs |
| `RfidRecorderModule` | Sync RFID + image + pose → disk; motion/image gating; path record/mark |
| `RfidCollectionRoundsModule` | Power ladder + drive waypoint path; Rerun Collection panel |
| DimOS `unitree_go2` | SLAM, A*, teleop, command center (`WebsocketVisModule` on `:7779`) |

HTTP power helpers live in `dimos_rfid/rfid_power.py` (`GET/PUT …/power`).

Example path files: `dimos_rfid/paths/example_loop.json`, `captured_path.json`.

---

## Command center (DimOS, not RFID)

[http://localhost:7779/command-center](http://localhost:7779/command-center) is served by DimOS **`WebsocketVisModule`**.

Useful actions: click-to-goal, teleop, **Explore the env**. Explore uses DimOS’s built-in **`WavefrontFrontierExplorer`** (frontier / wavefront on the costmap) + **`ReplanningAStarPlanner`** — not an LLM and not RFID-aware.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `waypoint_cli` RPC timeout | Keep `go2-dataset` / `go2-dataset-rounds` running; wait until recorder has started |
| Blank 3D map | Align `rerun-sdk` / `dimos-viewer` (0.32.x); walk so SLAM builds map |
| Too many images | Raise `RFID_DATASET_MIN_IMAGE_MOVE_M` (e.g. `0.5`) |
| Power set fails | `curl $RFID_API_BASE/power` on dog; check Settings UI |
| Replay path in wrong place | Same map/world frame as when recorded |
| Rerun “memory limit” | Viewer dropping old history; map logging is heavy — normal on long runs |

---

## Tests

```bash
uv run pytest tests/test_recorder.py tests/test_rfid_power.py tests/test_collection_rounds.py -v
```

---

## Related docs

- [README.md](README.md) — repo overview / Go2 + RFID setup  
- [SEMANTIC_LOCALIZER.md](SEMANTIC_LOCALIZER.md) — particle-filter tag localization  
- Dog API — `rfid scanner python/RFID_API.md` (on machine docs may live under `~/RFID_API.md`)
