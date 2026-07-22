# Dimos + Unitree Go2 + Vulcan RFID

This repository combines three pieces into one robotics stack:

1. **[DimOS](https://github.com/dimensionalOS/dimos)** — robot operating system for the Unitree Go2 (SLAM, navigation, perception, agents, visualization).
2. **`rfid scanner python/`** — Python service for the **Vulcan RFID Titanium** reader on the dog (HTTP API + web dashboard).
3. **`dimos_rfid/`** — DimOS module that bridges RFID tag reads into the DimOS runtime (LCM streams, Rerun overlay, agent skills).

The intended deployment is a **two-machine setup**: the RFID HTTP server runs **on the Go2**, and DimOS runs on a **Linux laptop** (or onboard computer) that connects to the dog over Wi‑Fi.

---

## Repository layout

```
Dimos/
├── README.md                    ← you are here
├── pyproject.toml               ← project metadata and dependencies (uv)
├── uv.lock                      ← locked dependency versions
├── rfid scanner python/         ← runs on the robot (reader hardware access)
│   ├── rfid_scanner_server.py   ← Flask HTTP API + web UI (port 8765)
│   ├── rfid_service.py          ← Python RfidScanner library
│   ├── vulcan_rfid_reader.py      ← low-level Keonn AdvanNet client
│   ├── RFID_SCANNER.md            ← hardware & server setup
│   └── RFID_API.md                ← HTTP API reference
└── dimos_rfid/                    ← DimOS integration (runs on the laptop)
    ├── rfid_module.py             ← RfidModule (DimOS Module)
    ├── recorder.py                ← synchronized offline data recorder
    ├── msgs.py                    ← RfidTag / RfidTagArray message types
    ├── go2_blueprints.py          ← unitree-go2-rfid blueprint
    ├── demo_blueprint.py          ← rfid-demo blueprint (RFID + viewer only)
    ├── integrate_with_dimos.sh    ← registers blueprints with `dimos run`
    └── README.md                  ← module design & integration (detailed)
```

---

## System architecture

```
┌────────────────────────── Unitree Go2 (onboard) ──────────────────────────┐
│                                                                           │
│   Vulcan RFID reader ──Ethernet──► 192.168.123.2                          │
│         ▲                                                                 │
│         │  Keonn AdvanNet (REST :3161, events :3177)                    │
│         │                                                                 │
│   rfid_scanner_server.py  ──►  http://<dog-wifi-ip>:8765/api/v1/...     │
│                                                                           │
│   (DimOS may also connect here via WebRTC for camera / lidar / control)   │
└───────────────────────────────────────────────────────────────────────────┘
                                    │
                          Wi‑Fi / hotspot
                                    │
                                    ▼
┌────────────────────────── Linux laptop ───────────────────────────────────┐
│                                                                           │
│   dimos run unitree-go2-rfid                                              │
│     ├── GO2Connection          (WebRTC to robot)                          │
│     ├── SLAM / navigation / perception  (unitree_go2 stack)                 │
│     ├── RfidModule               (polls RFID HTTP API on the dog)         │
│     ├── RerunBridgeModule        (3D visualization)                       │
│     └── WebsocketVisModule       (2D command center on :7779)             │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘
```

**Important:** The Vulcan reader is on the dog's Ethernet subnet (`192.168.123.x`). The laptop does **not** need direct access to `192.168.123.2` in the default configuration. `RfidModule` talks to `rfid_scanner_server.py` over the dog's **Wi‑Fi IP** on port **8765**.

---

## Prerequisites

| Component | Where | Notes |
|-----------|--------|-------|
| Linux (Ubuntu 22.04+ recommended) | Laptop | DimOS is Linux-first (x86_64) |
| Python 3.10+ | Laptop | 3.12 recommended; managed by `uv` |
| [uv](https://docs.astral.sh/uv/) | Laptop | Creates the venv and installs dependencies |
| Go2 on same network | Robot + laptop | Phone hotspot or lab Wi‑Fi |
| Vulcan reader powered & cabled | Robot | `ping 192.168.123.2` from the dog |
| `flask`, `requests` on robot | Robot | Use `uv sync --extra robot` or system Python |

---

## Quick start (uv)

```bash
git clone https://github.com/salirezav/dimos_rfid.git
cd dimos_rfid

# Create .venv, install DimOS + dimos-rfid (locked versions)
uv sync --extra unitree

# Register RFID blueprints with the dimos CLI
chmod +x dimos_rfid/integrate_with_dimos.sh
./dimos_rfid/integrate_with_dimos.sh
```

Verify DimOS sees the stack:

```bash
uv run dimos list | grep -i rfid
```

Run commands through `uv run` (no manual `source .venv/bin/activate` required), or activate `.venv` if you prefer.

Keep the project on a **native Linux filesystem** (e.g. `~/projects/`), not on a network mount or cross-OS bind mount. DimOS does heavy I/O during first startup (model weights, LFS data); a slow filesystem causes multi-minute hangs with little console output.

---

## Setup guide (full path)

These are the steps to go from a fresh Linux machine to a running `dimos run unitree-go2-rfid` stack.

### 1. Install uv and clone the repository

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh   # or: pip install uv

git clone https://github.com/salirezav/dimos_rfid.git
cd dimos_rfid
```

### 2. Create the environment and install dependencies

```bash
uv sync --extra unitree
```

This reads `pyproject.toml` and `uv.lock`, creates `.venv`, installs `dimos[base,unitree]`, and installs this repo's `dimos_rfid` package in editable mode.

Optional extras:

| Extra | Purpose | Command |
|-------|---------|---------|
| `unitree` | Go2 blueprints (`dimos[unitree]`) | `uv sync --extra unitree` |
| `robot` | Flask server deps for the onboard RFID HTTP API | `uv sync --extra robot` |

```bash
uv sync --all-extras    # unitree + robot
```

Verify:

```bash
uv run dimos list | grep unitree-go2
```

### 3. Integrate with the `dimos` CLI

DimOS discovers runnable stacks via blueprint files inside the `dimos` Python package. The integration script vendors the RFID module into `dimos/hardware/sensors/rfid/` and regenerates the blueprint registry so you can use `dimos run`:

```bash
chmod +x dimos_rfid/integrate_with_dimos.sh
./dimos_rfid/integrate_with_dimos.sh
```

After success, `dimos list` should include:

- `rfid-demo`
- `unitree-go2-rfid`
- `unitree-go2-rfid-agentic`

Re-run `./dimos_rfid/integrate_with_dimos.sh` whenever you **upgrade the `dimos` package**, because pip upgrades overwrite files under `site-packages/`.

### 4. Deploy the RFID HTTP server on the dog

SSH into the Go2 and start the scanner server:

```bash
ssh unitree@<dog-wifi-ip>
cd /path/to/"rfid scanner python"
python3 rfid_scanner_server.py
```

The server:

1. Connects to the reader at `192.168.123.2` via Keonn AdvanNet
2. Streams live tag events on TCP port 3177
3. Exposes HTTP API + web dashboard on port **8765** (`0.0.0.0`)

Verify from the laptop:

```bash
curl http://<dog-wifi-ip>:8765/api/v1/health
```

See [`rfid scanner python/RFID_SCANNER.md`](rfid%20scanner%20python/RFID_SCANNER.md) for hardware notes, ports, and troubleshooting.

### 5. Configure environment variables

On the laptop, before starting DimOS:

```bash
export ROBOT_IP=<go2-wifi-ip>          # WebRTC connection to the dog
export RFID_API_BASE=http://<go2-wifi-ip>:8765/api/v1
```

Optional:

```bash
export RFID_CONNECTION_MODE=http        # default; see dimos_rfid/README.md for "direct"
export RFID_SCANNER_PYTHON_DIR=/path/to/rfid scanner python  # only for direct mode
```

### 6. Run DimOS with RFID

**Full Go2 spatial stack + RFID** (SLAM, map, camera, navigation, RFID tags in Rerun):

```bash
uv run dimos run unitree-go2-rfid
```

**RFID + viewer only** (no robot connection):

```bash
export RFID_API_BASE=http://<dog-wifi-ip>:8765/api/v1
uv run dimos run rfid-demo
```

**Without registering blueprints** (alternative entry point):

```bash
uv run python -m dimos_rfid go2
```

**Collect an offline RFID + image + robot-pose dataset:**

```bash
uv run python -m dimos_rfid go2-dataset
```

Walk the dog, then press `Ctrl+C`. The session directory and portable ZIP are
finalized under `~/Downloads/dimos_rfid_datasets/`. Set `RFID_DATASET_DIR` to
choose a different folder. See [`dimos_rfid/README.md`](dimos_rfid/README.md#offline-dataset-collection)
for the file schema and recorder controls.

### 7. Verify RFID data

In a second terminal (while DimOS is running):

```bash
# LCM stream
uv run dimos topic echo /rfid/tags

# If using an agentic blueprint with MCP server
uv run dimos mcp call get_active_rfid_tags
```

---

## Visualization

`unitree-go2-rfid` inherits the standard Go2 visualization stack:

| Viewer mode | Command flag | What you get |
|-------------|--------------|--------------|
| Native Rerun (default) | `--viewer rerun` | Desktop `dimos-viewer` window (3D + camera) |
| Web dashboard | `--viewer rerun-web` | Browser opens `http://localhost:7779/` |
| 2D command center only | (any mode) | `http://localhost:7779/command-center` |
| No viewer | `--viewer none` | Headless |

Example with browser-based viewer (useful on remote/headless machines):

```bash
uv run dimos --viewer rerun-web run unitree-go2-rfid
```

**First startup is slow.** DimOS downloads large assets (LFS objects, YOLO weights) on the first run. The Rerun window appears only after `RerunBridgeModule` deploys — often several minutes after "Starting DimOS". Wait for log lines like:

```
Deployed module. module=RerunBridgeModule
Rerun bridge starting
```

RFID tag status appears as **text logs** in the Rerun viewer (via `RfidTagArray.to_rerun()`).

---

## Registered blueprints

| CLI name | Description |
|----------|-------------|
| `rfid-demo` | `RfidModule` + Rerun bridge only |
| `unitree-go2-rfid` | Same stack as `unitree-go2` + RFID |
| `unitree-go2-rfid-agentic` | Spatial + RFID + MCP agent (requires full DimOS agent dependencies) |

Blueprint composition for `unitree-go2-rfid`:

```
unitree_go2                 ← SLAM, navigation, mapping (same as `dimos run unitree-go2`)
  + RfidModule               ← polls dog RFID HTTP API
  + LCM transport            ← /rfid/tags
```

Details: [`dimos_rfid/README.md`](dimos_rfid/README.md).

---

## Troubleshooting

### `ModuleNotFoundError: No module named 'dimos_rfid'`

Run integration again:

```bash
./dimos_rfid/integrate_with_dimos.sh
```

The script vendors code into the `dimos` package so `dimos run` does not depend on a fragile editable path.

### DimOS hangs after "Starting DimOS"

Usually first-run asset download or slow disk I/O. Check:

```bash
uv run dimos log -f          # follow logs
uv run dimos status          # is it still running?
```

Ensure the project lives on a fast local disk.

### No visualization window

Startup may still be loading modules. Try:

- Wait for `RerunBridgeModule` in logs
- Open `http://localhost:7779/command-center` in a browser
- Use `uv run dimos --viewer rerun-web run unitree-go2-rfid`

### RFID tags not appearing

1. Confirm `rfid_scanner_server.py` is running on the dog
2. `curl http://<dog-ip>:8765/api/v1/health`
3. Check `RFID_API_BASE` points to the dog's **Wi‑Fi IP**, not `192.168.123.2`
4. `uv run dimos topic echo /rfid/tags`

### WebRTC errors on port 8081 then recovery

The log may show a failed connection attempt on port 8081 before succeeding on the standard path. If you see `Peer Connection State: connected` and `Data Channel Verification: OK`, the robot link is fine.

### Clock sync prompt

DimOS may ask to fix clock drift on first run. Answer `y` or install `systemd-timesyncd` for persistent sync.

### `unitree-go2-rfid` fails with `No module named 'sam2'`

Older versions of this blueprint extended `unitree_go2_spatial`, which includes `SecurityModule` (SAM2 / EdgeTAM). The current blueprint uses **`unitree_go2`** — the same stack as `dimos run unitree-go2`. Re-run `./dimos_rfid/integrate_with_dimos.sh` after pulling updates.

If you need spatial memory without SAM2, compose a custom blueprint with `unitree_go2_spatial` and `.disabled_modules(SecurityModule)`.

---

## Development workflow

| Task | Command |
|------|---------|
| Install / update deps | `uv sync --extra unitree` |
| Edit RFID module | Change files in `dimos_rfid/`, re-run integration |
| Re-register blueprints | `./dimos_rfid/integrate_with_dimos.sh` |
| Run without `dimos run` | `uv run python -m dimos_rfid go2` |
| Stop DimOS | `Ctrl+C` or `uv run dimos stop` |
| View logs | `uv run dimos log -f` |

---

## Roadmap

Planned extensions (see `dimos_rfid/README.md`):

1. **`RfidLocalizerModule`** — fuse RFID RSSI with robot odometry for tag localization
2. **Rerun 3D markers** — show estimated tag positions in the world view
3. **`SpatialMemory` integration** — persist last-seen tag locations

---

## Further reading

- [`dimos_rfid/README.md`](dimos_rfid/README.md) — module internals, integration options, API reference
- [`rfid scanner python/RFID_SCANNER.md`](rfid%20scanner%20python/RFID_SCANNER.md) — reader hardware & server setup
- [`rfid scanner python/RFID_API.md`](rfid%20scanner%20python/RFID_API.md) — HTTP API endpoints
- [DimOS documentation](https://github.com/dimensionalOS/dimos)
