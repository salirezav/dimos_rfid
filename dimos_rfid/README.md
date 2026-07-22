# DimOS RFID module (`dimos_rfid`)

DimOS integration for the **Vulcan RFID Titanium** UHF reader on a Unitree Go2. This package wraps the existing [`rfid scanner python`](../rfid%20scanner%20python/) code and exposes tag reads as a first-class DimOS **Module** with LCM streams, Rerun visualization, and optional agent skills.

---

## Table of contents

- [Design overview](#design-overview)
- [How it works](#how-it-works)
- [Package contents](#package-contents)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [DimOS integration](#dimos-integration)
- [Running](#running)
- [Connection modes](#connection-modes)
- [Blueprints](#blueprints)
- [Module API](#module-api)
- [Message types](#message-types)
- [Environment variables](#environment-variables)
- [Visualization](#visualization)
- [Extending the module](#extending-the-module)
- [Troubleshooting](#troubleshooting)

---

## Design overview

### Two-process architecture

RFID hardware access and DimOS run on **different machines**:

| Process | Host | Responsibility |
|---------|------|----------------|
| `rfid_scanner_server.py` | **Go2 onboard** | Talks to reader at `192.168.123.2`; serves HTTP API on `:8765` |
| `RfidModule` (this package) | **Linux laptop** | Polls HTTP API; publishes tags into DimOS |

```
Reader (192.168.123.2)
    │  Keonn AdvanNet
    ▼
rfid_service.RfidScanner  ──in-process──►  rfid_scanner_server.py
                                                    │
                                          HTTP :8765/api/v1
                                                    │
                                                    ▼
                                          RfidModule (DimOS)
                                                    │
                                          LCM /rfid/tags
                                                    │
                                    ┌───────────────┼───────────────┐
                                    ▼               ▼               ▼
                              RerunBridge    Other modules    MCP agent skills
```

The laptop never needs Layer-2 access to `192.168.123.2` when using **HTTP mode** (the default). Only the dog must be on the same Ethernet segment as the reader.

### Why a separate package?

DimOS blueprints are discovered from Python files inside the `dimos` package tree (`dimos/robot/all_blueprints.py`). This repo keeps RFID code in `dimos_rfid/` (outside the upstream `dimos` distribution) and uses `integrate_with_dimos.sh` to vendor it into `dimos/hardware/sensors/rfid/` at install time. That gives you:

- Editable development in `dimos_rfid/`
- Standard `dimos run unitree-go2-rfid` CLI
- No fork of the upstream DimOS repository

---

## How it works

### `RfidModule` lifecycle

1. **`start()`** — reads `RFID_CONNECTION_MODE` (`http` or `direct`).
2. **HTTP mode (default)** — starts a reactive poll loop at `poll_hz` (default 2 Hz). Each tick `GET`s `{RFID_API_BASE}/tags` (or equivalent endpoint via `rfid_service.to_api_payload()` shape).
3. **Direct mode** — imports `rfid_service.RfidScanner` in-process, connects to the reader IP, registers an `on_tag` callback for live events.
4. **`rfid_tags.publish()`** — emits `RfidTagArray` on the module's `Out` stream.
5. **LCM transport** — blueprint maps `rfid_tags` → `/rfid/tags` so any DimOS module or `dimos topic echo` can subscribe.
6. **`to_rerun()`** — `RfidTagArray` implements Rerun conversion; `RerunBridgeModule` renders tag status as text logs in the 3D viewer.
7. **Agent skills** — `@skill` methods (`get_active_rfid_tags`, etc.) are exposed when the blueprint includes `McpServer` (agentic variant).

### Static transform

`RfidModule` publishes a static transform `base_link → rfid_antenna` (configurable Z offset, default 0.25 m) for future localization and Rerun frame alignment.

### Data flow from API to DimOS

```python
# HTTP response (from rfid_scanner_server.py)
{"tags": [{"epc": "...", "rssi_dbm": -42, "in_range": true, ...}], ...}

# Converted in RfidModule
array = RfidTagArray.from_api_payload(payload)

# Published to LCM
self.rfid_tags.publish(array)   # → /rfid/tags#dimos_rfid.msgs.RfidTagArray
```

---

## Package contents

| File | Purpose |
|------|---------|
| `rfid_module.py` | `RfidModule` + `RfidModuleConfig` — core DimOS Module |
| `msgs.py` | `RfidTag`, `RfidTagArray` dataclasses with `to_rerun()` |
| `_backend.py` | Locates `rfid scanner python/` and constructs `RfidScanner` for direct mode |
| `semantic_map.py` | `SemanticOccupancyGrid3D` + Class A/B material labels |
| `semantic_particle_filter.py` | `SemanticParticleFilter3D` + multipath LOS gate |
| `rfid_tracker.py` | `RFIDTracker` orchestrator (per-tag particle filters) |
| `rfid_semantic_localizer.py` | DimOS module: TF pose + RFID tags → particle filter |
| `semantic_rfid_blueprints.py` | `unitree_go2_rfid_semantic` blueprint |
| `demo_blueprint.py` | `rfid_demo` — RFID + Rerun only |
| `go2_blueprints.py` | `unitree_go2_rfid` — Go2 spatial stack + RFID |
| `go2_agentic_blueprints.py` | `unitree_go2_rfid_agentic` — adds MCP agent (separate file to avoid import-time agent deps) |
| `__main__.py` | `python -m dimos_rfid {demo,go2,go2-agentic,semantic}` runner |
| `integrate_with_dimos.sh` | Vendors into `dimos` package + regenerates blueprint registry |

Repo root also has [`run_semantic_rfid.py`](../run_semantic_rfid.py) — one-command DimOS launcher for the semantic localizer.

Project metadata and dependencies live in the repository root: `pyproject.toml` and `uv.lock`.

---

## Prerequisites

1. **Linux x86_64** with **[uv](https://docs.astral.sh/uv/)** installed.

2. **DimOS + this package** via uv from the repository root:

   ```bash
   git clone https://github.com/salirezav/dimos_rfid.git
   cd dimos_rfid
   uv sync --extra unitree
   ```

3. **`rfid_scanner_server.py` running on the Go2** (recommended):

   ```bash
   cd "rfid scanner python"
   python3 rfid_scanner_server.py
   ```

   On the robot you can also use `uv sync --extra robot` from the repo root if uv is available there.

---

## Installation

From the repository root:

```bash
cd /path/to/dimos_rfid
uv sync --extra unitree
```

This creates `.venv`, installs locked versions of `dimos[base,unitree]`, and installs the `dimos_rfid` package in editable mode.

Use `uv run` to invoke tools without activating the venv, e.g. `uv run dimos list`.

---

## DimOS integration

### Recommended: `integrate_with_dimos.sh`

```bash
./dimos_rfid/integrate_with_dimos.sh
```

The script:

1. Runs `uv sync --extra unitree --frozen` to ensure the environment matches `uv.lock`
2. Copies module files into `{dimos}/hardware/sensors/rfid/` inside your environment's `site-packages`
3. Rewrites imports from `dimos_rfid.*` → `dimos.hardware.sensors.rfid.*` in the vendored copies (so `dimos run` does not depend on a separate import path)
4. Regenerates `dimos/robot/all_blueprints.py` by scanning for `autoconnect(...)` assignments
5. Verifies `rfid-demo` and `unitree-go2-rfid` import cleanly

**Re-run after every `dimos` package upgrade**, because pip overwrites `site-packages`.

### What gets registered

| Blueprint variable | CLI name | Module path after integration |
|--------------------|----------|-------------------------------|
| `rfid_demo` | `rfid-demo` | `dimos.hardware.sensors.rfid.demo_blueprint:rfid_demo` |
| `unitree_go2_rfid` | `unitree-go2-rfid` | `dimos.hardware.sensors.rfid.go2_blueprints:unitree_go2_rfid` |
| `unitree_go2_rfid_agentic` | `unitree-go2-rfid-agentic` | `dimos.hardware.sensors.rfid.go2_agentic_blueprints:unitree_go2_rfid_agentic` |

DimOS CLI naming rule: `snake_case` blueprint variables become `kebab-case` commands.

### Alternative integration paths

#### Option A — No CLI registration (`python -m dimos_rfid`)

```bash
export ROBOT_IP=<go2-ip>
export RFID_API_BASE=http://<go2-ip>:8765/api/v1
uv run python -m dimos_rfid go2
```

Calls `ModuleCoordinator.build(blueprint).loop()` — identical runtime to `dimos run`, without touching `all_blueprints.py`.

#### Option B — Programmatic

```python
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos_rfid.go2_blueprints import unitree_go2_rfid

ModuleCoordinator.build(unitree_go2_rfid).loop()
```

#### Option C — Compose into your own blueprint

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos_rfid.rfid_module import RfidModule
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_spatial import unitree_go2_spatial

my_stack = autoconnect(
    unitree_go2_spatial,
    RfidModule.blueprint(api_base="http://10.42.0.1:8765/api/v1"),
)
```

#### Option D — Upstream DimOS source tree

If you develop against a `dimos` git clone, you can symlink or copy this package under `dimos/hardware/sensors/rfid/` and run:

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

Blueprint files must live under the `dimos/` package and use top-level `autoconnect(...)` assignments (not inside functions) for auto-discovery.

---

## Running

### Environment

```bash
export ROBOT_IP=<go2-wifi-ip>
export RFID_API_BASE=http://<go2-wifi-ip>:8765/api/v1
```

Use the dog's **Wi‑Fi / hotspot IP**, not the reader's Ethernet IP (`192.168.123.2`).

### Commands

```bash
# Full Go2 + RFID (recommended for tag ingest only)
uv run dimos run unitree-go2-rfid

# Go2 + RFID + semantic particle-filter localization
uv run python run_semantic_rfid.py
# or:
uv run python -m dimos_rfid semantic

# RFID viewer only
uv run dimos run rfid-demo

# With web-based visualization
uv run dimos --viewer rerun-web run unitree-go2-rfid

# Without dimos CLI registration
uv run python -m dimos_rfid go2
```

### Verify

```bash
# While DimOS is running
uv run dimos topic echo /rfid/tags

# Agentic blueprint (if MCP server is up)
uv run dimos mcp call get_active_rfid_tags
uv run dimos agent-send "what RFID tags do you see?"
```

---

## Connection modes

| Mode | When to use | Configuration |
|------|-------------|---------------|
| **`http`** (default) | `rfid_scanner_server.py` on the dog | `RFID_API_BASE=http://<dog-ip>:8765/api/v1` |
| **`direct`** | DimOS host can reach `192.168.123.2` directly | `RFID_CONNECTION_MODE=direct` |

### HTTP mode (recommended)

- Server runs on the Go2 alongside the reader
- DimOS polls REST endpoints every `poll_hz` seconds
- No `rfid scanner python` code needed on the laptop

### Direct mode

- `RfidModule` imports `rfid_service.RfidScanner` in-process via `_backend.py`
- Requires `rfid scanner python/` on disk; located automatically from:
  1. `$RFID_SCANNER_PYTHON_DIR` if set
  2. Sibling of `dimos_rfid/` (`../rfid scanner python`)
  3. `~/projects/Dimos/rfid scanner python`
- DimOS machine must have network route to `192.168.123.2` (unusual for laptop setups)

---

## Blueprints

### `rfid_demo`

```python
autoconnect(
    RfidModule.blueprint(...),
    RerunBridgeModule.blueprint(),
).transports({("rfid_tags", RfidTagArray): pLCMTransport("/rfid/tags")})
```

RFID tags + Rerun text overlay. No robot required.

### `unitree_go2_rfid`

```python
autoconnect(
    unitree_go2,   # SLAM, navigation, mapping (same as `dimos run unitree-go2`)
    RfidModule.blueprint(...),
).transports({("rfid_tags", RfidTagArray): pLCMTransport("/rfid/tags")})
```

Inherits everything from `unitree_go2`:

- WebRTC connection to Go2 (`GO2Connection`)
- Voxel mapping, costmaps, A* planner, frontier exploration, patrolling
- Rerun 3D viewer + websocket command center (`:7779`)

Does **not** include `unitree_go2_spatial` extras (`SpatialMemory`, `SecurityModule` / SAM2). That keeps it aligned with `dimos run unitree-go2`, which does not require the `sam2` package.

### `unitree_go2_rfid_agentic`

Same as above plus `McpServer`, `McpClient`, and `_common_agentic` for LLM agent interaction. Requires full DimOS agent/web dependencies. Kept in a separate file (`go2_agentic_blueprints.py`) so importing `unitree_go2_rfid` does not pull agent imports at module load time.

---

## Module API

### Output stream

| Stream | Type | LCM topic |
|--------|------|-----------|
| `rfid_tags` | `Out[RfidTagArray]` | `/rfid/tags` |

Subscribe from another DimOS module:

```python
from dimos.core.module import Module
from dimos.core.stream import In
from dimos_rfid.msgs import RfidTagArray

class MyListener(Module):
    rfid_tags: In[RfidTagArray]

    @rpc
    def start(self):
        super().start()
        self.rfid_tags.subscribe(self._on_tags)

    def _on_tags(self, msg: RfidTagArray):
        for tag in msg.active_tags():
            print(tag.epc, tag.rssi_dbm)
```

### Agent skills

Available when the running blueprint includes `McpServer`:

| Skill | Description |
|-------|-------------|
| `get_active_rfid_tags()` | Human-readable list of in-range tags with RSSI |
| `lookup_rfid_tag(epc)` | Look up one tag by EPC hex string |
| `get_rfid_reader_status()` | Reader connection health and tag counts |
| `get_estimated_target_location(tag_id)` | 3D particle-filter mean (requires semantic localizer / `--agentic`) |
| `get_location_confidence(tag_id)` | Cluster-variance confidence in `[0, 1]` |

---

## Semantic particle filter

3D Monte Carlo localization that fuses unidirectional RSSI, robot TF pose, and a semantic occupancy map.

| Material class | Behavior |
|----------------|----------|
| **Class A / STRUCTURAL** (walls, floor, pillars) | Particles inside/behind → weight 0; early ray hit → multipath discount |
| **Class B / INVENTORY** (boxes, pallets) | Particles stay valid; expected RSSI attenuated per meter penetrated |

### How to work with it (quick)

1. Start the RFID HTTP server on the Go2.
2. Put your tag-of-interest in [`rfid_focus.txt`](rfid_focus.txt) (same idea as the experimental module):

   ```text
   # one EPC or short suffix per line
   8f
   ```

   Empty file = localize **all** in-range tags. Edit while running; next poll picks it up.
3. Run DimOS with the semantic stack:

   ```bash
   export ROBOT_IP=<go2-wifi-ip>
   export RFID_API_BASE=http://<go2-wifi-ip>:8765/api/v1
   uv run python run_semantic_rfid.py
   ```
4. Walk the dog so it sees the tag from several poses/angles. Watch logs for:

   ```text
   TOI …8f @ [x, y, z] m  conf=0.72
   ```
5. Query the estimate (agentic mode) or from code:

   ```bash
   uv run python run_semantic_rfid.py --agentic
   # then: get_estimated_target_location("8f")
   #       get_location_confidence("8f")
   ```

### Inputs / outputs

```
                    ┌──────────────────────────────────────┐
  RFID HTTP API ──► │ RfidModule                           │
                    │   Out: rfid_tags  (/rfid/tags)       │
                    └──────────────────┬───────────────────┘
                                       │  In: rfid_tags
  TF world←antenna ───────────────────►│
  rfid_focus.txt (TOI filter) ────────►│ RfidSemanticLocalizerModule
  semantic map (.npz / blank) ────►│
                    └──────────────────┬───────────────────┘
                                       │
                    Outputs:           ▼
                      • logs: TOI @ [x,y,z] + confidence
                      • skills: get_estimated_target_location(tag_id)
                                get_location_confidence(tag_id)
                      • Python: tracker.get_estimated_target_location(tag_id)
                                tracker.get_location_confidence(tag_id)
```

| Kind | Name | What it is |
|------|------|------------|
| **Input** | `rfid_tags` | Live tag list from `RfidModule` (EPC + RSSI) |
| **Input** | TF pose | Dog/antenna position + yaw/pitch in `world` |
| **Input** | `rfid_focus.txt` | Which EPC(s) to treat as TOI (empty = all) |
| **Input** | Semantic map | Class A/B voxels (optional `.npz`; default = free space + floor) |
| **Output** | Log line | Estimated `[x,y,z]` + confidence for focused tags |
| **Output** | Agent skills | Human-readable location / confidence strings |
| **Output** | `RFIDTracker` API | `np.ndarray` location + `float` confidence |

Unlike the experimental `rfid_module/` multilateration UI, this module does **not** draw Rerun 3D markers yet — primary outputs are logs + query APIs.

### Run with DimOS (Go2)

From the **repository root** (RFID HTTP server must already be running on the dog):

```bash
export ROBOT_IP=<go2-wifi-ip>
export RFID_API_BASE=http://<go2-wifi-ip>:8765/api/v1

# Recommended launcher
uv run python run_semantic_rfid.py

# With MCP agent skills
uv run python run_semantic_rfid.py --agentic

# Equivalent module entry point
uv run python -m dimos_rfid semantic
```

After `./dimos_rfid/integrate_with_dimos.sh`:

```bash
uv run dimos run unitree-go2-rfid-semantic
```

### What the stack does

```
unitree_go2                      ← SLAM, TF, camera, lidar
  + RfidModule                   ← polls RFID HTTP API → /rfid/tags
  + RfidSemanticLocalizerModule  ← TF pose + tags → RFIDTracker particle filter
  + RerunBridgeModule            ← visualization (tag list panel)
```

On each focused in-range tag with RSSI, the localizer:

1. Looks up `world ← rfid_antenna` (falls back to `base_link`)
2. Runs a multipath LOS gate against the semantic map
3. Updates that tag’s particle filter
4. Periodically logs `[x, y, z]` + confidence

### Focus file (TOI selection)

File: [`rfid_focus.txt`](rfid_focus.txt) (or `$RFID_FOCUS_FILE`).

| File contents | Behavior |
|---------------|----------|
| Empty / comments only | Localize **all** in-range tags |
| `8f` | Only EPCs containing `8f` (usually the suffix) |
| Full EPC hex | Only that exact tag |

You can also call RPC `set_focus(["8f"])` at runtime.

### Environment / tuning

| Variable | Default | Description |
|----------|---------|-------------|
| `RFID_FOCUS_FILE` | `dimos_rfid/rfid_focus.txt` | TOI focus list path |
| `RFID_PF_PARTICLES` | `5000` | Particles per tag |
| `RFID_PF_XMIN` / `XMAX` | `-5` / `15` | Map X bounds (m) |
| `RFID_PF_YMIN` / `YMAX` | `-5` / `15` | Map Y bounds (m) |
| `RFID_PF_ZMIN` / `ZMAX` | `0` / `3` | Map Z bounds (m) |
| `RFID_PF_MAP_RES` | `0.2` | Voxel resolution (m) |
| `RFID_SEMANTIC_MAP` | _(empty)_ | Path to `.npz` map (`labels`, `origin`, `resolution`) |
| `RFID_PF_LOG_HZ` | `0.5` | Estimate log rate (0 disables) |

Without `RFID_SEMANTIC_MAP`, the module builds an empty free-space grid with a structural floor slab (tags cannot be hypothesized underground). Populate Class B inventory / Class A walls later via `set_semantic_map()` or a saved `.npz`.

Save a map for reload:

```python
import numpy as np
from dimos_rfid import SemanticOccupancyGrid3D, MaterialClass

grid = SemanticOccupancyGrid3D(origin=(0, 0, 0), resolution=0.2, shape=(50, 50, 15))
grid.set_box([4, 0, 0], [4.4, 10, 3], MaterialClass.STRUCTURAL)
grid.set_box([2, 4, 0.2], [5, 6, 2], MaterialClass.INVENTORY)
np.savez("warehouse.npz", labels=grid.labels, origin=grid.origin, resolution=grid.resolution)
# then: export RFID_SEMANTIC_MAP=/path/to/warehouse.npz
```

### Unit tests (no robot)

```bash
uv run pytest tests/test_semantic_particle_filter.py -v
```

### Library API (offline / scripts)

```python
from dimos_rfid import RFIDTracker, SemanticOccupancyGrid3D, MaterialClass

tracker = RFIDTracker(bounds=((-5, 15), (-5, 15), (0, 3)))
# tracker.ingest(dog_x, dog_y, dog_z, yaw, pitch, tag_id, rssi, grid)
# tracker.get_estimated_target_location(tag_id)
# tracker.get_location_confidence(tag_id)
```

---

### Configuration (`RfidModuleConfig`)

| Field | Default | Description |
|-------|---------|-------------|
| `connection_mode` | `http` | `http` or `direct` |
| `api_base` | `$RFID_API_BASE` or `http://localhost:8765/api/v1` | HTTP API root |
| `reader_host` | `192.168.123.2` | Reader IP (direct mode) |
| `reader_user` / `reader_password` | `admin` / `admin` | Digest auth (direct mode) |
| `poll_hz` | `2.0` | HTTP poll rate |
| `stale_seconds` | `5.0` | Tag staleness threshold |
| `antenna_frame_id` | `rfid_antenna` | TF frame name |
| `antenna_offset_z` | `0.25` | Antenna height above `base_link` (meters) |

---

## Message types

### `RfidTag`

Single tag observation:

| Field | Type | Description |
|-------|------|-------------|
| `epc` | `str` | Electronic Product Code (hex) |
| `rssi_dbm` | `float \| None` | Received signal strength |
| `antenna` | `int \| None` | Antenna port |
| `frequency_khz` | `int \| None` | Carrier frequency |
| `read_count` | `int` | Session read count |
| `in_range` | `bool` | Seen within `stale_seconds` |
| `last_seen` | `float` | Unix timestamp |
| `name` | `str` | Short display label |

### `RfidTagArray`

Batch message for one publish cycle. Implements `to_rerun()` → `rr.TextLog` listing in-range tags for the Rerun bridge.

---

## Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `RFID_API_BASE` | `http://localhost:8765/api/v1` | HTTP API base URL |
| `RFID_CONNECTION_MODE` | `http` | `http` or `direct` |
| `RFID_SCANNER_PYTHON_DIR` | (auto-detect) | Path to `rfid scanner python/` for direct mode |
| `VULCAN_READER_HOST` | `192.168.123.2` | Reader IP (direct mode) |
| `VULCAN_READER_USER` | `admin` | Digest auth user |
| `VULCAN_READER_PASS` | `admin` | Digest auth password |
| `ROBOT_IP` | — | Go2 Wi‑Fi IP (DimOS `GlobalConfig`) |
| `RFID_PF_PARTICLES` | `5000` | Semantic PF particle count |
| `RFID_SEMANTIC_MAP` | _(empty)_ | Optional `.npz` semantic occupancy map |
| `RFID_PF_XMIN` … `RFID_PF_ZMAX` | see semantic section | Particle / map bounds |

Reader-side variables (`RFID_WEB_PORT`, `RFID_STALE_SECONDS`, etc.) are documented in [`rfid scanner python/RFID_SCANNER.md`](../rfid%20scanner%20python/RFID_SCANNER.md).

---

## Visualization

### Rerun (default)

`RfidTagArray.to_rerun()` produces a `TextLog` archetype. When `RerunBridgeModule` is in the blueprint (all Go2 stacks include it), active tags appear in the Rerun log panel.

Future work: `Points3D` / `Arrows3D` at estimated world positions once localization is implemented.

### Debug without Rerun

```bash
uv run dimos topic echo /rfid/tags
```

### RFID-only demo

`rfid-demo` uses `RerunBridgeModule` directly (no Go2). Useful for verifying the HTTP API connection before involving the robot.

---

## Extending the module

### Adding to a custom blueprint

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import pLCMTransport
from dimos_rfid.rfid_module import RfidModule
from dimos_rfid.rfid_semantic_localizer import RfidSemanticLocalizerModule
from dimos_rfid.msgs import RfidTagArray

my_blueprint = autoconnect(
    # ... your modules ...
    RfidModule.blueprint(api_base="http://10.0.0.1:8765/api/v1"),
    RfidSemanticLocalizerModule.blueprint(n_particles=5000),
).transports({
    ("rfid_tags", RfidTagArray): pLCMTransport("/rfid/tags"),
})
```

After adding new top-level `autoconnect` assignments under `dimos/`, regenerate the registry (integration script or `pytest dimos/robot/test_all_blueprints_generation.py`).

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `No module named 'dimos_rfid'` on `dimos run` | Integration not run or overwritten by dimos upgrade | `./dimos_rfid/integrate_with_dimos.sh` |
| No tags on `/rfid/tags` | Server not running or wrong `RFID_API_BASE` | `curl http://<dog>:8765/api/v1/health` |
| `RFID reader unreachable` in logs | Dog server down or network isolation | Start `rfid_scanner_server.py` on Go2 |
| Tags in API but not in Rerun | `RerunBridgeModule` not started yet | Wait for full startup; check `dimos log -f` |
| `unitree-go2-rfid-agentic` import error | Missing DimOS web/agent packages | Use `unitree-go2-rfid` or install full dimos extras |
| `No module named 'sam2'` | Old spatial-based blueprint or `unitree-go2-spatial` | Use current `unitree-go2-rfid` (based on `unitree_go2`); re-run `integrate_with_dimos.sh` |
| Direct mode `FileNotFoundError` | `rfid scanner python/` not found | Set `RFID_SCANNER_PYTHON_DIR` |

---

## Related documentation

- [Project README](../README.md) — full setup from scratch
- [RFID scanner server](../rfid%20scanner%20python/RFID_SCANNER.md) — hardware & onboard server
- [RFID HTTP API](../rfid%20scanner%20python/RFID_API.md) — endpoint reference
