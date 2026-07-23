# Semantic RFID localizer — how to use it

3D particle-filter localization for a tag-of-interest (TOI) on the Unitree Go2.
Fuses RFID RSSI, robot TF pose, and an optional semantic occupancy map
(walls vs boxes/pallets).

Launcher: [`run_semantic_rfid.py`](run_semantic_rfid.py)  
Focus file: [`dimos_rfid/rfid_focus.txt`](dimos_rfid/rfid_focus.txt)

---

## Quick start

1. Start `rfid_scanner_server.py` on the Go2.
2. Put your tag ID (full EPC or short suffix) in `dimos_rfid/rfid_focus.txt`:

   ```text
   # one EPC or suffix per line; empty file = all tags
   8f
   ```

3. On the laptop:

   ```bash
   cd /path/to/dimos_rfid
   export ROBOT_IP=<go2-wifi-ip>
   export RFID_API_BASE=http://<go2-wifi-ip>:8765/api/v1

   uv run python run_semantic_rfid.py
   ```

4. Walk the dog so it sees the tag from several poses. Watch logs:

   ```text
   TOI …8f @ [1.23, 4.56, 0.80] m  conf=0.72
   ```

That log line **is** the location estimate. You do not need an agent for basic use.

---

## Inputs / outputs

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
                      • MCP skills (optional): get_estimated_target_location
                                               get_location_confidence
```

| Kind | Name | What it is |
|------|------|------------|
| **Input** | `rfid_tags` | Live tags from `RfidModule` (EPC + RSSI) |
| **Input** | TF pose | Dog/antenna position + yaw/pitch in `world` |
| **Input** | `rfid_focus.txt` | Which EPC(s) to localize (empty = all) |
| **Input** | Semantic map | Class A/B voxels (optional; default = free space + floor) |
| **Output** | Log line | Estimated `[x,y,z]` + confidence |
| **Output** | MCP skills | Queryable location / confidence strings |

This module does **not** draw Rerun 3D markers yet (unlike the experimental
`rfid_module/` multilateration UI). Primary outputs are **logs** and optional **MCP queries**.

### Material classes (semantic map)

| Class | Meaning | Behavior |
|-------|---------|----------|
| **A / STRUCTURAL** | Walls, floor, metal pillars | Particles inside/behind → rejected; early ray hit → multipath discount |
| **B / INVENTORY** | Boxes, pallets, totes | Particles stay valid; RSSI attenuated per meter penetrated |

---

## How do I query `get_estimated_target_location("8f")`?

You do **not** type that into the DimOS Rerun 3D UI. Options:

### Option A — Just read the logs (simplest)

```bash
uv run python run_semantic_rfid.py
# or: uv run dimos log -f
```

Look for `TOI … @ [x, y, z] m  conf=…`.

### Option B — Call the skill via MCP (no LLM / no API key)

MCP is DimOS’s **local tool server**, not a paid cloud API.

1. Start with agent skills / MCP enabled:

   ```bash
   uv run python run_semantic_rfid.py --agentic
   ```

2. In a **second terminal**:

   ```bash
   uv run dimos mcp list-tools
   uv run dimos mcp call get_estimated_target_location -a tag_id=8f
   uv run dimos mcp call get_location_confidence -a tag_id=8f
   ```

### Option C — Natural-language agent chat (needs an LLM)

Only if you want to ask things like “where is tag 8f?” in English.
That requires an LLM behind DimOS’s `McpClient` (see below).

```bash
uv run dimos agent-send "where is RFID tag 8f?"
```

---

## Setting up “agentic” DimOS (first time)

Two different pieces people mix up:

| Piece | Role | Needs a cloud API key? |
|-------|------|-------------------------|
| **McpServer** | Exposes skills (`get_estimated_…`) for `dimos mcp call` | **No** |
| **McpClient / LLM agent** | Chatbot that calls those skills in English | **Yes** (or local Ollama) |

You do **not** buy or configure an “MCP API.” MCP runs on your machine inside DimOS.

### 1. Install agent extras (if missing)

```bash
cd /path/to/dimos_rfid
uv sync --extra unitree
uv pip install "dimos[agents,web]"
```

### 2. Choose a model for the chat agent

**OpenAI (default model is `gpt-4o`):**

```bash
export OPENAI_API_KEY=sk-...
```

**Or local Ollama** (no cloud key): install/run [Ollama](https://ollama.com), pull a model
(e.g. `qwen3:8b`), then point DimOS at `ollama:…` (see DimOS `unitree-go2-agentic-ollama`
blueprint pattern). Our `run_semantic_rfid.py --agentic` currently uses the default
`McpClient` model (`gpt-4o`) unless you change it.

### 3. Run

```bash
export ROBOT_IP=<go2-wifi-ip>
export RFID_API_BASE=http://<go2-wifi-ip>:8765/api/v1
export OPENAI_API_KEY=sk-...   # only for chatty agent

uv run python run_semantic_rfid.py --agentic
```

### 4. Query

```bash
# Direct skill call (works whenever McpServer is up — no LLM required for the call itself)
uv run dimos mcp call get_estimated_target_location -a tag_id=8f

# Natural language (needs working LLM / API key or Ollama)
uv run dimos agent-send "where is RFID tag 8f?"
```

If `--agentic` fails with missing web/agent modules, fall back to **Option A (logs)** —
localization still works without the agent.

---

## Focus file (same UX as experimental `rfid_module`)

Edit [`dimos_rfid/rfid_focus.txt`](dimos_rfid/rfid_focus.txt) while DimOS is running:

| Contents | Behavior |
|----------|----------|
| Empty / comments only | Localize **all** in-range tags |
| `8f` | Only EPCs containing `8f` |
| Full EPC hex | Only that tag |

Override path: `export RFID_FOCUS_FILE=/path/to/my_focus.txt`

---

## Optional semantic map

Without a map, the localizer uses free space + a structural floor slab.

```bash
export RFID_SEMANTIC_MAP=/path/to/warehouse.npz
```

`.npz` keys: `labels` (int8 grid), `origin` (xyz), `resolution` (meters).

```python
import numpy as np
from dimos_rfid import SemanticOccupancyGrid3D, MaterialClass

grid = SemanticOccupancyGrid3D(origin=(0, 0, 0), resolution=0.2, shape=(50, 50, 15))
grid.set_box([4, 0, 0], [4.4, 10, 3], MaterialClass.STRUCTURAL)
grid.set_box([2, 4, 0.2], [5, 6, 2], MaterialClass.INVENTORY)
np.savez("warehouse.npz", labels=grid.labels, origin=grid.origin, resolution=grid.resolution)
```

---

## Tuning env vars

| Variable | Default | Description |
|----------|---------|-------------|
| `RFID_FOCUS_FILE` | `dimos_rfid/rfid_focus.txt` | TOI list |
| `RFID_PF_PARTICLES` | `5000` | Particles per tag |
| `RFID_PF_XMIN` / `XMAX` | `-5` / `15` | X bounds (m) |
| `RFID_PF_YMIN` / `YMAX` | `-5` / `15` | Y bounds (m) |
| `RFID_PF_ZMIN` / `ZMAX` | `0` / `3` | Z bounds (m) |
| `RFID_PF_MAP_RES` | `0.2` | Voxel size (m) |
| `RFID_SEMANTIC_MAP` | _(empty)_ | Optional `.npz` map |
| `RFID_PF_LOG_HZ` | `0.5` | Estimate log rate |

---

## Other ways to start

```bash
uv run python -m dimos_rfid semantic

# After ./dimos_rfid/integrate_with_dimos.sh
uv run dimos run unitree-go2-rfid-semantic
```

### Unit tests (no robot)

```bash
uv run pytest tests/test_semantic_particle_filter.py -v
```

### Offline library API

```python
from dimos_rfid import RFIDTracker, SemanticOccupancyGrid3D

tracker = RFIDTracker(bounds=((-5, 15), (-5, 15), (0, 3)))
# tracker.ingest(dog_x, dog_y, dog_z, yaw, pitch, tag_id, rssi, grid)
# tracker.get_estimated_target_location(tag_id)
# tracker.get_location_confidence(tag_id)
```

---

## Related docs

- [README.md](README.md) — full DimOS + Go2 + RFID setup
- [dimos_rfid/README.md](dimos_rfid/README.md) — package / blueprint details
