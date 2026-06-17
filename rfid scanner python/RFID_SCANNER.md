# Vulcan RFID Scanner — Documentation

This project provides Python tools to read UHF RFID tags from a **Vulcan RFID Titanium** reader attached to a Unitree robot over Ethernet. The main entry point for most users is **`rfid_scanner_server.py`**, which runs a live web dashboard.

---

## What We Discovered About the Reader

The Vulcan Titanium reader at `192.168.123.2` is **not** a raw ThingMagic/Mercury device on the network. It runs **Keonn AdvanNet** firmware (the built-in web UI is branded Keonn/Vulcan). That matters because:

| Approach | Result |
|----------|--------|
| `python-mercuryapi` with `tmr://192.168.123.2` | **Fails** — tries LLRP on port 5084, which is closed |
| Keonn AdvanNet REST + TCP event stream | **Works** |

### Network ports (verified)

| Port | Protocol | Purpose |
|------|----------|---------|
| **3161** | HTTP + **Digest** auth | REST control API (start/stop, inventory, config) |
| **3177** | **ADVANNET/1.1** framed XML | Real-time tag event stream (no auth) |
| **80** | HTTP + Digest auth | Factory web dashboard |
| **11987** | WebSocket | Dashboard live updates (needed if tunneling the UI) |
| **8080** | HTTP | Keonn bootloader utility |
| **5084** | Closed | LLRP — not available on this reader |

### Device identity (example unit)

| Field | Value |
|-------|-------|
| Device ID | `VUL-TITANIUM-4PG-4e4e` |
| MAC | `68:5e:1c:cc:4e:4e` |
| RF module | M7e Tera (ThingMagic hardware, accessed via AdvanNet) |
| Default credentials | `admin` / `admin` (HTTP **Digest**, not Basic) |

### Authentication note

REST calls on port 3161 require **HTTP Digest** authentication:

```bash
curl --digest -u admin:admin http://192.168.123.2:3161/devices
```

Plain Basic auth (`-u admin:admin` without `--digest`) returns 401.

---

## Project Files

```
churan_alireza/
├── rfid_scanner_server.py   # Web dashboard + HTTP API server (start here)
├── rfid_service.py          # Python API (RfidScanner) — use in your own code
├── vulcan_rfid_reader.py    # Low-level AdvanNet client + CLI
├── minimal.py               # Minimal streaming example
├── templates/
│   └── rfid_scanner.html    # Web UI (auto-refreshes every 1s)
├── RFID_SCANNER.md          # This file — setup & hardware notes
└── RFID_API.md              # API reference — Python & HTTP endpoints
```

### `vulcan_rfid_reader.py`

Reusable library and command-line tool. Provides:

- `AdvanNetClient` — REST control on port 3161
- `AdvanNetEventStream` — TCP event parser for port 3177
- `parse_event_xml()` — converts XML events into `TagRead` objects

CLI examples:

```bash
python3 vulcan_rfid_reader.py diagnose
python3 vulcan_rfid_reader.py stream --duration 30
python3 vulcan_rfid_reader.py inventory --start
```

### `minimal.py`

Short example that starts the reader and prints EPCs from the event stream:

```python
from vulcan_rfid_reader import AdvanNetClient, AdvanNetEventStream, parse_event_xml

HOST = "192.168.123.2"
DEVICE = "VUL-TITANIUM-4PG-4e4e"

client = AdvanNetClient(HOST, "admin", "admin")
client.start(DEVICE)

with AdvanNetEventStream(HOST) as stream:
    for xml_msg in stream.messages():
        for tag in parse_event_xml(xml_msg):
            print(tag.epc, tag.rssi, tag.antenna)

client.stop(DEVICE)
```

---

## Using `rfid_scanner_server.py`

### Prerequisites

- Robot and RFID reader on the same Ethernet segment (e.g. `eth0` / `192.168.123.x`)
- Reader powered on and reachable: `ping 192.168.123.2`
- Python packages: `flask`, `requests` (already present on the Unitree image)

### Start the server

```bash
cd /home/unitree/churan_alireza
python3 rfid_scanner_server.py
```

On startup the server will:

1. Connect to the reader via REST (port 3161) and send **start**
2. Open a background thread on TCP port **3177** for live tag events
3. Serve the web UI on **port 8765** (all interfaces: `0.0.0.0`)

### Open the dashboard

**On the robot:**

```
http://localhost:8765
```

**From another machine on the same Wi‑Fi** (robot `wlan0` IP may vary):

```
http://10.42.200.240:8765
```

**From the robot Ethernet subnet:**

```
http://192.168.123.18:8765
```

The startup log prints all detected URLs for your host.

### Remote access (SSH tunnel)

If you are off-network or Wi‑Fi client isolation blocks direct access:

```bash
ssh unitree@10.42.200.240 -L 8765:localhost:8765
```

Then open `http://localhost:8765` in your browser.

---

## Web Dashboard Behavior

The page polls `/api/tags` every **1 second** and shows:

| Column | Description |
|--------|-------------|
| Status | ● In range / ○ Out of range |
| Name | Short label derived from EPC (e.g. `Tag …6EA3715C`) |
| ID (EPC) | Full hex Electronic Product Code |
| Signal (dBm) | RSSI — color-coded when in range |
| Antenna | Antenna port number |
| Frequency | Carrier frequency in MHz |
| Reads | Cumulative read count for this session |
| Last seen | UTC timestamp of most recent read |

### Tag list rules

- **Tags are never deleted** once discovered.
- Tags with a read within the last **5 seconds** are **in range** (top of list, sorted by signal strength).
- Tags not seen for more than 5 seconds move to the **bottom**, grayed out, marked **Out of range**.
- The header shows e.g. `2 in range · 5 discovered`.

---

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `VULCAN_READER_HOST` | `192.168.123.2` | Reader IP address |
| `VULCAN_READER_USER` | `admin` | Digest auth username |
| `VULCAN_READER_PASS` | `admin` | Digest auth password |
| `VULCAN_REST_PORT` | `3161` | REST API port |
| `VULCAN_EVENT_PORT` | `3177` | Event stream port |
| `RFID_WEB_PORT` | `8765` | Web server port |
| `RFID_STALE_SECONDS` | `5` | Seconds without a read before a tag is "out of range" |
| `RFID_STREAM_DEAD_SECONDS` | `45` | Seconds without stream activity before UI shows stream unhealthy |

Example:

```bash
export VULCAN_READER_HOST=192.168.123.2
export RFID_STALE_SECONDS=10
python3 rfid_scanner_server.py
```

---

## HTTP API

For full API documentation (Python module + REST endpoints), see **[RFID_API.md](RFID_API.md)**.

### `GET /`

Returns the HTML dashboard (`templates/rfid_scanner.html`).

### `GET /api/v1/tags` and `GET /api/tags`

Returns JSON with the current tag list and scanner status.

**Example response:**

```json
{
  "tags": [
    {
      "id": "e2801191a50300664a20e848",
      "name": "Tag …4A20E848",
      "epc": "e2801191a50300664a20e848",
      "rssi_dbm": -64,
      "antenna": 1,
      "frequency_khz": 902750,
      "read_count": 42,
      "first_seen": 1781204415.9,
      "first_seen_iso": "19:00:15 UTC",
      "last_seen": 1781204500.1,
      "last_seen_iso": "19:01:40 UTC",
      "phase": "20",
      "device_id": "VUL-TITANIUM-4PG-4e4e",
      "in_range": true
    }
  ],
  "count": 1,
  "active_count": 1,
  "stale_seconds": 5,
  "scanner": {
    "connected": true,
    "error": null,
    "device_id": "VUL-TITANIUM-4PG-4e4e",
    "last_activity": 1781204500.5
  },
  "reader_host": "192.168.123.2",
  "updated_at": "2026-06-11T19:01:41.170144+00:00"
}
```

**Integration example (Python):**

```python
import requests

data = requests.get("http://localhost:8765/api/tags", timeout=2).json()
for tag in data["tags"]:
    if tag["in_range"]:
        print(tag["epc"], tag["rssi_dbm"])
```

---

## Architecture

```
┌─────────────────┐     REST :3161 (Digest)      ┌──────────────────────┐
│  rfid_scanner   │ ─── start / stop / devices ─▶│  Vulcan RFID Reader  │
│  _server.py     │                              │  (Keonn AdvanNet)    │
│                 │◀── TCP :3177 (ADVANNET XML) ───│                      │
│  Flask :8765    │     tag events, no auth       │  192.168.123.2       │
└────────┬────────┘                              └──────────────────────┘
         │
         │ HTTP /api/tags (every 1s)
         ▼
┌─────────────────┐
│  Web browser    │
│  (any machine)  │
└─────────────────┘
```

1. **Control plane** — `AdvanNetClient` sends REST commands to start inventory.
2. **Data plane** — `AdvanNetEventStream` parses ADVANNET-framed XML from port 3177.
3. **State** — an in-memory dict keyed by EPC; the API annotates each tag with `in_range`.
4. **UI** — static HTML polls JSON; no WebSocket required on the client side.

### ADVANNET TCP framing (port 3177)

Each message looks like:

```
ADVANNET/1.1
Content-Length: 1234
Content-Type: text/xml

<?xml version="1.0" encoding="UTF-8"?>
<inventory>...</inventory>
```

Keepalive frames with `Content-Length: 0` are sent periodically. The server treats any received frame as stream activity.

---

## Troubleshooting

### `No route to host` / reader unreachable

- Confirm reader power and Ethernet cable to the robot `eth0` port.
- Ping the reader: `ping 192.168.123.2`
- Robot should have an address on `192.168.123.x` (e.g. `192.168.123.18`).

### `401 Unauthorized` on REST API

- Use **Digest** auth, not Basic.
- Default credentials: `admin` / `admin` (may have been changed via the reader web UI).

### Mercury API / `tmr://` fails

Expected. Use AdvanNet (this project) instead. Port 5084 (LLRP) is not open on this hardware.

### Web dashboard shows "Stream error — retrying"

- Reader may be offline or port 3177 blocked.
- Restart the server after the reader is back online.
- Check `scanner.error` in `/api/tags` JSON for details.

### SSH tunnel to reader dashboard hangs on "Loading..."

The factory UI on port 80 also needs WebSocket port **11987**:

```bash
ssh -L 8080:192.168.123.2:80 -L 11987:192.168.123.2:11987 unitree@<robot-ip>
```

This project's dashboard does **not** need port 11987 — only 8765 on the robot.

### Port 8765 already in use

```bash
export RFID_WEB_PORT=9000
python3 rfid_scanner_server.py
```

### Reader SSH access

SSH user is `keonn`. Password is device-specific; connect to get a challenge and email MAC + challenge to `support@atlasrfidstore.com`.

---

## Robot Network Reference

Typical Unitree network layout:

| Interface | Example IP | Notes |
|-----------|------------|-------|
| `eth0` | `192.168.123.18/24` | Robot ↔ reader / onboard devices |
| `wlan0` | `10.42.200.240/22` | Wi‑Fi — use this IP for external laptop access |

The RFID reader is usually at **`192.168.123.2`** on `eth0`.

---

## Further Reading

- [Keonn AdvanNet REST API wiki](https://wiki.keonn.com/software/advannet/development/rest-api-development/)
- [Keonn C# REST examples (TCP 3177)](https://github.com/Keonn-Technologies/CSharpRestExamples)
- [Vulcan RFID Titanium product page](https://www.vulcanrfid.com/titanium-rfid-reader)
