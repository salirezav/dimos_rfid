# Vulcan RFID API Reference

This document describes how to read RFID tags programmatically using this project. There are two interfaces:

1. **Python API** — import `rfid_service` in your own scripts or ROS nodes
2. **HTTP API** — call REST endpoints while `rfid_scanner_server.py` is running

Both use the same underlying `RfidScanner` service in `rfid_service.py`.

---

## Quick Start

### Python (in-process)

```python
from rfid_service import RfidScanner
import time

with RfidScanner() as scanner:
    time.sleep(3)  # let tags accumulate
    for tag in scanner.get_active_tags():
        print(tag["epc"], tag["rssi_dbm"], "dBm")
```

### HTTP (remote)

```bash
# Start the server on the robot
python3 rfid_scanner_server.py

# From any machine on the network
curl http://10.42.200.240:8765/api/v1/tags/active
```

---

## Python API (`rfid_service.py`)

### `RfidScanner`

Main class. Connects to the reader, starts inventory, and tracks discovered tags in memory.

#### Constructor

```python
from rfid_service import RfidScanner, ScannerConfig

scanner = RfidScanner()  # uses env vars / defaults

# Or explicit config:
scanner = RfidScanner(ScannerConfig(
    host="192.168.123.2",
    user="admin",
    password="admin",
    stale_seconds=5.0,
))
```

| `ScannerConfig` field | Env variable | Default |
|-----------------------|--------------|---------|
| `host` | `VULCAN_READER_HOST` | `192.168.123.2` |
| `user` | `VULCAN_READER_USER` | `admin` |
| `password` | `VULCAN_READER_PASS` | `admin` |
| `stale_seconds` | `RFID_STALE_SECONDS` | `5` |
| `stream_dead_seconds` | `RFID_STREAM_DEAD_SECONDS` | `45` |

#### Lifecycle methods

| Method | Description |
|--------|-------------|
| `connect()` | Discover reader device via REST. Returns device ID string. |
| `start()` | Start reader inventory + background TCP event listener. |
| `stop()` | Stop listener and send reader stop command. |
| `is_running()` | `True` if inventory is active. |

Context manager (recommended):

```python
with RfidScanner() as scanner:
    # connect() + start() on enter, stop() on exit
    ...
```

#### Reading methods

| Method | Returns | Description |
|--------|---------|-------------|
| `get_tags(active_only=False)` | `list[dict]` | All discovered tags; in-range first, then out-of-range. |
| `get_active_tags()` | `list[dict]` | Only tags seen within `stale_seconds`. |
| `get_tag(epc)` | `dict \| None` | Single tag by EPC hex string. |
| `poll_inventory()` | `list[dict]` | One-shot REST inventory snapshot from reader. |
| `clear_tags()` | `int` | Clear in-memory tag list. Returns count removed. |
| `get_status()` | `dict` | Reader/stream health and tag counts. |
| `to_api_payload()` | `dict` | Full payload (same shape as HTTP `/api/v1/tags`). |

#### Callbacks

```python
def on_read(tag):
    print("New read:", tag["epc"], tag["rssi_dbm"])

scanner = RfidScanner()
scanner.on_tag(on_read)
scanner.connect()
scanner.start()
```

#### Tag dict schema

Each tag returned by the reading methods:

```python
{
    "id": "e2801191a50300664a20e848",       # same as epc
    "name": "Tag …4A20E848",                # short display name
    "epc": "e2801191a50300664a20e848",      # hex EPC
    "rssi_dbm": -64,                        # signal strength (dBm)
    "antenna": 1,
    "frequency_khz": 902750,
    "read_count": 12,                       # reads this session
    "first_seen": 1781204415.9,             # unix timestamp
    "first_seen_iso": "19:00:15 UTC",
    "last_seen": 1781204500.1,
    "last_seen_iso": "19:01:40 UTC",
    "phase": "20",
    "device_id": "VUL-TITANIUM-4PG-4e4e",
    "in_range": True                        # seen within stale_seconds
}
```

Tags are **never removed** automatically. `in_range` becomes `False` when no read arrives for `stale_seconds`.

---

### `get_scanner()` singleton

Used by the HTTP server. Returns a shared `RfidScanner` for the process:

```python
from rfid_service import get_scanner

scanner = get_scanner(autostart=True)
tags = scanner.get_active_tags()
```

---

## HTTP API

**Base URL:** `http://<robot-ip>:8765/api/v1`

The server must be running:

```bash
python3 rfid_scanner_server.py
```

All responses are JSON. Successful responses include `"ok": true`.

### Endpoints

#### `GET /api/v1/health`

Health check.

```bash
curl http://localhost:8765/api/v1/health
```

```json
{"ok": true, "service": "vulcan-rfid-api", "version": "1"}
```

---

#### `GET /api/v1/reader/status`

Reader connection, stream health, and tag counts.

```bash
curl http://localhost:8765/api/v1/reader/status
```

```json
{
  "ok": true,
  "reader_host": "192.168.123.2",
  "device_id": "VUL-TITANIUM-4PG-4e4e",
  "reader_started": true,
  "stream": {
    "connected": true,
    "error": null,
    "device_id": "VUL-TITANIUM-4PG-4e4e",
    "last_activity": 1781204500.5
  },
  "tag_count": 3,
  "active_count": 1,
  "stale_seconds": 5.0,
  "updated_at": "2026-06-11T19:01:41.170144+00:00"
}
```

---

#### `POST /api/v1/reader/start`

Start reader inventory and event stream.

```bash
curl -X POST http://localhost:8765/api/v1/reader/start
```

---

#### `POST /api/v1/reader/stop`

Stop reader inventory.

```bash
curl -X POST http://localhost:8765/api/v1/reader/stop
```

---

#### `GET /api/v1/tags`

All discovered tags (in-range first, then out-of-range). **Auto-starts** the reader if not running.

```bash
curl http://localhost:8765/api/v1/tags
```

```json
{
  "ok": true,
  "tags": [ { "...": "..." } ],
  "count": 3,
  "active_count": 1,
  "stale_seconds": 5.0,
  "scanner": { "connected": true, "error": null },
  "reader_host": "192.168.123.2",
  "device_id": "VUL-TITANIUM-4PG-4e4e",
  "reader_started": true,
  "updated_at": "2026-06-11T19:01:41.170144+00:00"
}
```

---

#### `GET /api/v1/tags/active`

Only tags currently in range.

```bash
curl http://localhost:8765/api/v1/tags/active
```

```json
{
  "ok": true,
  "tags": [ { "epc": "...", "rssi_dbm": -64, "in_range": true } ],
  "count": 1,
  "reader_host": "192.168.123.2",
  "updated_at": "2026-06-11T19:01:41.170144+00:00"
}
```

---

#### `GET /api/v1/tags/<epc>`

Look up one tag by EPC.

```bash
curl http://localhost:8765/api/v1/tags/e2801191a50300664a20e848
```

```json
{
  "ok": true,
  "tag": { "epc": "e2801191a50300664a20e848", "rssi_dbm": -64, "in_range": true }
}
```

`404` if the tag has never been seen.

---

#### `POST /api/v1/tags/clear`

Clear all discovered tags from server memory.

```bash
curl -X POST http://localhost:8765/api/v1/tags/clear
```

```json
{"ok": true, "removed": 3}
```

---

#### `GET /api/v1/inventory`

One-shot synchronous inventory from the reader (REST snapshot). Starts the reader if needed.

```bash
curl http://localhost:8765/api/v1/inventory
```

---

### Legacy endpoint

`GET /api/tags` — same data as `/api/v1/tags` (without the `"ok"` wrapper). Kept for the web dashboard.

---

## Usage Examples

### ROS 2 node (polling)

```python
import rclpy
from rclpy.node import Node
import requests

class RfidNode(Node):
    def __init__(self):
        super().__init__("rfid_poller")
        self.api = "http://localhost:8765/api/v1/tags/active"
        self.create_timer(1.0, self.poll)

    def poll(self):
        try:
            data = requests.get(self.api, timeout=1).json()
            for tag in data.get("tags", []):
                self.get_logger().info(f"{tag['epc']} {tag['rssi_dbm']} dBm")
        except Exception as e:
            self.get_logger().warn(str(e))
```

### JavaScript (browser / Node)

```javascript
async function getActiveTags() {
  const res = await fetch("http://10.42.200.240:8765/api/v1/tags/active");
  const data = await res.json();
  return data.tags;
}

setInterval(async () => {
  const tags = await getActiveTags();
  console.log(tags.map(t => `${t.epc} @ ${t.rssi_dbm} dBm`));
}, 1000);
```

### Python script (no HTTP server)

```python
#!/usr/bin/env python3
"""Standalone tag reader — no web server required."""
import time
from rfid_service import RfidScanner

def main():
    with RfidScanner() as scanner:
        print("Scanning... Ctrl+C to stop")
        seen = set()
        while True:
            for tag in scanner.get_active_tags():
                if tag["epc"] not in seen:
                    seen.add(tag["epc"])
                    print(f"NEW  {tag['name']}  {tag['epc']}  {tag['rssi_dbm']} dBm")
            time.sleep(0.5)

if __name__ == "__main__":
    main()
```

### Check if a specific tag is present

```python
from rfid_service import RfidScanner
import time

TARGET = "e2801191a50300664a20e848"

with RfidScanner() as scanner:
    for _ in range(20):
        tag = scanner.get_tag(TARGET)
        if tag and tag["in_range"]:
            print("Tag found:", tag["rssi_dbm"], "dBm")
            break
        time.sleep(0.5)
    else:
        print("Tag not in range")
```

---

## Error Responses

HTTP errors return:

```json
{"ok": false, "error": "description of the problem"}
```

| Status | Meaning |
|--------|---------|
| `404` | Tag EPC not found |
| `503` | Reader unreachable or start failed |

---

## Architecture

```
Your code                rfid_service.py              Vulcan reader
─────────                ───────────────              ─────────────
Python API    ────────▶  RfidScanner  ── REST :3161 ─▶ start/stop
                         │              ◀─ TCP :3177 ── tag events
HTTP request  ────────▶  get_scanner()
  /api/v1/*              │
                         ▼
                    in-memory tag registry
```

- **Control:** REST on port 3161 (HTTP Digest auth, handled internally)
- **Data:** TCP event stream on port 3177 (ADVANNET XML, no auth)
- **Your API:** Python methods or HTTP JSON on port 8765

---

## See Also

- [RFID_SCANNER.md](RFID_SCANNER.md) — hardware discovery, web dashboard, troubleshooting
- [vulcan_rfid_reader.py](vulcan_rfid_reader.py) — low-level AdvanNet protocol client
- [rfid_scanner_server.py](rfid_scanner_server.py) — web UI + HTTP API server
