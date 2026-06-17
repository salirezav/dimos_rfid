#!/usr/bin/env python3
"""
Web UI and HTTP API for live Vulcan RFID tag scanning.

Run:
  python3 rfid_scanner_server.py
  Web UI:  http://<robot-ip>:8765
  API:     http://<robot-ip>:8765/api/v1/...
"""

from __future__ import annotations

import atexit
import os
import socket

from flask import Flask, jsonify, render_template

from rfid_service import get_scanner, reset_scanner

WEB_PORT = int(os.environ.get("RFID_WEB_PORT", "8765"))

app = Flask(__name__)


def _error(message: str, status: int = 400):
    return jsonify({"ok": False, "error": message}), status


# ------------------------------------------------------------------
# Web UI
# ------------------------------------------------------------------

@app.route("/")
def index():
    return render_template("rfid_scanner.html")


# ------------------------------------------------------------------
# Legacy endpoint (web UI)
# ------------------------------------------------------------------

@app.route("/api/tags")
def api_tags_legacy():
    scanner = get_scanner(autostart=True)
    return jsonify(scanner.to_api_payload())


# ------------------------------------------------------------------
# HTTP API v1
# ------------------------------------------------------------------

@app.route("/api/v1/health")
def api_health():
    return jsonify({"ok": True, "service": "vulcan-rfid-api", "version": "1"})


@app.route("/api/v1/reader/status")
def api_reader_status():
    scanner = get_scanner()
    return jsonify({"ok": True, **scanner.get_status()})


@app.route("/api/v1/reader/start", methods=["POST"])
def api_reader_start():
    scanner = get_scanner()
    try:
        if not scanner.device_id:
            scanner.connect()
        scanner.start()
        return jsonify({"ok": True, **scanner.get_status()})
    except Exception as exc:
        return _error(str(exc), 503)


@app.route("/api/v1/reader/stop", methods=["POST"])
def api_reader_stop():
    scanner = get_scanner()
    scanner.stop()
    return jsonify({"ok": True, **scanner.get_status()})


@app.route("/api/v1/tags")
def api_tags():
    scanner = get_scanner(autostart=True)
    return jsonify({"ok": True, **scanner.to_api_payload()})


@app.route("/api/v1/tags/active")
def api_tags_active():
    scanner = get_scanner(autostart=True)
    tags = scanner.get_active_tags()
    return jsonify({
        "ok": True,
        "tags": tags,
        "count": len(tags),
        "reader_host": scanner.config.host,
        "updated_at": scanner.get_status()["updated_at"],
    })


@app.route("/api/v1/tags/<epc>")
def api_tag_by_epc(epc: str):
    scanner = get_scanner(autostart=True)
    tag = scanner.get_tag(epc)
    if tag is None:
        return _error(f"Tag not found: {epc}", 404)
    return jsonify({"ok": True, "tag": tag})


@app.route("/api/v1/tags/clear", methods=["POST"])
def api_tags_clear():
    scanner = get_scanner()
    removed = scanner.clear_tags()
    return jsonify({"ok": True, "removed": removed})


@app.route("/api/v1/inventory")
def api_inventory():
    """One-shot synchronous inventory poll from the reader."""
    scanner = get_scanner()
    try:
        if not scanner.device_id:
            scanner.connect()
        if not scanner.is_running():
            scanner.start()
        tags = scanner.poll_inventory()
        return jsonify({
            "ok": True,
            "tags": tags,
            "count": len(tags),
            "updated_at": scanner.get_status()["updated_at"],
        })
    except Exception as exc:
        return _error(str(exc), 503)


# ------------------------------------------------------------------
# Server startup
# ------------------------------------------------------------------

def _local_urls(port: int) -> list[str]:
    urls: list[str] = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                urls.append(f"http://{ip}:{port}")
    except OSError:
        pass
    seen: set[str] = set()
    for url in urls:
        seen.add(url.split("://", 1)[1].rsplit(":", 1)[0])
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for probe in ("10.42.200.1", "192.168.123.1", "8.8.8.8"):
            try:
                s.connect((probe, 80))
                ip = s.getsockname()[0]
                if ip not in seen:
                    urls.append(f"http://{ip}:{port}")
                    seen.add(ip)
            except OSError:
                continue
        s.close()
    except OSError:
        pass
    return sorted(set(urls))


def main() -> None:
    scanner = get_scanner(autostart=True)
    atexit.register(reset_scanner)

    print(f"RFID scanner listening on 0.0.0.0:{WEB_PORT}")
    print(f"Reader: {scanner.config.host}  device: {scanner.get_status().get('device_id')}")
    print("Web UI:")
    for url in _local_urls(WEB_PORT):
        print(f"  {url}")
    print("API base:")
    for url in _local_urls(WEB_PORT):
        print(f"  {url}/api/v1/")
    print("Remote access via SSH tunnel:")
    print(f"  ssh unitree@<robot-ip> -L {WEB_PORT}:localhost:{WEB_PORT}")
    app.run(host="0.0.0.0", port=WEB_PORT, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
