#!/usr/bin/env python3
"""
Vulcan RFID Titanium reader client (Keonn AdvanNet platform).

Protocol summary (discovered on 192.168.123.2):
  - REST API (control):  TCP 3161, HTTP Digest auth
  - Event stream (tags): TCP 3177, ADVANNET/1.1 framed XML (no auth)
  - Web dashboard:       TCP 80, Digest auth; WebSocket on 11987
  - Mercury/LLRP:        NOT exposed (port 5084 closed) — use AdvanNet instead

Usage:
  python3 vulcan_rfid_reader.py diagnose
  python3 vulcan_rfid_reader.py stream --duration 30
  python3 vulcan_rfid_reader.py inventory
"""

from __future__ import annotations

import argparse
import os
import re
import socket
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Iterator, Optional

import requests
from requests.auth import HTTPDigestAuth

DEFAULT_HOST = os.environ.get("VULCAN_READER_HOST", "192.168.123.2")
DEFAULT_USER = os.environ.get("VULCAN_READER_USER", "admin")
DEFAULT_PASS = os.environ.get("VULCAN_READER_PASS", "admin")
REST_PORT = int(os.environ.get("VULCAN_REST_PORT", "3161"))
EVENT_PORT = int(os.environ.get("VULCAN_EVENT_PORT", "3177"))


@dataclass
class TagRead:
    epc: str
    rssi: Optional[int] = None
    antenna: Optional[int] = None
    frequency_khz: Optional[int] = None
    timestamp_ms: Optional[int] = None
    raw_props: dict[str, str] = field(default_factory=dict)


class AdvanNetClient:
    """REST control plane for Keonn AdvanNet readers."""

    def __init__(self, host: str, user: str, password: str, rest_port: int = REST_PORT):
        self.base_url = f"http://{host}:{rest_port}"
        self.session = requests.Session()
        self.session.auth = HTTPDigestAuth(user, password)
        self.session.headers["User-Agent"] = "VulcanRFID-Python/1.0"

    def _get(self, path: str) -> str:
        url = f"{self.base_url}{path}"
        resp = self.session.get(url, timeout=10)
        resp.raise_for_status()
        return resp.text

    def status(self) -> ET.Element:
        return ET.fromstring(self._get("/status"))

    def devices(self) -> list[dict]:
        root = ET.fromstring(self._get("/devices"))
        devices = []
        for dev in root.findall(".//device"):
            devices.append(
                {
                    "id": _text(dev, "id"),
                    "ip": _text(dev, "ip"),
                    "mac": _text(dev, "mac"),
                    "status": _text(dev, "status"),
                    "active_device_mode": _text(dev, "activeDeviceMode"),
                    "active_read_mode": _text(dev, "activeReadMode"),
                    "rf_module": _text(dev, "rf-module"),
                }
            )
        return devices

    def start(self, device_id: str) -> None:
        self._get(f"/devices/{device_id}/start")

    def stop(self, device_id: str) -> None:
        self._get(f"/devices/{device_id}/stop")

    def inventory(self, device_id: str) -> list[TagRead]:
        root = ET.fromstring(self._get(f"/devices/{device_id}/inventory"))
        return _parse_tag_reads(root)


class AdvanNetEventStream:
    """TCP event stream on port 3177 (ADVANNET/1.1 framed XML)."""

    def __init__(self, host: str, event_port: int = EVENT_PORT):
        self.host = host
        self.event_port = event_port
        self._sock: Optional[socket.socket] = None

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(30)
        self._sock.connect((self.host, self.event_port))

    def close(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def __enter__(self) -> "AdvanNetEventStream":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def _read_line(self) -> Optional[str]:
        assert self._sock is not None
        buf = bytearray()
        while True:
            chunk = self._sock.recv(1)
            if not chunk:
                return None
            if chunk == b"\n":
                return buf.decode("utf-8", errors="replace").strip()
            if chunk != b"\r":
                buf.extend(chunk)

    def _read_message(self) -> Optional[str]:
        assert self._sock is not None
        while True:
            line = self._read_line()
            if line is None:
                return None
            if line.startswith("ADVANNET"):
                break

        content_length = 0
        while True:
            line = self._read_line()
            if line is None:
                return None
            if line.lower().startswith("content-length:"):
                content_length = int(line.split(":", 1)[1].strip())
            elif line == "":
                break

        if content_length <= 0:
            return ""

        data = bytearray()
        while len(data) < content_length:
            chunk = self._sock.recv(content_length - len(data))
            if not chunk:
                break
            data.extend(chunk)
        return bytes(data).decode("utf-8", errors="replace")

    def messages(self) -> Iterator[str]:
        while True:
            msg = self._read_message()
            if msg is None:
                break
            yield msg


def _text(parent: ET.Element, tag: str) -> str:
    el = parent.find(tag)
    return (el.text or "").strip() if el is not None else ""


def _parse_props(props_el: Optional[ET.Element]) -> dict[str, str]:
    result: dict[str, str] = {}
    if props_el is None:
        return result
    for prop in props_el.findall("prop"):
        text = (prop.text or "").strip()
        if ":" in text:
            key, value = text.split(":", 1)
            result[key] = value
    return result


def _parse_tag_reads(root: ET.Element) -> list[TagRead]:
    reads: list[TagRead] = []
    for item in root.findall(".//item"):
        epc = _text(item, "epc")
        if not epc:
            epc_el = item.find(".//hexepc")
            epc = (epc_el.text or "").strip() if epc_el is not None else ""
        if not epc:
            continue
        data_el = item.find("data")
        props = _parse_props(data_el.find("props") if data_el is not None else None)
        reads.append(
            TagRead(
                epc=epc,
                rssi=_safe_int(props.get("RSSI")),
                antenna=_safe_int(props.get("ANTENNA_PORT")),
                frequency_khz=_safe_int(props.get("FREQ")),
                timestamp_ms=_safe_int(_text(item, "ts") or props.get("TIME_STAMP")),
                raw_props=props,
            )
        )
    return reads


def parse_event_xml(xml_text: str) -> list[TagRead]:
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError:
        return []
    return _parse_tag_reads(root)


def _safe_int(value: Optional[str]) -> Optional[int]:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except ValueError:
        return None


def check_reachability(host: str, rest_port: int, event_port: int) -> dict:
    results = {"ping": False, "rest_port": False, "event_port": False}
    import subprocess

    ping = subprocess.run(
        ["ping", "-c", "1", "-W", "2", host],
        capture_output=True,
    )
    results["ping"] = ping.returncode == 0

    for name, port in [("rest_port", rest_port), ("event_port", event_port)]:
        try:
            s = socket.create_connection((host, port), timeout=3)
            s.close()
            results[name] = True
        except OSError:
            results[name] = False
    return results


def cmd_diagnose(args: argparse.Namespace) -> int:
    print(f"=== Vulcan RFID Reader Diagnostic ===")
    print(f"Host: {args.host}")
    reach = check_reachability(args.host, REST_PORT, EVENT_PORT)
    print(f"  Ping:       {'OK' if reach['ping'] else 'FAIL'}")
    print(f"  REST :{REST_PORT}:  {'OK' if reach['rest_port'] else 'FAIL'}")
    print(f"  Events:{EVENT_PORT}: {'OK' if reach['event_port'] else 'FAIL'}")

    if not reach["rest_port"]:
        print("\nREST port unreachable — check cabling and IP.")
        return 1

    client = AdvanNetClient(args.host, args.user, args.password)
    try:
        status = client.status()
        version = _text(status.find(".//version"), "version")
        uptime = _text(status, "uptime") or _text(status.find(".//data"), "uptime")
        print(f"\nAdvanNet version: {version}")
        if uptime:
            print(f"Uptime: {uptime}")

        devices = client.devices()
        print(f"\nDevices ({len(devices)}):")
        for d in devices:
            print(f"  ID:     {d['id']}")
            print(f"  MAC:    {d['mac']}")
            print(f"  Status: {d['status']} | Mode: {d['active_device_mode']}/{d['active_read_mode']}")
            print(f"  RF:     {d['rf_module']}")
    except requests.HTTPError as exc:
        print(f"\nREST auth/API error: {exc}")
        print("Tip: credentials use HTTP Digest (not Basic). Default is admin:admin.")
        return 1

    print("\nProtocol: Keonn AdvanNet (NOT Mercury/LLRP)")
    print(f"  Control:  http://{args.host}:{REST_PORT}/  (Digest auth)")
    print(f"  Stream:   tcp://{args.host}:{EVENT_PORT}/  (ADVANNET/1.1 XML, no auth)")
    return 0


def cmd_inventory(args: argparse.Namespace) -> int:
    client = AdvanNetClient(args.host, args.user, args.password)
    devices = client.devices()
    if not devices:
        print("No devices found.")
        return 1

    device_id = args.device_id or devices[0]["id"]
    if args.start:
        print(f"Starting reader {device_id}...")
        client.start(device_id)

    print(f"Polling inventory for {device_id}...")
    reads = client.inventory(device_id)
    if reads:
        for tag in reads:
            _print_tag(tag)
    else:
        print("No tags in current inventory buffer.")
        print("Use 'stream' mode for live reads, or place tags near antennas.")

    if args.start:
        client.stop(device_id)
    return 0


def cmd_stream(args: argparse.Namespace) -> int:
    client = AdvanNetClient(args.host, args.user, args.password)
    devices = client.devices()
    if not devices:
        print("No devices found.")
        return 1

    device_id = args.device_id or devices[0]["id"]
    started_here = False

    if args.start:
        print(f"Starting reader {device_id}...")
        client.start(device_id)
        started_here = True

    seen: set[str] = set()
    deadline = time.time() + args.duration
    print(f"Streaming tag events from {args.host}:{EVENT_PORT} for {args.duration:.0f}s...")
    print("(Ctrl+C to stop early)\n")

    try:
        with AdvanNetEventStream(args.host) as stream:
            for xml_msg in stream.messages():
                if time.time() >= deadline:
                    break
                for tag in parse_event_xml(xml_msg):
                    if args.unique and tag.epc in seen:
                        continue
                    seen.add(tag.epc)
                    _print_tag(tag)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if started_here:
            print(f"\nStopping reader {device_id}...")
            client.stop(device_id)

    print(f"\nTotal unique tags: {len(seen)}")
    return 0


def _print_tag(tag: TagRead) -> None:
    parts = [f"EPC={tag.epc}"]
    if tag.rssi is not None:
        parts.append(f"RSSI={tag.rssi}")
    if tag.antenna is not None:
        parts.append(f"ANT={tag.antenna}")
    if tag.frequency_khz is not None:
        parts.append(f"FREQ={tag.frequency_khz}")
    print("  ".join(parts))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Vulcan RFID Titanium reader client (Keonn AdvanNet API)"
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="Reader IP address")
    parser.add_argument("--user", default=DEFAULT_USER, help="Digest auth username")
    parser.add_argument("--password", default=DEFAULT_PASS, help="Digest auth password")
    parser.add_argument("--device-id", default=None, help="Device ID (auto-detected if omitted)")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("diagnose", help="Check connectivity and reader status")

    inv = sub.add_parser("inventory", help="One-shot inventory via REST")
    inv.add_argument("--start", action="store_true", help="Start reader before inventory")

    stream = sub.add_parser("stream", help="Stream live tag reads from TCP 3177")
    stream.add_argument("--duration", type=float, default=30.0, help="Seconds to stream")
    stream.add_argument("--start", action="store_true", default=True, help="Start reader (default: on)")
    stream.add_argument("--no-start", action="store_false", dest="start", help="Do not send start command")
    stream.add_argument("--unique", action="store_true", default=True, help="Only print each EPC once")
    stream.add_argument("--all", action="store_false", dest="unique", help="Print every read event")

    args = parser.parse_args()

    if args.command == "diagnose":
        return cmd_diagnose(args)
    if args.command == "inventory":
        return cmd_inventory(args)
    if args.command == "stream":
        return cmd_stream(args)
    return 1


if __name__ == "__main__":
    sys.exit(main())
