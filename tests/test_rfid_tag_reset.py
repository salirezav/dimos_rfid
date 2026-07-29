from __future__ import annotations

import sys
from pathlib import Path


RFID_SCANNER_DIR = Path(__file__).parents[1] / "rfid scanner python"
sys.path.insert(0, str(RFID_SCANNER_DIR))

from rfid_service import RfidScanner  # noqa: E402
from vulcan_rfid_reader import TagRead  # noqa: E402


def _read(epc: str, reader_count: int) -> TagRead:
    return TagRead(epc=epc, raw_props={"READ_COUNT": str(reader_count)})


def test_clear_tag_cache_removes_epcs_and_rebases_reader_count() -> None:
    scanner = RfidScanner()
    scanner._apply_read(_read("epc-1", 40))
    scanner._apply_read(_read("epc-1", 41))
    assert scanner.get_tag("epc-1")["read_count"] == 2

    assert scanner.clear_tag_cache() == 1
    assert scanner.get_tags() == []

    scanner._apply_read(_read("epc-1", 42))
    assert scanner.get_tag("epc-1")["read_count"] == 1

    scanner._apply_read(_read("epc-1", 43))
    assert scanner.get_tag("epc-1")["read_count"] == 2


def test_clear_tags_remains_a_backward_compatible_alias() -> None:
    scanner = RfidScanner()
    scanner._apply_read(_read("epc-1", 10))

    assert scanner.clear_tags() == 1
    assert scanner.get_tags() == []


def test_api_payload_reports_empty_discovered_and_active_counts_after_reset() -> None:
    scanner = RfidScanner()
    scanner._apply_read(_read("epc-1", 10))
    scanner.clear_tag_cache()

    payload = scanner.to_api_payload()
    assert payload["tags"] == []
    assert payload["count"] == 0
    assert payload["discovered_count"] == 0
    assert payload["active_count"] == 0
