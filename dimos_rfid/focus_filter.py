"""Live-editable EPC focus filter (same UX as experimental rfid_focus.txt)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class FocusFilter:
    """Select which EPCs to localize; empty patterns = all tags.

    Patterns match case-insensitively as substrings, so a short suffix like
    ``8f`` focuses the full EPC ending in ``…8f``. Edit ``focus_file`` while
    running; changes apply on the next tag batch.
    """

    config_patterns: list[str] = field(default_factory=list)
    focus_file: str = ""
    _file_patterns: list[str] = field(default_factory=list)
    _file_mtime: float | None = None
    _rpc_patterns: list[str] | None = None

    def set_rpc_focus(self, patterns: list[str] | None) -> None:
        """Override focus from an RPC. ``None`` clears the override."""
        self._rpc_patterns = None if patterns is None else [p.strip() for p in patterns if p.strip()]

    def _parse_file(self, path: Path) -> list[str]:
        patterns: list[str] = []
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            return []
        for line in text.splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            for part in line.split(","):
                part = part.strip()
                if part and not part.startswith("#"):
                    patterns.append(part)
        return patterns

    def _reload_file_if_needed(self) -> None:
        if not self.focus_file:
            self._file_patterns = []
            self._file_mtime = None
            return
        path = Path(self.focus_file)
        if not path.is_file():
            self._file_patterns = []
            self._file_mtime = None
            return
        try:
            mtime = path.stat().st_mtime
        except OSError:
            return
        if self._file_mtime is not None and mtime == self._file_mtime:
            return
        self._file_mtime = mtime
        self._file_patterns = self._parse_file(path)

    def patterns(self) -> list[str]:
        self._reload_file_if_needed()
        if self._rpc_patterns is not None:
            return list(self._rpc_patterns)
        out: list[str] = []
        seen: set[str] = set()
        for p in [*self.config_patterns, *self._file_patterns]:
            key = p.lower()
            if key not in seen:
                seen.add(key)
                out.append(p)
        return out

    @property
    def active(self) -> bool:
        return bool(self.patterns())

    def matches(self, epc: str) -> bool:
        """True if this EPC should be localized (always True when filter inactive)."""
        pats = self.patterns()
        if not pats:
            return True
        epc_l = epc.lower()
        return any(p.lower() in epc_l for p in pats)


def ensure_focus_file(path: str) -> None:
    """Create an empty focus file with usage comments if it does not exist."""
    p = Path(path)
    if p.is_file():
        return
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(
        "# RFID focus list — one EPC (or suffix) per line.\n"
        "# Empty file = localize ALL in-range tags. Edit while running; changes apply next poll.\n"
        "# Examples:\n"
        "#   8f\n"
        "#   E280116060000203B5A908F\n",
        encoding="utf-8",
    )
