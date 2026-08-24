"""Bounded append-only logs for long-running supervision processes."""

from __future__ import annotations

import os
from pathlib import Path
from threading import Lock


DEFAULT_ENTITY_LOG_MAX_BYTES = 16 * 1024 * 1024
DEFAULT_DAEMON_LOG_MAX_BYTES = 64 * 1024 * 1024
_TRUNCATION_MARKER = b"\n[system_manager] Older log output removed by bounded retention.\n"
_WRITE_LOCK = Lock()


def configured_max_bytes(environment_name: str, default: int) -> int:
    """Read a positive byte limit, falling back when configuration is invalid."""
    try:
        value = int(os.environ.get(environment_name, str(default)))
    except ValueError:
        return default
    return value if value > 0 else default


def write_bounded_log(
    path: Path,
    text: str | bytes,
    *,
    append: bool = True,
    max_bytes: int = DEFAULT_ENTITY_LOG_MAX_BYTES,
    ensure_newline: bool = True,
) -> None:
    """Write a log while retaining at most the newest ``max_bytes`` bytes."""
    payload = text.encode("utf-8", errors="replace") if isinstance(text, str) else text
    if ensure_newline and payload and not payload.endswith(b"\n"):
        payload += b"\n"

    path.parent.mkdir(parents=True, exist_ok=True)
    with _WRITE_LOCK:
        if not append:
            with path.open("wb") as log_file:
                log_file.write(payload[-max_bytes:])
            return

        existing_size = path.stat().st_size if path.exists() else 0
        if existing_size + len(payload) > max_bytes and path.exists():
            payload_budget = max(0, max_bytes - len(_TRUNCATION_MARKER) - len(payload))
            with path.open("r+b") as log_file:
                if payload_budget:
                    log_file.seek(max(0, existing_size - payload_budget))
                    retained = log_file.read(payload_budget)
                    newline = retained.find(b"\n")
                    if newline >= 0:
                        retained = retained[newline + 1 :]
                else:
                    retained = b""
                log_file.seek(0)
                log_file.write(_TRUNCATION_MARKER)
                log_file.write(retained)
                log_file.truncate()

        with path.open("ab") as log_file:
            if len(payload) > max_bytes:
                log_file.seek(0)
                log_file.truncate()
                log_file.write(_TRUNCATION_MARKER)
                log_file.write(payload[-max(0, max_bytes - len(_TRUNCATION_MARKER)) :])
            else:
                log_file.write(payload)


class BoundedLogStream:
    """Minimal text stream used for daemon stdout/stderr."""

    encoding = "utf-8"

    def __init__(self, path: Path, max_bytes: int):
        self._path = path
        self._max_bytes = max_bytes

    def write(self, text: str) -> int:
        if text:
            write_bounded_log(
                self._path,
                text,
                max_bytes=self._max_bytes,
                ensure_newline=False,
            )
        return len(text)

    def flush(self) -> None:
        return None

    def isatty(self) -> bool:
        return False
