"""Bounded append-only logs for long-running supervision processes."""

from __future__ import annotations

from collections import deque
import hashlib
import json
import os
from pathlib import Path
from threading import Event, Lock, Thread
import time
from typing import Any, Mapping


DEFAULT_ENTITY_LOG_MAX_BYTES = 16 * 1024 * 1024
DEFAULT_DAEMON_LOG_MAX_BYTES = 64 * 1024 * 1024
_TRUNCATION_MARKER = (
    b"\n[system_manager] Older log output removed by bounded retention.\n"
)
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


def _canonical(value: Mapping[str, Any]) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")


def _identity(value: Mapping[str, Any], field: str) -> str:
    return hashlib.sha256(
        _canonical({key: item for key, item in value.items() if key != field})
    ).hexdigest()


class ClockGatedLogStream:
    """Keep daemon output process-local until the receiver clock barrier."""

    encoding = "utf-8"

    def __init__(
        self,
        path: Path,
        max_bytes: int,
        *,
        clock_state_path: Path,
        boot_id_path: Path,
        flush_commit_path: Path,
        monotonic_ns=time.monotonic_ns,
        maximum_records: int = 10_000,
        maximum_buffer_bytes: int = 16 * 1024**2,
    ) -> None:
        self._path = path
        self._max_bytes = max_bytes
        self._clock_state_path = clock_state_path
        self._boot_id_path = boot_id_path
        self._flush_commit_path = flush_commit_path
        self._monotonic_ns = monotonic_ns
        self._maximum_records = maximum_records
        self._maximum_buffer_bytes = maximum_buffer_bytes
        self._rows: deque[tuple[int, str, int]] = deque()
        self._buffer_bytes = 0
        self._dropped = 0
        self._trusted = False
        self._last_gate = "DEGRADED_CLOCK"
        self._lock = Lock()
        self._stop = Event()
        self._thread = Thread(
            target=self._watch_clock, name="iii-daemon-clock-flush", daemon=True
        )
        self._thread.start()

    def _boot_id(self) -> str:
        return self._boot_id_path.read_text(encoding="ascii").strip()

    def _clock(self) -> tuple[str, dict[str, Any] | None]:
        try:
            if self._clock_state_path.is_symlink():
                return "DEGRADED_CLOCK", None
            raw = self._clock_state_path.read_bytes()
            value = json.loads(raw)
            boot_id = self._boot_id()
        except (OSError, UnicodeDecodeError, json.JSONDecodeError):
            return "DEGRADED_CLOCK", None
        if (
            not isinstance(value, dict)
            or raw != _canonical(value) + b"\n"
            or value.get("schema") != "iii.receiver-clock-state/v1"
            or value.get("state_id") != _identity(value, "state_id")
            or value.get("boot_id") != boot_id
        ):
            return "DEGRADED_CLOCK", None
        gate = str(value.get("gate"))
        if gate == "CLOCK_FAULT_ACTIVE":
            return gate, None
        if gate not in {"FLUSHING_CLOCK", "OPERATIONAL"}:
            return "DEGRADED_CLOCK", None
        for field in (
            "synchronized_monotonic_ns",
            "synchronized_utc_ns",
            "uncertainty_ns",
        ):
            if (
                not isinstance(value.get(field), int)
                or isinstance(value[field], bool)
                or value[field] < 0
            ):
                return "DEGRADED_CLOCK", None
        return gate, value

    def _buffer(self, text: str) -> None:
        size = len(text.encode("utf-8", errors="replace"))
        self._rows.append((self._monotonic_ns(), text, size))
        self._buffer_bytes += size
        while (
            len(self._rows) > self._maximum_records
            or self._buffer_bytes > self._maximum_buffer_bytes
        ):
            _monotonic, _text, removed = self._rows.popleft()
            self._buffer_bytes -= removed
            self._dropped += 1

    def _durable(self) -> None:
        descriptor = os.open(self._path, os.O_RDONLY | os.O_NOFOLLOW)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        directory = os.open(self._path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)

    def _commit(self, state: Mapping[str, Any], records: int) -> None:
        value: dict[str, Any] = {
            "schema": "iii.clock-flush-commit/v1",
            "commit_id": "0" * 64,
            "service": "system-daemon",
            "boot_id": state["boot_id"],
            "clock_state_id": state["state_id"],
            "records_flushed": records,
            "dropped_records": self._dropped,
            "committed_monotonic_ns": self._monotonic_ns(),
        }
        value["commit_id"] = _identity(value, "commit_id")
        self._flush_commit_path.parent.mkdir(parents=True, exist_ok=True, mode=0o750)
        temporary = self._flush_commit_path.with_name(
            f".{self._flush_commit_path.name}.partial-{os.getpid()}"
        )
        try:
            if temporary.exists() and not temporary.is_symlink():
                temporary.unlink()
            descriptor = os.open(
                temporary,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                0o640,
            )
            try:
                view = memoryview(_canonical(value) + b"\n")
                while view:
                    written = os.write(descriptor, view)
                    if written <= 0:
                        raise OSError("clock flush commit write made no progress")
                    view = view[written:]
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            os.replace(temporary, self._flush_commit_path)
        finally:
            if temporary.exists() and not temporary.is_symlink():
                temporary.unlink()
        directory = os.open(
            self._flush_commit_path.parent, os.O_RDONLY | os.O_DIRECTORY
        )
        try:
            os.fsync(directory)
        finally:
            os.close(directory)

    def _flush_ring(self, gate: str, state: Mapping[str, Any]) -> None:
        records = len(self._rows)
        for monotonic, text, _size in self._rows:
            estimate = state["synchronized_utc_ns"] + (
                monotonic - state["synchronized_monotonic_ns"]
            )
            row = {
                "boot_id": state["boot_id"],
                "monotonic_ns": monotonic,
                "utc_estimate_ns": estimate,
                "utc_lower_ns": estimate - state["uncertainty_ns"],
                "utc_upper_ns": estimate + state["uncertainty_ns"],
                "utc_reconstructed": True,
                "utc_uncertainty_ns": state["uncertainty_ns"],
                "output": text,
            }
            write_bounded_log(
                self._path,
                _canonical(row) + b"\n",
                max_bytes=self._max_bytes,
                ensure_newline=False,
            )
        marker = {
            "boot_id": state["boot_id"],
            "kind": "preclock-flush",
            "records_flushed": records,
            "dropped_records": self._dropped,
            "utc_reconstructed": True,
            "utc_uncertainty_ns": state["uncertainty_ns"],
        }
        write_bounded_log(
            self._path,
            _canonical(marker) + b"\n",
            max_bytes=self._max_bytes,
            ensure_newline=False,
        )
        self._durable()
        self._rows.clear()
        self._buffer_bytes = 0
        self._trusted = True
        if gate == "FLUSHING_CLOCK":
            self._commit(state, records)

    def _observe(self) -> None:
        gate, state = self._clock()
        if gate in {"DEGRADED_CLOCK", "CLOCK_FAULT_ACTIVE"} and self._last_gate in {
            "FLUSHING_CLOCK",
            "OPERATIONAL",
        }:
            self._trusted = False
            self._dropped = 0
        self._last_gate = gate
        if state is not None and not self._trusted:
            self._flush_ring(gate, state)

    def _watch_clock(self) -> None:
        while not self._stop.wait(0.05):
            with self._lock:
                self._observe()

    def write(self, text: str) -> int:
        if not text:
            return 0
        with self._lock:
            self._observe()
            if self._trusted:
                write_bounded_log(
                    self._path,
                    text,
                    max_bytes=self._max_bytes,
                    ensure_newline=False,
                )
            else:
                self._buffer(text)
        return len(text)

    def flush(self) -> None:
        return None

    def isatty(self) -> bool:
        return False

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
