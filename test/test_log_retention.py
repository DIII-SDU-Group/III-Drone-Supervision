import hashlib
import json
import time

from iii_drone_supervision.log_retention import (
    BoundedLogStream,
    ClockGatedLogStream,
    configured_max_bytes,
    write_bounded_log,
)


def test_bounded_log_retains_newest_complete_output(tmp_path):
    path = tmp_path / "process.log"
    write_bounded_log(path, b"old-line\n" * 20, max_bytes=96)
    write_bounded_log(path, b"new-line\n", max_bytes=96)

    content = path.read_bytes()
    assert len(content) <= 96
    assert b"Older log output removed" in content
    assert content.endswith(b"new-line\n")


def test_non_append_write_replaces_previous_log(tmp_path):
    path = tmp_path / "current.log"
    write_bounded_log(path, "previous", max_bytes=64)
    write_bounded_log(path, "current", append=False, max_bytes=64)

    assert path.read_text(encoding="utf-8") == "current\n"


def test_invalid_configured_limit_uses_default(monkeypatch):
    monkeypatch.setenv("III_TEST_LOG_LIMIT", "not-a-number")
    assert configured_max_bytes("III_TEST_LOG_LIMIT", 123) == 123

    monkeypatch.setenv("III_TEST_LOG_LIMIT", "0")
    assert configured_max_bytes("III_TEST_LOG_LIMIT", 123) == 123


def test_bounded_stream_preserves_partial_write_semantics(tmp_path):
    path = tmp_path / "daemon.log"
    stream = BoundedLogStream(path, 128)
    stream.write("partial")
    stream.write(" line")
    stream.write("\n")

    assert path.read_text(encoding="utf-8") == "partial line\n"


def _clock_state(path, boot_id, gate):
    value = {
        "schema": "iii.receiver-clock-state/v1",
        "state_id": "0" * 64,
        "boot_id": boot_id,
        "gate": gate,
        "synchronized_monotonic_ns": 100,
        "synchronized_utc_ns": 1_000,
        "uncertainty_ns": 25,
    }
    canonical = json.dumps(
        {key: item for key, item in value.items() if key != "state_id"},
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    value["state_id"] = hashlib.sha256(canonical).hexdigest()
    path.write_text(json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n")
    return value


def test_clock_gated_daemon_output_flushes_durably_then_faults_to_memory(tmp_path):
    boot = tmp_path / "boot-id"
    boot.write_text("boot-a\n")
    clock = tmp_path / "clock.json"
    output = tmp_path / "daemon.log"
    commit = tmp_path / "run/clock-flush/system-daemon.json"
    stream = ClockGatedLogStream(
        output,
        1024 * 1024,
        clock_state_path=clock,
        boot_id_path=boot,
        flush_commit_path=commit,
    )
    stream.write("before-clock\n")
    assert not output.exists()
    flushing = _clock_state(clock, "boot-a", "FLUSHING_CLOCK")
    for _attempt in range(100):
        if commit.exists():
            break
        time.sleep(0.01)
    assert commit.exists()
    committed = json.loads(commit.read_text())
    assert committed["clock_state_id"] == flushing["state_id"]
    assert committed["records_flushed"] == 1
    assert "before-clock" in output.read_text()
    _clock_state(clock, "boot-a", "OPERATIONAL")
    stream.write("trusted\n")
    trusted = output.read_text()
    invalid = _clock_state(clock, "boot-a", "OPERATIONAL")
    invalid["uncertainty_ns"] = -1
    canonical = json.dumps(
        {key: item for key, item in invalid.items() if key != "state_id"},
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    invalid["state_id"] = hashlib.sha256(canonical).hexdigest()
    clock.write_text(json.dumps(invalid, sort_keys=True, separators=(",", ":")) + "\n")
    stream.write("invalid-clock-buffered\n")
    assert output.read_text() == trusted
    _clock_state(clock, "boot-a", "CLOCK_FAULT_ACTIVE")
    stream.write("fault-buffered\n")
    assert output.read_text() == trusted
    stream.close()
