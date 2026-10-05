from iii_drone_supervision.log_retention import (
    BoundedLogStream,
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


def test_full_log_is_compacted_rarely_not_on_every_line(tmp_path, monkeypatch):
    import iii_drone_supervision.log_retention as retention

    compactions = []
    original = retention._compact
    monkeypatch.setattr(
        retention, "_compact", lambda *args: (compactions.append(args), original(*args))
    )
    path = tmp_path / "process.log"
    line = b"0123456789abcdef-line\n"
    for _ in range(2000):
        write_bounded_log(path, line, max_bytes=4096)

    content = path.read_bytes()
    assert len(content) <= 4096
    assert content.endswith(line)
    assert b"Older log output removed" in content
    # 2000 lines of 22 bytes overflow a 4096-byte budget ~10 times when each
    # compaction frees half of it; compacting per line would be ~1800 times.
    assert 5 <= len(compactions) <= 25
    # Retained newest output never drops below half the budget.
    assert len(content) >= 4096 // 2 - len(line)
