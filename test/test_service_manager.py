import time

from iii_drone_supervision import service_manager
from iii_drone_supervision.service_manager import ServiceProcess, TopicReadinessMonitor
from iii_drone_supervision.system_spec import SystemServiceSpec, TopicReadinessSpec


class _FakeNode:
    def create_subscription(self, *args, **kwargs):
        return object()

    def destroy_subscription(self, subscription):
        return None


def test_service_process_writes_prepared_log_before_start(tmp_path):
    spec = SystemServiceSpec(
        service_id="demo_service",
        command_factory=lambda profile: "python3 -c \"print('ready')\"",
        readiness_topics=(),
        restart_on_exit=False,
    )
    service = ServiceProcess(spec, "sim", None, tmp_path)

    try:
        snapshot = service.snapshot()
        assert snapshot.alive is False
        assert snapshot.ready is False
        assert snapshot.ready_reason == "service process is not running"
        assert snapshot.current_log_path == str(tmp_path / "current.log")

        log_text = (tmp_path / "current.log").read_text(encoding="utf-8")
        assert "SERVICE PREPARED" in log_text
        assert "loaded by the daemon and not running yet" in log_text
        assert "iii system service start demo_service" in log_text
    finally:
        service.destroy()


def test_service_process_tracks_state_and_writes_current_run_log(tmp_path):
    spec = SystemServiceSpec(
        service_id="demo_service",
        command_factory=lambda profile: "python3 -c \"import time; print('service output', flush=True); time.sleep(1)\"",
        readiness_topics=(),
        restart_on_exit=False,
        stop_timeout_sec=1.0,
    )
    service = ServiceProcess(spec, "sim", None, tmp_path)

    try:
        result = service.start()
        assert result["success"] is True

        deadline = time.time() + 2.0
        snapshot = service.snapshot()
        while time.time() < deadline and not snapshot.alive:
            time.sleep(0.05)
            snapshot = service.snapshot()

        assert snapshot.alive is True
        assert snapshot.ready is True
        assert snapshot.ready_reason == "no readiness checks configured"

        deadline = time.time() + 2.0
        while time.time() < deadline:
            if "service output" in (tmp_path / "current.log").read_text(encoding="utf-8"):
                break
            time.sleep(0.05)

        service.stop()
        log_text = (tmp_path / "current.log").read_text(encoding="utf-8")
        assert "SERVICE START" in log_text
        assert "service output" in log_text
        assert "SERVICE END" in log_text
    finally:
        service.destroy()


def test_topic_readiness_requires_stable_fresh_messages(monkeypatch):
    now = 100.0
    monkeypatch.setattr(service_manager.time, "monotonic", lambda: now)

    monitor = TopicReadinessMonitor(
        _FakeNode(),
        "demo_service",
        (
            TopicReadinessSpec(
                topic="/ready",
                message_type="std_msgs/msg/Header",
                timeout_sec=1.0,
                stable_for_sec=0.5,
            ),
        ),
    )

    try:
        monitor._mark_seen("/ready")
        ready, reason = monitor.readiness()
        assert ready is False
        assert "fresh for 0.0/0.5s" in reason

        now = 100.6
        ready, reason = monitor.readiness()
        assert ready is True
        assert reason == "ready"

        now = 102.0
        ready, reason = monitor.readiness()
        assert ready is False
        assert "/ready stale for" in reason

        monitor._mark_seen("/ready")
        ready, reason = monitor.readiness()
        assert ready is False
        assert "fresh for 0.0/0.5s" in reason
    finally:
        monitor.destroy()
