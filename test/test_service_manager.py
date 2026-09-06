import time
from types import SimpleNamespace

from iii_drone_supervision import service_manager
from iii_drone_supervision.service_manager import (
    Px4MessageFormatReadinessMonitor,
    ServiceProcess,
    TopicReadinessMonitor,
)
from iii_drone_supervision.system_spec import Px4MessageFormatReadinessSpec, SystemServiceSpec, TopicReadinessSpec


class _FakeNode:
    def __init__(self):
        self.publishers = []

    def create_subscription(self, *args, **kwargs):
        return object()

    def create_publisher(self, *args, **kwargs):
        self.publishers.append((args, kwargs))
        return _FakePublisher()

    def destroy_subscription(self, subscription):
        return None

    def destroy_publisher(self, publisher):
        return None


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


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
        monitor._mark_seen("/ready", object())
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

        monitor._mark_seen("/ready", object())
        ready, reason = monitor.readiness()
        assert ready is False
        assert "fresh for 0.0/0.5s" in reason
    finally:
        monitor.destroy()


def test_topic_readiness_reset_forgets_previous_generation(monkeypatch):
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
                stable_for_sec=0.0,
            ),
        ),
    )

    try:
        monitor._mark_seen("/ready", object())
        ready, reason = monitor.readiness()
        assert ready is True
        assert reason == "ready"

        monitor.reset()
        ready, reason = monitor.readiness()
        assert ready is False
        assert "waiting for topic(s): /ready" in reason
    finally:
        monitor.destroy()


def test_topic_readiness_snapshot_never_runs_synchronous_probe(monkeypatch):
    monitor = TopicReadinessMonitor(
        _FakeNode(),
        "demo_service",
        (
            TopicReadinessSpec(
                topic="/ready",
                message_type="std_msgs/msg/Header",
                timeout_sec=1.0,
                stable_for_sec=0.0,
            ),
        ),
    )
    monkeypatch.setattr(
        monitor,
        "_probe_topic_once",
        lambda _topic: (_ for _ in ()).throw(AssertionError("blocking probe called")),
    )

    try:
        ready, reason = monitor.readiness()
        assert ready is False
        assert reason == "waiting for topic(s): /ready"
    finally:
        monitor.destroy()


def test_px4_message_format_probe_uses_best_effort_publisher_qos():
    node = _FakeNode()
    monitor = Px4MessageFormatReadinessMonitor(
        node,
        (
            Px4MessageFormatReadinessSpec(
                topic_name="/fmu/in/register_ext_component_request",
                timeout_sec=1.0,
            ),
        ),
    )

    try:
        assert node.publishers
        _, _, publisher_qos = node.publishers[0][0]
        assert publisher_qos.reliability == service_manager.qos.QoSReliabilityPolicy.BEST_EFFORT
    finally:
        monitor.destroy()


def test_px4_message_format_probe_uses_dedicated_executor(monkeypatch):
    topic_name = "/fmu/in/register_ext_component_request"
    parent_node = _FakeNode()
    monitor = Px4MessageFormatReadinessMonitor(
        parent_node,
        (Px4MessageFormatReadinessSpec(topic_name=topic_name, timeout_sec=1.0),),
    )

    class FakeProbeNode(_FakeNode):
        context = object()

        def __init__(self):
            super().__init__()
            self.callback = None

        def create_subscription(self, _message_type, _topic, callback, _qos_profile):
            self.callback = callback
            return object()

        def destroy_node(self):
            return None

    probe_node = FakeProbeNode()

    class FakeExecutor:
        def __init__(self):
            self.node = None
            self.removed = False
            self.shutdown_called = False

        def add_node(self, node):
            self.node = node

        def spin_once(self, timeout_sec):
            assert timeout_sec == 0.1
            probe_node.callback(
                SimpleNamespace(
                    success=True,
                    topic_name=monitor._encode_topic_name(topic_name),
                )
            )

        def remove_node(self, node):
            assert node is self.node
            self.removed = True

        def shutdown(self):
            self.shutdown_called = True

    executor = FakeExecutor()

    def fail_global_spin(*_args, **_kwargs):
        raise AssertionError("global executor used")

    monkeypatch.setattr(service_manager.rclpy, "ok", lambda: True)
    monkeypatch.setattr(service_manager.rclpy, "create_node", lambda _name: probe_node)
    monkeypatch.setattr(service_manager.rclpy, "spin_once", fail_global_spin)
    monkeypatch.setattr(
        service_manager.rclpy.executors,
        "SingleThreadedExecutor",
        lambda *, context: executor if context is probe_node.context else None,
    )

    try:
        monitor._probe_topic_once(topic_name)
        assert monitor._last_success[topic_name] > 0
        assert executor.removed is True
        assert executor.shutdown_called is True
    finally:
        monitor.destroy()


def test_px4_message_format_probe_success_is_latched(monkeypatch):
    now = 100.0
    monkeypatch.setattr(service_manager.time, "monotonic", lambda: now)
    node = _FakeNode()
    monitor = Px4MessageFormatReadinessMonitor(
        node,
        (
            Px4MessageFormatReadinessSpec(
                topic_name="/fmu/in/register_ext_component_request",
                timeout_sec=1.0,
            ),
        ),
    )

    try:
        monitor._last_success["/fmu/in/register_ext_component_request"] = now
        ready, reason = monitor.readiness()
        assert ready is True
        assert reason == "ready"

        now = 200.0
        ready, reason = monitor.readiness()
        assert ready is True
        assert reason == "ready"
    finally:
        monitor.destroy()


def test_px4_message_format_reset_forgets_previous_generation(monkeypatch):
    now = 100.0
    monkeypatch.setattr(service_manager.time, "monotonic", lambda: now)
    node = _FakeNode()
    monitor = Px4MessageFormatReadinessMonitor(
        node,
        (
            Px4MessageFormatReadinessSpec(
                topic_name="/fmu/in/register_ext_component_request",
                timeout_sec=1.0,
            ),
        ),
    )

    try:
        monitor._last_success["/fmu/in/register_ext_component_request"] = now
        ready, reason = monitor.readiness()
        assert ready is True
        assert reason == "ready"

        monitor.reset()
        ready, reason = monitor.readiness()
        assert ready is False
        assert "waiting for PX4 message-format response" in reason
    finally:
        monitor.destroy()


def test_px4_readiness_snapshot_never_runs_synchronous_probe(monkeypatch):
    topic_name = "/fmu/in/register_ext_component_request"
    monitor = Px4MessageFormatReadinessMonitor(
        _FakeNode(),
        (Px4MessageFormatReadinessSpec(topic_name=topic_name, timeout_sec=1.0),),
    )
    monkeypatch.setattr(
        monitor,
        "_probe_topic_once",
        lambda _topic: (_ for _ in ()).throw(AssertionError("blocking probe called")),
    )

    try:
        ready, reason = monitor.readiness()
        assert ready is False
        assert topic_name in reason
    finally:
        monitor.destroy()
