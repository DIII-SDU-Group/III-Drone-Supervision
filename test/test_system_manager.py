from lifecycle_msgs.msg import State
from types import SimpleNamespace
import asyncio

from iii_drone_supervision.system_manager import EntityRuntimeState, SystemManager
import iii_drone_supervision.system_manager as system_manager_module


class _ProcessEvent:
    pid = 42
    returncode = 0
    text = b"traceback line\n"


def test_process_callbacks_update_entity_runtime_state(tmp_path):
    manager = SystemManager.__new__(SystemManager)
    manager._entity_states = {
        "trajectory_generator": EntityRuntimeState(entity_id="trajectory_generator"),
    }

    start_callback = SystemManager._make_process_started_callback(manager, "trajectory_generator", 1, tmp_path)
    exit_callback = SystemManager._make_process_exited_callback(manager, "trajectory_generator", 1, tmp_path)

    class _NullLock:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    manager._lock = _NullLock()

    start_callback(_ProcessEvent(), None)
    assert manager._entity_states["trajectory_generator"].alive is True
    assert manager._entity_states["trajectory_generator"].start_count == 1
    assert manager._entity_states["trajectory_generator"].pid == 42
    assert "RUN START" in (tmp_path / "process.log").read_text(encoding="utf-8")
    assert "RUN START" in (tmp_path / "current.log").read_text(encoding="utf-8")

    exit_callback(_ProcessEvent(), None)
    assert manager._entity_states["trajectory_generator"].alive is False
    assert manager._entity_states["trajectory_generator"].pid is None
    assert manager._entity_states["trajectory_generator"].exit_count == 1
    assert "RUN END" in (tmp_path / "process.log").read_text(encoding="utf-8")
    assert "RUN END" in (tmp_path / "current.log").read_text(encoding="utf-8")


def test_health_status_message_aggregates_services_and_processes(monkeypatch):
    class _FakeSubsystemHealthStatus:
        STATUS_OK = 1
        STATUS_DEGRADED = 2
        STATUS_UNAVAILABLE = 3

        def __init__(self):
            self.subsystem_id = ""
            self.label = ""
            self.ready = False
            self.degraded = False
            self.status = 0
            self.reason = ""
            self.degraded_reasons = []
            self.owner = ""

    class _FakeSystemHealthStatus:
        SYSTEM_STATE_STOPPED = 1
        SYSTEM_STATE_RUNNING = 3
        SYSTEM_STATE_READY = 4
        SYSTEM_STATE_DEGRADED = 5

        def __init__(self):
            self.profile = ""
            self.system_state = 0
            self.daemon_ready = False
            self.runtime_booted = False
            self.system_active = False
            self.ready = False
            self.degraded = False
            self.degraded_reasons = []
            self.managed_node_count = 0
            self.active_managed_node_count = 0
            self.service_count = 0
            self.ready_service_count = 0
            self.subsystems = []

    monkeypatch.setattr(system_manager_module, "SubsystemHealthStatus", _FakeSubsystemHealthStatus)
    monkeypatch.setattr(system_manager_module, "SystemHealthStatus", _FakeSystemHealthStatus)

    manager = SystemManager.__new__(SystemManager)
    manager._node = None
    manager.status = lambda: {
        "booted": True,
        "profile": "sim",
        "managed_nodes": {"mission": "active", "perception": "inactive"},
        "services": {
            "micro_ros_agent": {"ready": True, "reason": "ready"},
            "px4_gazebo": {"ready": False, "reason": "waiting for Gazebo"},
        },
        "processes": {
            "mission": {"alive": True},
            "perception": {"alive": False},
        },
    }

    message = manager.health_status_message()

    assert message.profile == "sim"
    assert message.runtime_booted is True
    assert message.system_active is False
    assert message.service_count == 2
    assert message.ready_service_count == 1
    assert message.degraded is True
    assert "px4_gazebo: waiting for Gazebo" in message.degraded_reasons
    assert any(subsystem.subsystem_id == "perception" for subsystem in message.subsystems)


def test_process_io_callback_appends_current_run_log_when_generation_matches(tmp_path):
    manager = SystemManager.__new__(SystemManager)
    manager._entity_states = {
        "trajectory_generator": EntityRuntimeState(
            entity_id="trajectory_generator",
            generation=1,
            current_log_path=str(tmp_path / "current.log"),
        ),
    }

    class _NullLock:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    manager._lock = _NullLock()
    callback = SystemManager._make_process_io_callback(manager, "trajectory_generator", 1, tmp_path, "stderr")

    callback(_ProcessEvent())

    assert (tmp_path / "process.log").read_text(encoding="utf-8") == "traceback line\n"
    assert (tmp_path / "current.log").read_text(encoding="utf-8") == "traceback line\n"


def test_process_io_callback_appends_entity_process_log(tmp_path):
    manager = SystemManager.__new__(SystemManager)
    manager._entity_states = {}

    class _NullLock:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    manager._lock = _NullLock()
    callback = SystemManager._make_process_io_callback(manager, "configuration_server", 1, tmp_path, "stderr")

    callback(_ProcessEvent())

    assert (tmp_path / "process.log").read_text(encoding="utf-8") == "traceback line\n"


def test_stale_process_exit_does_not_mark_current_generation_dead(tmp_path):
    manager = SystemManager.__new__(SystemManager)
    manager._entity_states = {
        "trajectory_generator": EntityRuntimeState(
            entity_id="trajectory_generator",
            alive=True,
            start_count=1,
            generation=2,
            pid=99,
        ),
    }

    class _NullLock:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    manager._lock = _NullLock()
    exit_callback = SystemManager._make_process_exited_callback(manager, "trajectory_generator", 1, tmp_path)

    exit_callback(_ProcessEvent(), None)

    assert manager._entity_states["trajectory_generator"].alive is True
    assert manager._entity_states["trajectory_generator"].pid == 99
    assert manager._entity_states["trajectory_generator"].exit_count == 0


def test_shutdown_runtime_is_idempotent_when_not_booted():
    manager = SystemManager.__new__(SystemManager)
    manager._booted = False

    result = asyncio.run(manager.shutdown_runtime(select_nodes=[], include_dependencies=False))

    assert result["success"] is True
    assert "not booted" in result["message"]


def test_start_waits_for_lifecycle_services_before_supervisor_start():
    manager = SystemManager.__new__(SystemManager)
    manager._booted = True

    class _Supervisor:
        def __init__(self):
            self.wait_called = False
            self.start_called = False

        def wait_for_managed_nodes(self):
            self.wait_called = True
            return True, []

        def start(self, **kwargs):
            self.start_called = True
            return True, [{"key": "tf", "transition": "active"}]

    supervisor = _Supervisor()
    manager._supervisor = supervisor

    result = manager.start(activate=True, select_nodes=[], include_dependencies=True)

    assert result["success"] is True
    assert supervisor.wait_called is True
    assert supervisor.start_called is True


def test_start_reports_missing_lifecycle_services_without_starting():
    manager = SystemManager.__new__(SystemManager)
    manager._booted = True

    class _Supervisor:
        def __init__(self):
            self.start_called = False

        def wait_for_managed_nodes(self):
            return False, ["tf"]

        def start(self, **kwargs):
            self.start_called = True
            return True, []

    supervisor = _Supervisor()
    manager._supervisor = supervisor

    result = manager.start(activate=True, select_nodes=[], include_dependencies=True)

    assert result["success"] is False
    assert "tf" in result["error"]
    assert supervisor.start_called is False


def test_start_reports_non_active_nodes_on_failed_activation():
    manager = SystemManager.__new__(SystemManager)
    manager._booted = True

    inactive = State()
    inactive.id = State.PRIMARY_STATE_INACTIVE
    inactive.label = "inactive"
    active = State()
    active.id = State.PRIMARY_STATE_ACTIVE
    active.label = "active"

    class _Supervisor:
        def wait_for_managed_nodes(self):
            return True, []

        def start(self, **kwargs):
            del kwargs
            return False, [{"key": "tf", "transition": "active"}]

        def _get_node_states(self):
            return {"mission_executor": inactive, "tf": active}

    manager._supervisor = _Supervisor()

    result = manager.start(activate=True, select_nodes=[], include_dependencies=True)

    assert result["success"] is False
    assert "mission_executor=inactive" in result["error"]
    assert "iii system logs mission_executor" in result["error"] or "iii system logs <node>" in result["error"]


class _FakeService:
    def __init__(self, *, alive=True, ready=False, ready_after_wait=False, reason="waiting for PX4"):
        self.spec = SimpleNamespace(ready_timeout_sec=0.0)
        self.start_called = False
        self.alive = alive
        self.ready = ready
        self.ready_after_wait = ready_after_wait
        self.reason = reason

    def start(self):
        self.start_called = True
        return {"success": True, "already_running": False, "pid": 100}

    def wait_ready(self, timeout_sec):
        del timeout_sec
        if self.ready_after_wait:
            self.ready = True
            self.reason = "ready"
        return self.ready, self.reason

    def snapshot(self):
        return SimpleNamespace(
            service_id="micro_ros_agent",
            alive=self.alive,
            ready=self.ready,
            start_count=1,
            exit_count=0,
            generation=1,
            pid=100 if self.alive else None,
            command="micro_ros_agent",
            current_log_path="/tmp/current.log",
            last_returncode=None,
            ready_reason=self.reason,
        )


def test_full_start_waits_for_blocked_service_nodes_after_starting_unblocked_nodes():
    manager = SystemManager.__new__(SystemManager)
    manager._booted = True
    manager._profile_name = "sim"
    manager._service_runtimes = {"micro_ros_agent": _FakeService(ready=False, ready_after_wait=True)}

    class _Supervisor:
        def __init__(self):
            self.wait_node_keys = []
            self.start_calls = []

        def wait_for_managed_nodes(self, node_keys=None):
            self.wait_node_keys.append(node_keys)
            return True, []

        def start(self, **kwargs):
            self.start_calls.append(kwargs)
            return True, [{"key": "tf", "transition": "active"}]

    supervisor = _Supervisor()
    manager._supervisor = supervisor

    result = manager.start(activate=True, select_nodes=[], include_dependencies=False)

    assert result["success"] is True
    assert result["blocked_nodes"] == {}
    assert len(supervisor.start_calls) == 2
    assert "mission_executor" not in supervisor.start_calls[0]["select_nodes"]
    assert "custom_operation" not in supervisor.start_calls[0]["select_nodes"]
    assert set(supervisor.start_calls[1]["select_nodes"]) == {"mission_executor", "custom_operation"}
    assert supervisor.start_calls[0]["ignore_dependencies"] is False
    assert supervisor.start_calls[1]["ignore_dependencies"] is False
    assert manager._service_runtimes["micro_ros_agent"].start_called is True


def test_full_start_retries_all_nodes_after_unblocked_prestart_failure_once_services_ready():
    manager = SystemManager.__new__(SystemManager)
    manager._booted = True
    manager._profile_name = "sim"
    manager._service_runtimes = {"micro_ros_agent": _FakeService(ready=False, ready_after_wait=True)}

    class _Supervisor:
        def __init__(self):
            self.start_calls = []

        def wait_for_managed_nodes(self, node_keys=None):
            del node_keys
            return True, []

        def start(self, **kwargs):
            self.start_calls.append(kwargs)
            selected = set(kwargs["select_nodes"])
            if "mission_executor" not in selected and "custom_operation" not in selected:
                return False, [{"key": "tf", "transition": "config"}]
            return True, [{"key": node_id, "transition": "active"} for node_id in sorted(selected)]

        def _get_node_states(self):
            inactive = State()
            inactive.id = State.PRIMARY_STATE_INACTIVE
            inactive.label = "inactive"
            active = State()
            active.id = State.PRIMARY_STATE_ACTIVE
            active.label = "active"
            return {"mission_executor": inactive, "custom_operation": inactive, "tf": active}

    supervisor = _Supervisor()
    manager._supervisor = supervisor

    result = manager.start(activate=True, select_nodes=[], include_dependencies=False)

    assert result["success"] is True
    assert "warning" in result
    assert "prestart_error" in result
    assert len(supervisor.start_calls) == 2
    assert "mission_executor" not in supervisor.start_calls[0]["select_nodes"]
    assert "custom_operation" not in supervisor.start_calls[0]["select_nodes"]
    assert "mission_executor" in supervisor.start_calls[1]["select_nodes"]
    assert "custom_operation" in supervisor.start_calls[1]["select_nodes"]
    assert manager._service_runtimes["micro_ros_agent"].ready is True


def test_selected_service_blocked_node_fails_without_lifecycle_transition():
    manager = SystemManager.__new__(SystemManager)
    manager._booted = True
    manager._profile_name = "sim"
    manager._service_runtimes = {"micro_ros_agent": _FakeService(ready=False)}

    class _Supervisor:
        def __init__(self):
            self.start_called = False

        def wait_for_managed_nodes(self, node_keys=None):
            del node_keys
            return True, []

        def start(self, **kwargs):
            del kwargs
            self.start_called = True
            return True, []

    supervisor = _Supervisor()
    manager._supervisor = supervisor

    result = manager.start(activate=True, select_nodes=["mission_executor"], include_dependencies=True)

    assert result["success"] is False
    assert "mission_executor" in result["blocked_nodes"]
    assert "micro_ros_agent" in result["error"]
    assert supervisor.start_called is False


def test_selected_cold_restart_refreshes_launch_process(monkeypatch):
    manager = SystemManager.__new__(SystemManager)
    manager._entity_states = {
        "mission_executor": EntityRuntimeState(
            entity_id="mission_executor",
            alive=True,
            pid=10,
        ),
    }

    class _NullLock:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    manager._lock = _NullLock()
    killed = []

    def fake_kill(pid, sig):
        killed.append((pid, sig))
        manager._entity_states["mission_executor"].pid = 11
        manager._entity_states["mission_executor"].alive = True

    monkeypatch.setattr(system_manager_module.os, "kill", fake_kill)

    result = manager._restart_launch_processes(["mission_executor"], timeout_sec=1.0)

    assert result["success"] is True
    assert killed
    assert result["entities"]["mission_executor"]["old_pid"] == 10
    assert result["entities"]["mission_executor"]["new_pid"] == 11
