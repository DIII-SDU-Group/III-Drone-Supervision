from lifecycle_msgs.msg import State
from types import SimpleNamespace
from threading import Lock
import asyncio
import subprocess

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


def test_respawned_desired_active_process_schedules_lifecycle_recovery(tmp_path, monkeypatch):
    recovered = []

    class _ImmediateThread:
        def __init__(self, *, target, args, daemon):
            assert daemon is True
            self._target = target
            self._args = args

        def start(self):
            self._target(*self._args)

    manager = SystemManager.__new__(SystemManager)
    manager._entity_states = {
        "mission_executor": EntityRuntimeState(
            entity_id="mission_executor",
            alive=False,
            start_count=1,
            desired_active=True,
        ),
    }

    class _NullLock:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    manager._lock = _NullLock()
    manager._recover_respawned_entity = lambda *args: recovered.append(args)
    monkeypatch.setattr(system_manager_module, "Thread", _ImmediateThread)

    callback = SystemManager._make_process_started_callback(manager, "mission_executor", 3, tmp_path)
    callback(_ProcessEvent(), None)

    assert recovered == [("mission_executor", 3, 42, tmp_path)]
    assert manager._entity_states["mission_executor"].recovery_in_progress is True


def test_first_process_start_does_not_schedule_lifecycle_recovery(tmp_path, monkeypatch):
    started_threads = []

    class _Thread:
        def __init__(self, **kwargs):
            started_threads.append(kwargs)

        def start(self):
            raise AssertionError("initial process start must not schedule recovery")

    manager = SystemManager.__new__(SystemManager)
    manager._entity_states = {
        "mission_executor": EntityRuntimeState(
            entity_id="mission_executor",
            desired_active=True,
        ),
    }

    class _NullLock:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, tb):
            return False

    manager._lock = _NullLock()
    monkeypatch.setattr(system_manager_module, "Thread", _Thread)

    callback = SystemManager._make_process_started_callback(manager, "mission_executor", 3, tmp_path)
    callback(_ProcessEvent(), None)

    assert started_threads == []


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
    manager.runtime_snapshot = lambda: {
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


def test_runtime_snapshot_requires_live_desired_processes_and_ready_services():
    manager = SystemManager.__new__(SystemManager)
    manager._lock = Lock()
    manager._booted = True
    manager._profile_name = "sim"
    manager._entity_states = {
        "mission": EntityRuntimeState(
            entity_id="mission",
            alive=True,
            desired_active=True,
        ),
        "control": EntityRuntimeState(
            entity_id="control",
            alive=True,
            desired_active=True,
            recovery_in_progress=True,
        ),
    }
    manager._service_statuses = lambda: {"micro_ros_agent": {"ready": True}}

    recovering = manager.runtime_snapshot()
    assert recovering["active"] is False
    assert recovering["managed_nodes"]["control"] == "inactive"

    manager._entity_states["control"].recovery_in_progress = False
    assert manager.runtime_snapshot()["active"] is True

    manager._service_statuses = lambda: {"micro_ros_agent": {"ready": False}}
    assert manager.runtime_snapshot()["active"] is False


def test_system_health_qos_is_transient_local():
    qos = system_manager_module.system_health_qos()

    assert qos.depth == 1
    assert qos.durability.name == "TRANSIENT_LOCAL"


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


def test_shutdown_runtime_reaps_launch_entity_that_survives_launch_shutdown():
    child = subprocess.Popen(["sleep", "60"])
    try:
        manager = SystemManager.__new__(SystemManager)
        manager._booted = True
        manager._lock = Lock()
        manager._launch_generation = 4
        manager._entity_states = {
            "stuck_node": EntityRuntimeState(
                entity_id="stuck_node",
                alive=True,
                generation=4,
                pid=child.pid,
            )
        }
        manager._service_runtimes = {}
        manager._launch_task = None
        manager._profile_name = "sim"
        manager._set_desired_active = lambda *_args, **_kwargs: None
        manager.managed_node_ids = lambda: ["stuck_node"]
        manager._return_with_health = lambda payload: payload

        class _Supervisor:
            def shutdown(self, **_kwargs):
                return None

            def destroy(self):
                return None

        class _LaunchService:
            def shutdown(self):
                return None

        manager._supervisor = _Supervisor()
        manager._launch_service = _LaunchService()

        result = asyncio.run(manager.shutdown_runtime(select_nodes=[], include_dependencies=False))

        assert result["success"] is True
        assert child.wait(timeout=2.0) is not None
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=2.0)


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


def test_service_restart_restarts_active_dependents_after_service_is_ready(monkeypatch):
    class _RestartableService(_FakeService):
        def restart(self):
            self.alive = True
            self.ready = False
            self.ready_after_wait = True
            return {"success": True, "pid": 101}

    active = State()
    active.id = State.PRIMARY_STATE_ACTIVE
    inactive = State()
    inactive.id = State.PRIMARY_STATE_INACTIVE

    class _Supervisor:
        def __init__(self):
            self.stop_calls = []
            self.start_calls = []

        def _get_node_states(self):
            return {
                "mission_executor": active,
                "custom_operation": active,
                "tf": inactive,
            }

        def stop(self, **kwargs):
            self.stop_calls.append(kwargs)
            return True, [{"key": key} for key in kwargs["select_nodes"]]

        def start(self, **kwargs):
            self.start_calls.append(kwargs)
            return True, [{"key": key} for key in kwargs["select_nodes"]]

        def wait_for_managed_nodes(self, node_keys=None, timeout_sec=None):
            del timeout_sec
            return True, [] if node_keys else []

    manager = SystemManager.__new__(SystemManager)
    manager._booted = True
    manager._profile_name = "sim"
    manager._service_runtimes = {"micro_ros_agent": _RestartableService(ready=True)}
    manager._supervisor = _Supervisor()
    monkeypatch.setattr(manager, "_return_with_health", lambda result: result)

    result = manager.service_restart("micro_ros_agent")

    assert result["success"] is True
    assert result["dependent_nodes"] == ["custom_operation", "mission_executor"]
    assert manager._supervisor.stop_calls[0]["ignore_dependencies"] is True
    assert manager._supervisor.start_calls[0]["ignore_dependencies"] is False
    assert set(manager._supervisor.start_calls[0]["select_nodes"]) == {
        "custom_operation",
        "mission_executor",
    }


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

    result = asyncio.run(manager._restart_launch_processes(["mission_executor"], timeout_sec=1.0))

    assert result["success"] is True
    assert killed
    assert result["entities"]["mission_executor"]["old_pid"] == 10
    assert result["entities"]["mission_executor"]["new_pid"] == 11
