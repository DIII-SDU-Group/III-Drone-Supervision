from lifecycle_msgs.msg import State
from types import SimpleNamespace
import asyncio

from iii_drone_supervision.system_manager import EntityRuntimeState, SystemManager


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
    def __init__(self, *, alive=True, ready=False, reason="waiting for PX4"):
        self.spec = SimpleNamespace(ready_timeout_sec=0.0)
        self.start_called = False
        self.alive = alive
        self.ready = ready
        self.reason = reason

    def start(self):
        self.start_called = True
        return {"success": True, "already_running": False, "pid": 100}

    def wait_ready(self, timeout_sec):
        del timeout_sec
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


def test_full_start_skips_nodes_blocked_by_unready_services():
    manager = SystemManager.__new__(SystemManager)
    manager._booted = True
    manager._profile_name = "sim"
    manager._service_runtimes = {"micro_ros_agent": _FakeService(ready=False)}

    class _Supervisor:
        def __init__(self):
            self.wait_node_keys = None
            self.start_kwargs = None

        def wait_for_managed_nodes(self, node_keys=None):
            self.wait_node_keys = node_keys
            return True, []

        def start(self, **kwargs):
            self.start_kwargs = kwargs
            return True, [{"key": "tf", "transition": "active"}]

    supervisor = _Supervisor()
    manager._supervisor = supervisor

    result = manager.start(activate=True, select_nodes=[], include_dependencies=False)

    assert result["success"] is True
    assert result["degraded"] is True
    assert "mission_executor" in result["blocked_nodes"]
    assert "mission_executor" not in supervisor.wait_node_keys
    assert "mission_executor" not in supervisor.start_kwargs["select_nodes"]
    assert supervisor.start_kwargs["ignore_dependencies"] is False
    assert manager._service_runtimes["micro_ros_agent"].start_called is True


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
