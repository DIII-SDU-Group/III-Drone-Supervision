from datetime import timedelta

from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import State as LifecycleState

from iii_drone_supervision.managed_node_wrapper import ManagedNodeWrapper, _start_debug_listener
from iii_drone_supervision.managed_process import ManagedProcess


class _Logger:
    def __init__(self):
        self.messages = []

    def debug(self, message):
        self.messages.append(("debug", message))

    def info(self, message):
        self.messages.append(("info", message))

    def error(self, message):
        self.messages.append(("error", message))

    def warn(self, message):
        self.messages.append(("warn", message))

    def fatal(self, message):
        self.messages.append(("fatal", message))


class _Timer:
    def __init__(self):
        self.cancelled = False
        self.destroyed = False

    def cancel(self):
        self.cancelled = True

    def destroy(self):
        self.destroyed = True


class _ManagedProcess:
    def __init__(self, *, start_result=True, running=True):
        self.start_result = start_result
        self.running = running
        self.stop_called = False
        self.cleanup_called = False

    def start(self):
        return self.start_result

    def is_running(self):
        return self.running

    def stop(self):
        self.stop_called = True
        return True

    def cleanup(self):
        self.cleanup_called = True
        return True


class _Config:
    process_monitor_period = timedelta(seconds=1)


def _make_wrapper(process):
    wrapper = ManagedNodeWrapper.__new__(ManagedNodeWrapper)
    wrapper.process_management_configuration = _Config()
    wrapper.managed_process = process
    wrapper.process_monitor_timer = None
    wrapper._error = False
    wrapper._logger = _Logger()
    wrapper.get_logger = lambda: wrapper._logger
    wrapper.get_current_state = lambda: type(
        "State", (), {"id": LifecycleState.PRIMARY_STATE_ACTIVE}
    )()
    return wrapper


def test_activate_does_not_create_monitor_timer_when_process_start_fails(monkeypatch):
    process = _ManagedProcess(start_result=False)
    wrapper = _make_wrapper(process)

    monkeypatch.setattr(
        "iii_drone_supervision.managed_node_wrapper.Node.on_activate",
        lambda self, state: TransitionCallbackReturn.SUCCESS,
    )

    def fail_create_timer(*args, **kwargs):
        raise AssertionError("timer should not be created after failed process start")

    wrapper.create_timer = fail_create_timer

    result = ManagedNodeWrapper.on_activate(wrapper, None)

    assert result == TransitionCallbackReturn.FAILURE
    assert wrapper.process_monitor_timer is None


def test_process_monitor_failure_cleans_up_when_deactivate_transition_is_invalid():
    process = _ManagedProcess(running=False)
    wrapper = _make_wrapper(process)
    timer = _Timer()
    wrapper.process_monitor_timer = timer
    wrapper.trigger_deactivate = lambda: (_ for _ in ()).throw(RuntimeError("invalid transition"))

    ManagedNodeWrapper.process_monitor_callback(wrapper)

    assert timer.cancelled is True
    assert timer.destroyed is True
    assert wrapper.process_monitor_timer is None
    assert process.stop_called is True
    assert process.cleanup_called is True
    assert wrapper._error is False
    assert any("Failed to trigger lifecycle deactivation" in message for _, message in wrapper._logger.messages)


def test_queued_process_monitor_callback_does_not_deactivate_inactive_wrapper():
    process = _ManagedProcess(running=False)
    wrapper = _make_wrapper(process)
    timer = _Timer()
    wrapper.process_monitor_timer = timer
    wrapper.get_current_state = lambda: type(
        "State", (), {"id": LifecycleState.PRIMARY_STATE_INACTIVE}
    )()
    wrapper.trigger_deactivate = lambda: (_ for _ in ()).throw(
        AssertionError("inactive lifecycle node must not be deactivated again")
    )

    ManagedNodeWrapper.process_monitor_callback(wrapper)

    assert timer.cancelled is True
    assert timer.destroyed is True
    assert wrapper.process_monitor_timer is None
    assert process.stop_called is False
    assert process.cleanup_called is False
    assert wrapper._error is False


def test_managed_process_stop_handles_already_exited_process():
    managed_process = ManagedProcess.__new__(ManagedProcess)
    managed_process._is_started = True
    managed_process.process_management_configuration = type(
        "Config",
        (),
        {"process_monitor_command": None},
    )()
    logger = _Logger()
    managed_process._parent_node = type("Node", (), {"get_logger": lambda self: logger})()

    class _ExitedProcess:
        pid = 123
        returncode = 1

        def poll(self):
            return self.returncode

        def wait(self):
            return self.returncode

    managed_process._process = _ExitedProcess()

    assert ManagedProcess.stop(managed_process) is True
    assert managed_process._process is None
    assert managed_process._is_started is False
    assert any("parent already exited" in message for _, message in logger.messages)


def test_managed_process_stop_kills_process_group_even_when_parent_exited(monkeypatch):
    managed_process = ManagedProcess.__new__(ManagedProcess)
    managed_process._is_started = True
    managed_process.process_management_configuration = type(
        "Config",
        (),
        {"process_monitor_command": None},
    )()
    logger = _Logger()
    managed_process._parent_node = type("Node", (), {"get_logger": lambda self: logger})()
    killed_groups = []

    class _ExitedProcess:
        pid = 456
        returncode = 0

        def poll(self):
            return self.returncode

        def wait(self):
            return self.returncode

    managed_process._process = _ExitedProcess()
    monkeypatch.setattr(
        "iii_drone_supervision.managed_process.os.killpg",
        lambda pid, sig: killed_groups.append((pid, sig)),
    )

    assert ManagedProcess.stop(managed_process) is True
    assert killed_groups
    assert killed_groups[0][0] == 456


def test_simulation_without_debug_port_does_not_import_debugpy(monkeypatch):
    monkeypatch.setattr("iii_drone_supervision.managed_node_wrapper.SIMULATION", True)
    monkeypatch.delenv("TF_DEBUG_PORT", raising=False)
    monkeypatch.setattr(
        "iii_drone_supervision.managed_node_wrapper.importlib.import_module",
        lambda name: (_ for _ in ()).throw(AssertionError(f"unexpected import: {name}")),
    )

    _start_debug_listener("tf")


def test_missing_debugpy_does_not_crash_simulation_runtime(monkeypatch, capsys):
    monkeypatch.setattr("iii_drone_supervision.managed_node_wrapper.SIMULATION", True)
    monkeypatch.setenv("TF_DEBUG_PORT", "49162")
    monkeypatch.setattr(
        "iii_drone_supervision.managed_node_wrapper.importlib.import_module",
        lambda name: (_ for _ in ()).throw(ImportError(name)),
    )

    _start_debug_listener("tf")

    assert "debugpy is not installed" in capsys.readouterr().out
