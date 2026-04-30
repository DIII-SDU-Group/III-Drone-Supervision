from pathlib import Path

import pytest
import yaml
from lifecycle_msgs.msg import State
from rclpy.logging import LoggingSeverity

from iii_drone_supervision.managed_node_client import ManagedNodeClient
from iii_drone_supervision.process_management_configuration import ProcessManagementConfiguration
from iii_drone_supervision.supervisor import Supervisor


def _write_yaml(path: Path, payload: dict):
    path.write_text(yaml.safe_dump(payload))
    return path


def test_process_management_configuration_expands_environment_variables(tmp_path, monkeypatch):
    monkeypatch.setenv("III_WORKDIR", str(tmp_path))

    config_path = _write_yaml(
        tmp_path / "managed_node.yaml",
        {
            "node_name": "node",
            "node_namespace": "ns",
            "command": "echo hello",
            "working_directory": "$III_WORKDIR",
            "process_monitor_period_sec": 1,
        },
    )

    config = ProcessManagementConfiguration(str(config_path))

    assert config.node_name == "node"
    assert config.node_namespace == "ns"
    assert config.working_directory == str(tmp_path)
    assert config.process_monitor_command is None


def test_process_management_configuration_rejects_invalid_timeout_without_monitor(tmp_path):
    config_path = _write_yaml(
        tmp_path / "invalid.yaml",
        {
            "node_name": "node",
            "node_namespace": "",
            "command": "echo hello",
            "working_directory": str(tmp_path),
            "process_monitor_period_sec": 1,
            "process_start_timeout_sec": 3,
        },
    )

    with pytest.raises(ValueError, match="require a process monitor command"):
        ProcessManagementConfiguration(str(config_path))


def test_process_management_configuration_maps_log_level_and_timeouts(tmp_path):
    config_path = _write_yaml(
        tmp_path / "managed_node.yaml",
        {
            "node_name": "node",
            "node_namespace": "ns",
            "log_level": "warn",
            "command": "echo hello",
            "working_directory": str(tmp_path),
            "process_monitor_command": [{"type": "command", "command": "true"}],
            "process_monitor_period_sec": 2,
            "process_start_timeout_sec": 6,
            "process_stop_timeout_sec": 8,
        },
    )

    config = ProcessManagementConfiguration(str(config_path))

    assert config.log_level == LoggingSeverity.WARN
    assert config.process_monitor_period.total_seconds() == 2
    assert config.process_start_timeout.total_seconds() == 6
    assert config.process_stop_timeout.total_seconds() == 8


def test_process_management_configuration_rejects_unknown_fields(tmp_path):
    config_path = _write_yaml(
        tmp_path / "invalid.yaml",
        {
            "node_name": "node",
            "node_namespace": "ns",
            "command": "echo hello",
            "working_directory": str(tmp_path),
            "process_monitor_period_sec": 1,
            "unexpected": True,
        },
    )

    with pytest.raises(ValueError, match="Unknown fields"):
        ProcessManagementConfiguration(str(config_path))


def test_process_management_configuration_rejects_invalid_process_monitor_command(tmp_path):
    config_path = _write_yaml(
        tmp_path / "invalid_monitor.yaml",
        {
            "node_name": "node",
            "node_namespace": "ns",
            "command": "echo hello",
            "working_directory": str(tmp_path),
            "process_monitor_command": [
                {
                    "type": "topic",
                    "topic": "/status",
                    "message_type": "std_msgs/msg/Header",
                    "timeout_sec": "3",
                }
            ],
            "process_monitor_period_sec": 1,
        },
    )

    with pytest.raises(ValueError, match="timeout_sec"):
        ProcessManagementConfiguration(str(config_path))


def test_process_management_configuration_requires_topic_message_type(tmp_path):
    config_path = _write_yaml(
        tmp_path / "invalid_monitor.yaml",
        {
            "node_name": "node",
            "node_namespace": "ns",
            "command": "echo hello",
            "working_directory": str(tmp_path),
            "process_monitor_command": [{"type": "topic", "topic": "/status", "timeout_sec": 3}],
            "process_monitor_period_sec": 1,
        },
    )

    with pytest.raises(ValueError, match="message_type"):
        ProcessManagementConfiguration(str(config_path))


def test_managed_node_client_monitor_is_noop_after_destroy():
    client = ManagedNodeClient.__new__(ManagedNodeClient)
    client._destroyed = True
    client.get_state_client = None
    client._state = State()
    client._state.id = State.PRIMARY_STATE_UNKNOWN
    client._state.label = "UNKNOWN"

    client.monitor_callback()

    assert client._update_state() is client._state


def test_managed_node_client_verifies_state_after_unmonitored_transition():
    client = ManagedNodeClient.__new__(ManagedNodeClient)
    client.monitor_state = False
    client._request_state_timeout_ms = 100
    client._state = State()
    client._state.id = State.PRIMARY_STATE_INACTIVE
    client._state.label = "inactive"
    client._request_transition = lambda transition_id: True

    still_inactive = State()
    still_inactive.id = State.PRIMARY_STATE_INACTIVE
    still_inactive.label = "inactive"
    client.refresh_state = lambda timeout_ms=None: still_inactive

    assert client.request_activate() is False


class _ManagedNodeClient:
    def __init__(self, state_id):
        self._state_ids = [state_id] if not isinstance(state_id, list) else state_id
        self.refresh_count = 0

    @property
    def state(self):
        return type("StateValue", (), {"id": self._state_ids[-1]})()

    def refresh_state(self, timeout_ms=None):
        del timeout_ms
        self.refresh_count += 1
        state_id = self._state_ids[min(self.refresh_count - 1, len(self._state_ids) - 1)]
        return type("StateValue", (), {"id": state_id})()


class _Logger:
    def info(self, message):
        del message

    def debug(self, message):
        del message

    def warn(self, message):
        del message

    def error(self, message):
        del message

    def fatal(self, message):
        del message


class _Node:
    def get_logger(self):
        return _Logger()


def _make_supervisor(managed_nodes: dict, node_states: dict | None = None) -> Supervisor:
    supervisor = Supervisor.__new__(Supervisor)
    supervisor._managed_nodes_dict = managed_nodes
    supervisor._max_threads = 1
    supervisor.node = _Node()
    transitions = supervisor._expand_managed_node_transitions(managed_nodes)
    transition_tree, _, _ = supervisor._construct_transition_tree(transitions)
    supervisor._transition_tree = transition_tree
    supervisor._managed_node_clients = {
        key: _ManagedNodeClient((node_states or {}).get(key, State.PRIMARY_STATE_UNCONFIGURED))
        for key in managed_nodes
    }
    return supervisor


def test_supervisor_transition_tree_contains_dependency_edges():
    managed_nodes = {
        "perception": {
            "node_name": "perception",
            "node_namespace": "/core",
        },
        "mission": {
            "node_name": "mission",
            "node_namespace": "/core",
            "config_depend": {"perception": "config"},
            "active_depend": {"perception": "active"},
        },
    }

    Supervisor.validate_supervision_config(
        {
            "monitor_period_ms": 100,
            "request_state_timeout_ms": 500,
            "max_threads": 2,
            "managed_nodes": managed_nodes,
        }
    )

    supervisor = Supervisor.__new__(Supervisor)
    transitions = supervisor._expand_managed_node_transitions(managed_nodes)
    transition_tree, leaf_keys, root_keys = supervisor._construct_transition_tree(transitions)

    assert "perception_config" in transition_tree
    assert "mission_active" in transition_tree
    assert "perception_config" in leaf_keys
    assert "mission_active" in root_keys
    assert "perception_active" in transition_tree["mission_active"]["depends_on"]


def test_supervisor_validation_rejects_unknown_dependencies():
    with pytest.raises(AssertionError, match="not found"):
        Supervisor.validate_supervision_config(
            {
                "monitor_period_ms": 100,
                "request_state_timeout_ms": 500,
                "max_threads": 2,
                "managed_nodes": {
                    "mission": {
                        "node_name": "mission",
                        "node_namespace": "/core",
                        "active_depend": {"missing": "active"},
                    }
                },
            }
        )


def test_supervisor_evaluates_dangling_dependency_chain():
    managed_nodes = {
        "perception": {
            "node_name": "perception",
            "node_namespace": "/core",
        },
        "mission": {
            "node_name": "mission",
            "node_namespace": "/core",
            "config_depend": {"perception": "config"},
            "active_depend": {"perception": "active"},
        },
    }
    supervisor = _make_supervisor(
        managed_nodes,
        {
            "perception": State.PRIMARY_STATE_UNCONFIGURED,
            "mission": State.PRIMARY_STATE_ACTIVE,
        },
    )

    dangling_nodes = supervisor._evaluate_dependency_chain()

    assert ("mission", "active") in dangling_nodes
    assert ("mission", "config") in dangling_nodes


def test_supervisor_reports_failure_when_final_ready_node_transition_fails():
    class _FailingActivationClient:
        @property
        def state(self):
            value = State()
            value.id = State.PRIMARY_STATE_INACTIVE
            value.label = "inactive"
            return value

        @property
        def is_configured(self):
            return True

        @property
        def is_active(self):
            return False

        def request_activate(self):
            return False

    supervisor = _make_supervisor(
        {
            "mission": {
                "node_name": "mission",
                "node_namespace": "/mission",
            }
        },
        {"mission": State.PRIMARY_STATE_INACTIVE},
    )
    supervisor._managed_node_clients["mission"] = _FailingActivationClient()

    success, managed = supervisor._manage_nodes("bringup", "activation")

    assert success is False
    assert managed == [{"key": "mission", "transition": "config"}]


def test_supervisor_build_transition_tree_can_ignore_cross_node_dependencies():
    managed_nodes = {
        "perception": {
            "node_name": "perception",
            "node_namespace": "/core",
        },
        "mission": {
            "node_name": "mission",
            "node_namespace": "/core",
            "active_depend": {"perception": "active"},
        },
    }
    supervisor = _make_supervisor(managed_nodes)

    transition_tree = supervisor._build_transition_tree(
        "bringup",
        "activation",
        ["mission"],
        ignore_dependencies=True,
    )

    assert set(transition_tree) == {"mission_config", "mission_active"}


def test_supervisor_get_ready_nodes_for_bringup_returns_dependency_free_transitions():
    managed_nodes = {
        "perception": {
            "node_name": "perception",
            "node_namespace": "/core",
        },
        "mission": {
            "node_name": "mission",
            "node_namespace": "/core",
            "active_depend": {"perception": "active"},
        },
    }
    supervisor = _make_supervisor(managed_nodes)
    transition_tree = supervisor._build_transition_tree("bringup", "activation", [])

    ready_nodes = supervisor._get_ready_nodes("bringup", transition_tree)

    assert "perception_config" in ready_nodes
    assert "mission_active" not in ready_nodes


def test_supervisor_wait_for_managed_nodes_refreshes_until_available():
    managed_nodes = {
        "perception": {
            "node_name": "perception",
            "node_namespace": "/core",
        },
    }
    supervisor = _make_supervisor(
        managed_nodes,
        {"perception": [State.PRIMARY_STATE_UNKNOWN, State.PRIMARY_STATE_UNCONFIGURED]},
    )

    success, missing = supervisor.wait_for_managed_nodes(timeout_sec=1.0, poll_sec=0.01)

    assert success is True
    assert missing == []
    assert supervisor._managed_node_clients["perception"].refresh_count == 2
