import threading
import time
from types import SimpleNamespace

from lifecycle_msgs.msg import State

from iii_drone_supervision.managed_node_client import ManagedNodeClient


class _FakeLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class _FakeClock:
    def now(self):
        return SimpleNamespace()


class _FakeNode:
    def get_clock(self):
        return _FakeClock()

    def get_logger(self):
        return _FakeLogger()


class _DelayedFuture:
    def __init__(self, response, delay_s):
        self._response = None
        self._callbacks = []
        self._resolved_event = threading.Event()

        def resolve():
            time.sleep(delay_s)
            self._response = response
            self._resolved_event.set()
            for callback in list(self._callbacks):
                callback(self)

        threading.Thread(target=resolve, daemon=True).start()

    def add_done_callback(self, callback):
        self._callbacks.append(callback)
        if self._resolved_event.is_set():
            callback(self)

    def result(self):
        return self._response


class _FakeGetStateClient:
    def __init__(self, response, delay_s):
        self._response = response
        self._delay_s = delay_s
        self.pending_removed = False

    def wait_for_service(self, _timeout):
        return True

    def call_async(self, _request):
        return _DelayedFuture(self._response, self._delay_s)

    def remove_pending_request(self, _future):
        self.pending_removed = True


def _make_state(state_id: int, label: str) -> State:
    state = State()
    state.id = state_id
    state.label = label
    return state


def test_update_state_waits_for_response_within_timeout(monkeypatch):
    monkeypatch.setattr("iii_drone_supervision.managed_node_client.rclpy.ok", lambda: True)

    client = ManagedNodeClient.__new__(ManagedNodeClient)
    client.parent_node = _FakeNode()
    client.monitor_state = True
    client._request_state_timeout_ms = 500
    client._is_transitioning = False
    client._state = _make_state(State.PRIMARY_STATE_UNKNOWN, "UNKNOWN")
    client.long_node_name = "/test_node"
    client.get_state_client = _FakeGetStateClient(
        SimpleNamespace(current_state=_make_state(State.PRIMARY_STATE_UNCONFIGURED, "unconfigured")),
        delay_s=0.2,
    )

    state = client._update_state()

    assert state.id == State.PRIMARY_STATE_UNCONFIGURED
    assert state.label == "unconfigured"
    assert not client.get_state_client.pending_removed


def test_wait_for_state_polls_until_target_state(monkeypatch):
    monkeypatch.setattr("iii_drone_supervision.managed_node_client.rclpy.ok", lambda: True)

    client = ManagedNodeClient.__new__(ManagedNodeClient)
    client._state = _make_state(State.PRIMARY_STATE_UNCONFIGURED, "unconfigured")

    observed_states = iter(
        [
            _make_state(State.PRIMARY_STATE_UNCONFIGURED, "unconfigured"),
            _make_state(State.PRIMARY_STATE_INACTIVE, "inactive"),
        ]
    )

    def fake_update_state(overwrite_timeout_ms=None):
        del overwrite_timeout_ms
        client._state = next(observed_states)
        return client._state

    client._update_state = fake_update_state

    assert client._wait_for_state(
        State.PRIMARY_STATE_INACTIVE,
        State.PRIMARY_STATE_UNCONFIGURED,
        timeout_ms=500,
    )
