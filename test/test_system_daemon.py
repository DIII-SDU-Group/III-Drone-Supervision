from iii_drone_supervision.system_daemon import _handle_request


class _FakeManager:
    booted = False

    def boot(self, profile):
        return {"profile": profile}

    def start(self, **kwargs):
        return kwargs

    def stop(self, **kwargs):
        return kwargs

    def restart(self, **kwargs):
        return kwargs

    def shutdown_runtime(self, **kwargs):
        return kwargs

    def status(self):
        return {"booted": False}

    def managed_node_ids(self):
        return ["a", "b"]

    def tmux_session_spec(self):
        return {"session_name": "iii_sim", "windows": []}

    def log_dir(self, entity_id):
        return f"/tmp/{entity_id}"


def test_daemon_handle_request_routes_known_commands():
    manager = _FakeManager()

    assert _handle_request(manager, {"command": "ping"})["ok"]
    assert _handle_request(manager, {"command": "boot", "profile": "sim"})["result"] == {"profile": "sim"}
    assert _handle_request(manager, {"command": "list_nodes"})["result"]["managed_nodes"] == ["a", "b"]
    assert _handle_request(manager, {"command": "log_dir", "entity_id": "pl_mapper"})["result"]["log_dir"] == "/tmp/pl_mapper"


def test_daemon_handle_request_reports_unknown_commands():
    manager = _FakeManager()

    response = _handle_request(manager, {"command": "missing"})

    assert not response["ok"]
    assert "Unknown command" in response["error"]
