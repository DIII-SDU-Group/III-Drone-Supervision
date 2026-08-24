import asyncio
import time

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

    def runtime_snapshot(self):
        return {"booted": False, "active": False}

    def managed_node_ids(self):
        return ["a", "b"]

    def service_ids(self):
        return ["micro_ros_agent"]

    def service_start(self, service_id):
        return {"service": service_id, "success": True}

    def service_stop(self, service_id):
        return {"service": service_id, "success": True}

    def service_restart(self, service_id):
        return {"service": service_id, "success": True}

    def tmux_session_spec(self):
        return {"session_name": "iii_sim", "windows": []}

    def log_dir(self, entity_id):
        return f"/tmp/{entity_id}"


def test_daemon_handle_request_routes_known_commands():
    manager = _FakeManager()

    assert asyncio.run(_handle_request(manager, {"command": "ping"}))["ok"]
    assert asyncio.run(_handle_request(manager, {"command": "runtime_status"}))["result"] == {
        "booted": False,
        "active": False,
    }
    assert asyncio.run(_handle_request(manager, {"command": "boot", "profile": "sim"}))["result"] == {"profile": "sim"}
    assert asyncio.run(_handle_request(manager, {"command": "list_nodes"}))["result"]["managed_nodes"] == ["a", "b"]
    assert (
        asyncio.run(_handle_request(manager, {"command": "list_services"}))["result"]["services"]
        == ["micro_ros_agent"]
    )
    assert asyncio.run(
        _handle_request(manager, {"command": "service_start", "service_id": "micro_ros_agent"})
    )["result"] == {"service": "micro_ros_agent", "success": True}
    assert (
        asyncio.run(_handle_request(manager, {"command": "log_dir", "entity_id": "pl_mapper"}))["result"]["log_dir"]
        == "/tmp/pl_mapper"
    )


def test_daemon_handle_request_reports_unknown_commands():
    manager = _FakeManager()

    response = asyncio.run(_handle_request(manager, {"command": "missing"}))

    assert not response["ok"]
    assert "Unknown command" in response["error"]


def test_blocking_start_does_not_starve_daemon_asyncio_loop():
    class _BlockingManager(_FakeManager):
        def start(self, **kwargs):
            time.sleep(0.15)
            return kwargs

    async def exercise():
        started_at = time.monotonic()
        request = asyncio.create_task(
            _handle_request(
                _BlockingManager(),
                {
                    "command": "start",
                    "activate": True,
                    "select_nodes": [],
                    "include_dependencies": False,
                },
            )
        )
        await asyncio.sleep(0.02)
        loop_delay = time.monotonic() - started_at
        response = await request
        return loop_delay, response

    loop_delay, response = asyncio.run(exercise())

    assert loop_delay < 0.1
    assert response["ok"] is True
