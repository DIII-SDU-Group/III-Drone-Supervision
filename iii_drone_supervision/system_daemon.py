"""Background daemon exposing the III system manager over a Unix socket."""

from __future__ import annotations

import asyncio
import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
import inspect
import json
import os
from pathlib import Path
from queue import Empty, Queue
import signal
import socketserver
import sys
from threading import Event, Thread
import traceback

from .system_manager import SystemManager
from .system_spec import resolve_runtime_dir


def default_socket_path() -> Path:
    configured = Path(resolve_runtime_dir()) / "system_manager.sock"
    configured.parent.mkdir(parents=True, exist_ok=True)
    return configured


@contextmanager
def _daemon_lock(socket_path: Path):
    lock_path = socket_path.with_name(f"{socket_path.stem}.lock")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_file = open(lock_path, "w", encoding="utf-8")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock_file.close()
        raise RuntimeError(f"System daemon is already running; lock is held at {lock_path}") from exc

    try:
        lock_file.write(f"{os.getpid()}\n")
        lock_file.flush()
        yield
    finally:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        lock_file.close()
        try:
            lock_path.unlink()
        except FileNotFoundError:
            pass


class _DaemonServer(socketserver.ThreadingUnixStreamServer):
    allow_reuse_address = True

    def __init__(self, socket_path: str, manager: SystemManager):
        self.manager = manager
        super().__init__(socket_path, _DaemonHandler)


class _DaemonHandler(socketserver.StreamRequestHandler):
    def handle(self):
        raw = self.rfile.readline()
        if not raw:
            return
        request = json.loads(raw.decode("utf-8"))
        response = self.server.dispatch(request)  # type: ignore[attr-defined]
        self.wfile.write((json.dumps(response) + "\n").encode("utf-8"))


@dataclass
class _QueuedRequest:
    request: dict
    done: Event
    response: dict | None = None


class _DaemonRuntime:
    def __init__(self, manager: SystemManager):
        self.manager = manager
        self._requests: Queue[_QueuedRequest] = Queue()
        self._stopped = Event()

    def dispatch(self, request: dict) -> dict:
        queued = _QueuedRequest(request=request, done=Event())
        self._requests.put(queued)
        queued.done.wait()
        assert queued.response is not None
        return queued.response

    async def run(self) -> None:
        while not self._stopped.is_set():
            processed = False
            while True:
                try:
                    queued = self._requests.get_nowait()
                except Empty:
                    break

                queued.response = await _handle_request(self.manager, queued.request)
                queued.done.set()
                processed = True

            if not processed:
                await asyncio.sleep(0.05)

    def stop(self) -> None:
        self._stopped.set()


async def _maybe_await(result):
    if inspect.isawaitable(result):
        return await result
    return result


async def _handle_request(manager: SystemManager, request: dict) -> dict:
    command = request["command"]
    try:
        if command == "ping":
            return {"ok": True, "result": {"booted": manager.booted}}
        if command == "boot":
            return {"ok": True, "result": manager.boot(request["profile"])}
        if command == "start":
            return {
                "ok": True,
                "result": manager.start(
                    activate=request["activate"],
                    select_nodes=request["select_nodes"],
                    include_dependencies=request["include_dependencies"],
                ),
            }
        if command == "stop":
            return {
                "ok": True,
                "result": manager.stop(
                    cleanup=request["cleanup"],
                    select_nodes=request["select_nodes"],
                    include_dependencies=request["include_dependencies"],
                ),
            }
        if command == "restart":
            return {
                "ok": True,
                "result": manager.restart(
                    cold=request["cold"],
                    select_nodes=request["select_nodes"],
                    include_dependencies=request["include_dependencies"],
                ),
            }
        if command == "shutdown":
            return {
                "ok": True,
                "result": await _maybe_await(
                    manager.shutdown_runtime(
                        select_nodes=request["select_nodes"],
                        include_dependencies=request["include_dependencies"],
                    )
                ),
            }
        if command == "status":
            return {"ok": True, "result": manager.status()}
        if command == "list_nodes":
            return {"ok": True, "result": {"managed_nodes": manager.managed_node_ids()}}
        if command == "list_services":
            return {"ok": True, "result": {"services": manager.service_ids()}}
        if command == "service_start":
            return {"ok": True, "result": manager.service_start(request["service_id"])}
        if command == "service_stop":
            return {"ok": True, "result": manager.service_stop(request["service_id"])}
        if command == "service_restart":
            return {"ok": True, "result": manager.service_restart(request["service_id"])}
        if command == "tmux_spec":
            return {"ok": True, "result": manager.tmux_session_spec()}
        if command == "log_dir":
            return {"ok": True, "result": {"log_dir": manager.log_dir(request["entity_id"])}}
    except Exception as exc:  # pragma: no cover - exercised in integration tests
        traceback.print_exc()
        return {"ok": False, "error": str(exc)}

    return {"ok": False, "error": f"Unknown command: {command}"}


def serve(socket_path: Path) -> None:
    with _daemon_lock(socket_path):
        if socket_path.exists():
            socket_path.unlink()

        manager = SystemManager()
        runtime = _DaemonRuntime(manager)
        server = _DaemonServer(str(socket_path), manager)
        server.dispatch = runtime.dispatch  # type: ignore[attr-defined]
        server_thread = Thread(target=server.serve_forever, kwargs={"poll_interval": 0.5}, daemon=True)
        server_thread.start()

        def request_stop(signum, frame):
            del frame
            print(f"System daemon received signal {signum}; stopping.", flush=True)
            runtime.stop()

        previous_handlers = {
            sig: signal.signal(sig, request_stop)
            for sig in (signal.SIGINT, signal.SIGTERM)
        }

        try:
            asyncio.run(runtime.run())
        finally:
            for sig, previous_handler in previous_handlers.items():
                signal.signal(sig, previous_handler)
            runtime.stop()
            server.shutdown()
            server.server_close()
            server_thread.join(timeout=1.0)
            manager.close()
            if socket_path.exists():
                socket_path.unlink()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", default=str(default_socket_path()))
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    serve(Path(args.socket))
    return 0


if __name__ == "__main__":
    sys.exit(main())
