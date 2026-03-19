"""Background daemon exposing the III system manager over a Unix socket."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socketserver
import sys

from .system_manager import SystemManager
from .system_spec import resolve_runtime_dir


def default_socket_path() -> Path:
    configured = Path(resolve_runtime_dir()) / "system_manager.sock"
    configured.parent.mkdir(parents=True, exist_ok=True)
    return configured


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
        response = self.server.manager_handle(request)  # type: ignore[attr-defined]
        self.wfile.write((json.dumps(response) + "\n").encode("utf-8"))


def _handle_request(manager: SystemManager, request: dict) -> dict:
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
                "result": manager.shutdown_runtime(
                    select_nodes=request["select_nodes"],
                    include_dependencies=request["include_dependencies"],
                ),
            }
        if command == "status":
            return {"ok": True, "result": manager.status()}
        if command == "list_nodes":
            return {"ok": True, "result": {"managed_nodes": manager.managed_node_ids()}}
        if command == "tmux_spec":
            return {"ok": True, "result": manager.tmux_session_spec()}
        if command == "log_dir":
            return {"ok": True, "result": {"log_dir": manager.log_dir(request["entity_id"])}}
    except Exception as exc:  # pragma: no cover - exercised in integration tests
        return {"ok": False, "error": str(exc)}

    return {"ok": False, "error": f"Unknown command: {command}"}


def serve(socket_path: Path) -> None:
    if socket_path.exists():
        socket_path.unlink()

    manager = SystemManager()
    server = _DaemonServer(str(socket_path), manager)
    server.manager_handle = lambda request: _handle_request(manager, request)  # type: ignore[attr-defined]

    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        server.server_close()
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
