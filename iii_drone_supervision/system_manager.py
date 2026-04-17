"""Daemon-owned III system manager built on ROS 2 launch and lifecycle APIs."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from threading import Lock, Thread
import time
import os

from launch import LaunchDescription, LaunchService
from launch.actions import GroupAction, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.actions import RegisterEventHandler
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from lifecycle_msgs.msg import State

from .service_manager import ServiceProcess
from .supervisor import Supervisor
from .system_spec import entity_log_dir, get_system_profile, resolve_ros_params_file
from .tmux_spec import get_tmux_session_spec


@dataclass
class EntityRuntimeState:
    entity_id: str
    alive: bool = False
    start_count: int = 0
    exit_count: int = 0
    generation: int = 0
    pid: int | None = None
    current_log_path: str | None = None


class SystemManager:
    """Owns launch runtime, lifecycle orchestration, and CLI-facing status."""

    def __init__(self):
        self._lock = Lock()
        self._booted = False
        self._profile_name: str | None = None

        self._launch_service: LaunchService | None = None
        self._launch_task: asyncio.Task | None = None
        self._node: Node | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._executor_thread: Thread | None = None
        self._supervisor: Supervisor | None = None

        self._entity_states: dict[str, EntityRuntimeState] = {}
        self._log_dirs: dict[str, str] = {}
        self._launch_generation = 0
        self._service_runtimes: dict[str, ServiceProcess] = {}

        self._ensure_ros_runtime()

    def _ensure_ros_runtime(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)

        if self._node is None:
            self._node = Node("system_manager", namespace="/supervision")
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._executor_thread = Thread(target=self._executor.spin, daemon=True)
            self._executor_thread.start()

    @property
    def booted(self) -> bool:
        return self._booted

    def _build_launch_description(self, profile_name: str, generation: int) -> LaunchDescription:
        profile = get_system_profile(profile_name)
        entities = []
        self._entity_states = {}
        self._log_dirs = {}
        active_parameter_file = resolve_ros_params_file(profile_name)

        for entity in profile.entities:
            log_dir = entity_log_dir(profile_name, entity.entity_id)
            log_dir.mkdir(parents=True, exist_ok=True)
            self._entity_states[entity.entity_id] = EntityRuntimeState(entity_id=entity.entity_id)
            self._log_dirs[entity.entity_id] = str(log_dir)

            target_action = entity.launch_factory(profile_name)
            action = GroupAction(
                [
                    SetEnvironmentVariable("ROS_LOG_DIR", str(log_dir)),
                    SetEnvironmentVariable("III_SYSTEM_PARAMETER_FILE", active_parameter_file),
                    target_action,
                ]
            )
            entities.append(action)
            entities.append(
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=target_action,
                        on_start=self._make_process_started_callback(entity.entity_id, generation, log_dir),
                    )
                )
            )
            entities.append(
                RegisterEventHandler(
                    OnProcessIO(
                        target_action=target_action,
                        on_stdout=self._make_process_io_callback(entity.entity_id, generation, log_dir, "stdout"),
                        on_stderr=self._make_process_io_callback(entity.entity_id, generation, log_dir, "stderr"),
                    )
                )
            )
            entities.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=target_action,
                        on_exit=self._make_process_exited_callback(entity.entity_id, generation, log_dir),
                    )
                )
            )

        return LaunchDescription(entities)

    def _build_services(self, profile_name: str) -> None:
        profile = get_system_profile(profile_name)
        for service in profile.services:
            log_dir = entity_log_dir(profile_name, service.service_id)
            log_dir.mkdir(parents=True, exist_ok=True)
            self._log_dirs[service.service_id] = str(log_dir)
            self._service_runtimes[service.service_id] = ServiceProcess(
                service,
                profile_name,
                self._node,
                log_dir,
            )

    @staticmethod
    def _write_log_file(path, text: str | bytes, *, append: bool = True) -> None:
        if isinstance(text, str):
            payload = text.encode("utf-8", errors="replace")
        else:
            payload = text
        mode = "ab" if append else "wb"
        with open(path, mode) as file:
            file.write(payload)
            if payload and not payload.endswith(b"\n"):
                file.write(b"\n")

    @classmethod
    def _append_process_log(cls, log_dir, text: str | bytes) -> None:
        cls._write_log_file(log_dir / "process.log", text)

    @classmethod
    def _write_current_log(cls, log_dir, text: str | bytes, *, append: bool = True) -> None:
        cls._write_log_file(log_dir / "current.log", text, append=append)

    @staticmethod
    def _run_separator(kind: str, *, entity_id: str, generation: int, pid: int, returncode: int | None = None) -> str:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S %z", time.localtime())
        line = "=" * 88
        fields = f"entity={entity_id} generation={generation} pid={pid}"
        if returncode is not None:
            fields += f" returncode={returncode}"
        return f"\n{line}\n[system_manager] RUN {kind}: {fields} time={timestamp}\n{line}\n"

    def _make_process_started_callback(self, entity_id: str, generation: int, log_dir):
        def callback(event, context):
            del context
            header = self._run_separator("START", entity_id=entity_id, generation=generation, pid=event.pid)
            with self._lock:
                state = self._entity_states[entity_id]
                state.generation = generation
                state.pid = event.pid
                state.alive = True
                state.start_count += 1
                state.current_log_path = str(log_dir / "current.log")
            self._append_process_log(log_dir, header)
            self._write_current_log(log_dir, header, append=False)
            return None

        return callback

    def _make_process_io_callback(self, entity_id: str, generation: int, log_dir, stream_name: str):
        def callback(event):
            self._append_process_log(log_dir, event.text)
            with self._lock:
                state = self._entity_states.get(entity_id)
                write_current = state is not None and state.generation == generation and state.current_log_path is not None
            if write_current:
                self._write_current_log(log_dir, event.text)
            return None

        return callback

    def _make_process_exited_callback(self, entity_id: str, generation: int, log_dir):
        def callback(event, context):
            del context
            footer = self._run_separator(
                "END",
                entity_id=entity_id,
                generation=generation,
                pid=event.pid,
                returncode=event.returncode,
            )
            self._append_process_log(log_dir, footer)
            write_current = False
            with self._lock:
                state = self._entity_states[entity_id]
                if state.generation != generation or state.pid != event.pid:
                    return None
                write_current = state.current_log_path is not None
                state.alive = False
                state.pid = None
                state.exit_count += 1
            if write_current:
                self._write_current_log(log_dir, footer)
            return None

        return callback

    def boot(self, profile_name: str) -> dict:
        with self._lock:
            if self._booted:
                return {
                    "booted": True,
                    "profile": self._profile_name,
                    "tmux": self.tmux_session_spec(),
                }

            profile = get_system_profile(profile_name)
            self._profile_name = profile.name
            self._launch_generation += 1
            generation = self._launch_generation
            self._launch_service = LaunchService()
            launch_description = self._build_launch_description(profile.name, generation)
            self._build_services(profile.name)
            self._launch_service.include_launch_description(launch_description)
            self._launch_task = asyncio.get_running_loop().create_task(self._launch_service.run_async())
            self._supervisor = Supervisor(
                profile.build_supervision_config(),
                monitor_node_states=False,
                node=self._node,
            )
            self._booted = True

        time.sleep(1.0)

        return {
            "booted": True,
            "profile": self._profile_name,
            "tmux": self.tmux_session_spec(),
        }

    def _require_booted(self) -> None:
        if not self._booted or self._supervisor is None:
            raise RuntimeError("System is not booted.")

    def _service_statuses(self) -> dict[str, dict]:
        statuses = {}
        for service_id, service in getattr(self, "_service_runtimes", {}).items():
            snapshot = service.snapshot()
            statuses[service_id] = {
                "alive": snapshot.alive,
                "ready": snapshot.ready,
                "starts": snapshot.start_count,
                "exits": snapshot.exit_count,
                "pid": snapshot.pid,
                "generation": snapshot.generation,
                "last_returncode": snapshot.last_returncode,
                "reason": snapshot.ready_reason,
                "command": snapshot.command,
            }
        return statuses

    def _start_profile_services(self, selected_nodes: list[str]) -> dict:
        profile_name = getattr(self, "_profile_name", None)
        if profile_name is None or not getattr(self, "_service_runtimes", {}):
            return {}

        profile = get_system_profile(profile_name)
        dependencies = profile.service_dependencies()
        required_service_ids = set()

        if selected_nodes:
            for node_id in selected_nodes:
                required_service_ids.update(dependencies.get(node_id, {}).keys())
        else:
            required_service_ids.update(service.service_id for service in profile.services if service.autostart)

        started = {}
        for service_id in sorted(required_service_ids):
            service = self._service_runtimes.get(service_id)
            if service is None:
                started[service_id] = {"success": False, "error": f"Unknown service: {service_id}"}
                continue

            start_result = service.start()
            service.wait_ready(service.spec.ready_timeout_sec)
            snapshot = service.snapshot()
            started[service_id] = {
                **start_result,
                "alive": snapshot.alive,
                "ready": snapshot.ready,
                "reason": snapshot.ready_reason,
            }

        return started

    def _nodes_blocked_by_services(self, selected_nodes: list[str]) -> dict[str, dict[str, str]]:
        profile_name = getattr(self, "_profile_name", None)
        if profile_name is None or not getattr(self, "_service_runtimes", {}):
            return {}

        profile = get_system_profile(profile_name)
        dependencies = profile.service_dependencies()
        considered_nodes = selected_nodes or list(profile.build_supervision_config()["managed_nodes"].keys())
        statuses = self._service_statuses()
        blocked: dict[str, dict[str, str]] = {}

        for node_id in considered_nodes:
            for service_id, required_state in dependencies.get(node_id, {}).items():
                service_status = statuses.get(service_id)
                if service_status is None:
                    blocked.setdefault(node_id, {})[service_id] = "service is not loaded"
                    continue
                if required_state == "running":
                    if not service_status["alive"]:
                        blocked.setdefault(node_id, {})[service_id] = service_status["reason"]
                elif required_state == "ready":
                    if not service_status["ready"]:
                        blocked.setdefault(node_id, {})[service_id] = service_status["reason"]
                else:
                    blocked.setdefault(node_id, {})[service_id] = f"unsupported required state: {required_state}"

        return blocked

    @staticmethod
    def _format_service_blocks(blocked_nodes: dict[str, dict[str, str]]) -> str:
        parts = []
        for node_id, service_errors in sorted(blocked_nodes.items()):
            service_text = ", ".join(
                f"{service_id}: {reason}" for service_id, reason in sorted(service_errors.items())
            )
            parts.append(f"{node_id} blocked by {service_text}")
        return "; ".join(parts)

    def _wait_for_lifecycle_nodes(self, node_keys: list[str]) -> tuple[bool, list[str]]:
        assert self._supervisor is not None
        try:
            return self._supervisor.wait_for_managed_nodes(node_keys=node_keys)
        except TypeError:
            return self._supervisor.wait_for_managed_nodes()

    def start(self, *, activate: bool, select_nodes: list[str], include_dependencies: bool) -> dict:
        self._require_booted()
        assert self._supervisor is not None

        service_results = self._start_profile_services(select_nodes)
        blocked_nodes = self._nodes_blocked_by_services(select_nodes)
        if select_nodes and blocked_nodes:
            return {
                "success": False,
                "managed_nodes": [],
                "services": service_results,
                "blocked_nodes": blocked_nodes,
                "error": "Selected nodes are blocked by unavailable services: "
                + self._format_service_blocks(blocked_nodes),
            }

        effective_select_nodes = select_nodes
        forced_dependency_expansion = False
        if not select_nodes and blocked_nodes:
            all_nodes = self.managed_node_ids()
            effective_select_nodes = [node_id for node_id in all_nodes if node_id not in blocked_nodes]
            forced_dependency_expansion = True
            if not effective_select_nodes:
                return {
                    "success": False,
                    "managed_nodes": [],
                    "services": service_results,
                    "blocked_nodes": blocked_nodes,
                    "error": "All managed nodes are blocked by unavailable services: "
                    + self._format_service_blocks(blocked_nodes),
                }

        ready, missing_nodes = self._wait_for_lifecycle_nodes(effective_select_nodes)
        if not ready:
            return {
                "success": False,
                "managed_nodes": [],
                "services": service_results,
                "blocked_nodes": blocked_nodes,
                "error": "Timed out waiting for lifecycle services from: " + ", ".join(sorted(missing_nodes)),
            }
        success, managed = self._supervisor.start(
            activate=activate,
            select_nodes=effective_select_nodes,
            ignore_dependencies=not (include_dependencies or forced_dependency_expansion),
        )
        result = {
            "success": success,
            "managed_nodes": managed,
            "services": service_results,
            "blocked_nodes": blocked_nodes,
        }
        if not success:
            result["error"] = self._format_start_failure(activate=activate, ignored_nodes=set(blocked_nodes))
        elif blocked_nodes:
            result["degraded"] = True
            result["warning"] = (
                "System start completed with service-blocked nodes left inactive: "
                + self._format_service_blocks(blocked_nodes)
            )
        return result

    def stop(self, *, cleanup: bool, select_nodes: list[str], include_dependencies: bool) -> dict:
        self._require_booted()
        assert self._supervisor is not None
        self._supervisor._get_node_states()
        success, managed = self._supervisor.stop(
            cleanup=cleanup,
            select_nodes=select_nodes,
            ignore_dependencies=not include_dependencies,
        )
        result = {"success": success, "managed_nodes": managed}
        if not success:
            result["error"] = self._format_stop_failure(cleanup=cleanup)
        elif not select_nodes:
            service_results = {}
            for service_id, service in sorted(getattr(self, "_service_runtimes", {}).items()):
                service_results[service_id] = service.stop()
            result["services"] = service_results
        return result

    def restart(self, *, cold: bool, select_nodes: list[str], include_dependencies: bool) -> dict:
        self._require_booted()
        stop_result = self.stop(
            cleanup=cold,
            select_nodes=select_nodes,
            include_dependencies=include_dependencies,
        )
        if not stop_result["success"]:
            return stop_result
        return self.start(
            activate=True,
            select_nodes=select_nodes,
            include_dependencies=include_dependencies,
        )

    async def shutdown_runtime(self, *, select_nodes: list[str], include_dependencies: bool) -> dict:
        if not self._booted:
            return {"success": True, "message": "System runtime is not booted."}

        assert self._supervisor is not None
        self._supervisor.shutdown(
            select_nodes=select_nodes,
            ignore_dependencies=not include_dependencies,
        )
        if self._launch_service is not None:
            shutdown_coroutine = self._launch_service.shutdown()
            if shutdown_coroutine is not None:
                await shutdown_coroutine
        if self._launch_task is not None:
            try:
                await asyncio.wait_for(self._launch_task, timeout=10.0)
            except asyncio.TimeoutError:
                self._launch_task.cancel()
                await asyncio.gather(self._launch_task, return_exceptions=True)
        if self._supervisor is not None:
            self._supervisor.destroy()
        for service in self._service_runtimes.values():
            service.destroy()
        with self._lock:
            self._booted = False
            self._launch_service = None
            self._launch_task = None
            self._supervisor = None
            self._service_runtimes = {}
            for state in self._entity_states.values():
                state.alive = False
                state.pid = None
        return {"success": True}

    def managed_node_ids(self) -> list[str]:
        profile_name = self._profile_name or os.environ.get("III_SYSTEM_PROFILE", "sim")
        return list(get_system_profile(profile_name).build_supervision_config()["managed_nodes"].keys())

    def service_ids(self) -> list[str]:
        profile_name = self._profile_name or os.environ.get("III_SYSTEM_PROFILE", "sim")
        return [service.service_id for service in get_system_profile(profile_name).services]

    @staticmethod
    def _state_label(state: State) -> str:
        label = getattr(state, "label", "")
        if label:
            return label
        return f"id={getattr(state, 'id', 'unknown')}"

    def _format_start_failure(self, *, activate: bool, ignored_nodes: set[str] | None = None) -> str:
        assert self._supervisor is not None
        ignored_nodes = ignored_nodes or set()
        states = self._supervisor._get_node_states()
        if activate:
            failed = {
                key: self._state_label(state)
                for key, state in states.items()
                if key not in ignored_nodes and state.id != State.PRIMARY_STATE_ACTIVE
            }
            target = "ACTIVE"
        else:
            failed = {
                key: self._state_label(state)
                for key, state in states.items()
                if key not in ignored_nodes
                and state.id not in (State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE)
            }
            target = "configured"

        if not failed:
            return "System start failed, but all managed node states reached the requested target. Check `iii system logs daemon`."

        failed_text = ", ".join(f"{key}={state}" for key, state in sorted(failed.items()))
        return (
            f"System start failed before all managed nodes reached {target}. "
            f"Nodes not at target: {failed_text}. "
            "Check the node pane or run `iii system logs <node>` and `iii system logs daemon`."
        )

    def _format_stop_failure(self, *, cleanup: bool) -> str:
        assert self._supervisor is not None
        states = self._supervisor._get_node_states()
        if cleanup:
            failed = {
                key: self._state_label(state)
                for key, state in states.items()
                if state.id not in (State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_FINALIZED)
            }
            target = "UNCONFIGURED"
        else:
            failed = {
                key: self._state_label(state)
                for key, state in states.items()
                if state.id == State.PRIMARY_STATE_ACTIVE
            }
            target = "not ACTIVE"

        if not failed:
            return "System stop failed, but all managed node states reached the requested target. Check `iii system logs daemon`."

        failed_text = ", ".join(f"{key}={state}" for key, state in sorted(failed.items()))
        return (
            f"System stop failed before all managed nodes reached {target}. "
            f"Nodes not at target: {failed_text}. "
            "Check the node pane or run `iii system logs <node>` and `iii system logs daemon`."
        )

    def status(self) -> dict:
        managed_nodes: dict[str, str] = {}
        if self._booted and self._supervisor is not None:
            states = self._supervisor._get_node_states()
            for key, state in states.items():
                managed_nodes[key] = state.label
        return {
            "booted": self._booted,
            "profile": self._profile_name,
            "managed_nodes": managed_nodes,
            "services": self._service_statuses(),
            "processes": {
                key: {
                    "alive": state.alive,
                    "start_count": state.start_count,
                    "exit_count": state.exit_count,
                }
                for key, state in self._entity_states.items()
            },
        }

    def tmux_session_spec(self) -> dict:
        profile_name = self._profile_name or os.environ.get("III_SYSTEM_PROFILE", "sim")
        spec = get_tmux_session_spec(profile_name)
        windows = []
        for window in spec.windows:
            panes = []
            for pane in window.panes:
                if pane.mode == "logs" and pane.target is not None:
                    command = f"iii system logs {pane.target} --follow"
                elif pane.mode == "status":
                    command = "iii system status --watch"
                else:
                    command = pane.command or "bash"
                panes.append({"title": pane.title, "command": command})
            windows.append({"name": window.name, "layout": window.layout, "panes": panes})
        return {
            "session_name": spec.session_name,
            "startup_window": spec.startup_window,
            "windows": windows,
        }

    def log_dir(self, entity_id: str) -> str:
        if entity_id not in self._log_dirs:
            raise KeyError(f"Unknown entity: {entity_id}")
        return self._log_dirs[entity_id]

    def service_start(self, service_id: str) -> dict:
        self._require_booted()
        if service_id not in self._service_runtimes:
            raise KeyError(f"Unknown service: {service_id}")
        result = self._service_runtimes[service_id].start()
        snapshot = self._service_runtimes[service_id].snapshot()
        return {
            **result,
            "service": service_id,
            "alive": snapshot.alive,
            "ready": snapshot.ready,
            "reason": snapshot.ready_reason,
        }

    def service_stop(self, service_id: str) -> dict:
        self._require_booted()
        if service_id not in self._service_runtimes:
            raise KeyError(f"Unknown service: {service_id}")
        result = self._service_runtimes[service_id].stop()
        snapshot = self._service_runtimes[service_id].snapshot()
        return {
            **result,
            "service": service_id,
            "alive": snapshot.alive,
            "ready": snapshot.ready,
            "reason": snapshot.ready_reason,
        }

    def service_restart(self, service_id: str) -> dict:
        self._require_booted()
        if service_id not in self._service_runtimes:
            raise KeyError(f"Unknown service: {service_id}")
        result = self._service_runtimes[service_id].restart()
        snapshot = self._service_runtimes[service_id].snapshot()
        return {
            **result,
            "service": service_id,
            "alive": snapshot.alive,
            "ready": snapshot.ready,
            "reason": snapshot.ready_reason,
        }

    def close(self) -> None:
        if self._booted:
            if self._launch_service is not None:
                self._launch_service.shutdown(force_sync=True)
            if self._supervisor is not None:
                self._supervisor.destroy()
            for service in self._service_runtimes.values():
                service.destroy()
            self._booted = False
            self._service_runtimes = {}
        if self._executor is not None and self._node is not None:
            self._executor.remove_node(self._node)
            self._executor.shutdown(timeout_sec=1.0)
        if self._node is not None:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
