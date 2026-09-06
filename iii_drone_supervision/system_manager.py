"""Daemon-owned III system manager built on ROS 2 launch and lifecycle APIs."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from threading import Lock, Thread
import time
import os
import signal
import traceback

from launch import LaunchDescription, LaunchService
from launch.actions import GroupAction, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.actions import RegisterEventHandler
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from lifecycle_msgs.msg import State

from .service_manager import ServiceProcess
from .supervisor import Supervisor
from .system_spec import entity_log_dir, get_system_profile, resolve_ros_params_file
from .tmux_spec import get_tmux_session_spec
from .log_retention import DEFAULT_ENTITY_LOG_MAX_BYTES, configured_max_bytes, write_bounded_log

try:
    from iii_drone_interfaces.msg import SubsystemHealthStatus, SystemHealthStatus
except Exception:  # pragma: no cover - allows host-side unit tests before interfaces are built
    SubsystemHealthStatus = None
    SystemHealthStatus = None


SYSTEM_HEALTH_TOPIC = "/supervision/system_health"


def system_health_qos() -> QoSProfile:
    """Retain the current system state for runtime clients that start later."""
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


@dataclass
class EntityRuntimeState:
    entity_id: str
    alive: bool = False
    start_count: int = 0
    exit_count: int = 0
    generation: int = 0
    pid: int | None = None
    current_log_path: str | None = None
    desired_active: bool = False
    recovery_in_progress: bool = False


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

        if getattr(self, "_node", None) is None:
            self._node = Node("system_manager", namespace="/supervision")
            self._health_publisher = None
            self._setup_health_publisher()

        if getattr(self, "_executor_thread", None) is not None and self._executor_thread.is_alive():
            return

        if getattr(self, "_executor", None) is not None:
            try:
                self._executor.shutdown(timeout_sec=0.1)
            except Exception:
                pass

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)

        def spin_executor() -> None:
            while rclpy.ok() and self._executor is not None:
                try:
                    self._executor.spin()
                    return
                except Exception as exc:
                    print(
                        f"[system_manager] ROS executor spin failed: {type(exc).__name__}: {exc}",
                        flush=True,
                    )
                    traceback.print_exc()
                    time.sleep(0.2)

        self._executor_thread = Thread(target=spin_executor, daemon=True)
        self._executor_thread.start()

    def _setup_health_publisher(self) -> None:
        if SystemHealthStatus is None or self._node is None:
            return
        self._health_publisher = self._node.create_publisher(
            SystemHealthStatus,
            SYSTEM_HEALTH_TOPIC,
            system_health_qos(),
        )

    def publish_health_status(self) -> None:
        publisher = getattr(self, "_health_publisher", None)
        if publisher is None:
            return
        try:
            publisher.publish(self.health_status_message())
        except Exception as exc:  # pragma: no cover - defensive runtime path
            print(f"[system_manager] failed to publish health status: {exc}", flush=True)

    def _return_with_health(self, result: dict) -> dict:
        self.publish_health_status()
        return result

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
        write_bounded_log(
            path,
            text,
            append=append,
            max_bytes=configured_max_bytes(
                "III_SYSTEM_ENTITY_LOG_MAX_BYTES",
                DEFAULT_ENTITY_LOG_MAX_BYTES,
            ),
        )

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
            recover_after_respawn = False
            with self._lock:
                state = self._entity_states[entity_id]
                recover_after_respawn = state.start_count > 0 and state.desired_active
                state.generation = generation
                state.pid = event.pid
                state.alive = True
                state.start_count += 1
                state.current_log_path = str(log_dir / "current.log")
                state.recovery_in_progress = recover_after_respawn
            self._append_process_log(log_dir, header)
            self._write_current_log(log_dir, header, append=False)
            if recover_after_respawn:
                Thread(
                    target=self._recover_respawned_entity,
                    args=(entity_id, generation, event.pid, log_dir),
                    daemon=True,
                ).start()
            self.publish_health_status()
            return None

        return callback

    def _recover_respawned_entity(self, entity_id: str, generation: int, pid: int, log_dir) -> None:
        """Restore an unexpectedly respawned lifecycle node to its desired Active state."""
        deadline = time.monotonic() + 45.0
        result_message = ""

        try:
            while time.monotonic() < deadline:
                with self._lock:
                    state = self._entity_states.get(entity_id)
                    still_current = bool(
                        state
                        and state.alive
                        and state.generation == generation
                        and state.pid == pid
                        and state.desired_active
                    )
                if not still_current:
                    result_message = "automatic lifecycle recovery cancelled because the respawned process is no longer desired"
                    return

                ready, _ = self._wait_for_lifecycle_nodes([entity_id], timeout_sec=2.0)
                if ready:
                    assert self._supervisor is not None
                    success, managed = self._supervisor.start(
                        activate=True,
                        select_nodes=[entity_id],
                        ignore_dependencies=False,
                    )
                    result_message = (
                        f"automatic lifecycle recovery {'succeeded' if success else 'failed'} "
                        f"for respawned entity {entity_id}: {managed}"
                    )
                    return

                time.sleep(0.5)

            result_message = f"automatic lifecycle recovery timed out for respawned entity {entity_id}"
        except Exception as exc:  # pragma: no cover - defensive runtime path
            result_message = (
                f"automatic lifecycle recovery raised {type(exc).__name__} "
                f"for respawned entity {entity_id}: {exc}"
            )
        finally:
            with self._lock:
                state = self._entity_states.get(entity_id)
                if state is not None and state.generation == generation and state.pid == pid:
                    state.recovery_in_progress = False
            if result_message:
                line = f"[system_manager] {result_message}"
                self._append_process_log(log_dir, line)
                self._write_current_log(log_dir, line)
                print(line, flush=True)
            self.publish_health_status()

    def _set_desired_active(self, entity_ids: list[str], desired_active: bool) -> None:
        states = getattr(self, "_entity_states", {})
        lock = getattr(self, "_lock", None)

        def update_states() -> None:
            for entity_id in entity_ids:
                state = states.get(entity_id)
                if state is not None:
                    state.desired_active = desired_active

        if lock is None:
            update_states()
        else:
            with lock:
                update_states()

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
            self.publish_health_status()
            return None

        return callback

    def boot(self, profile_name: str) -> dict:
        self._ensure_ros_runtime()
        with self._lock:
            if self._booted:
                if self._profile_name != profile_name:
                    raise RuntimeError(
                        f"System is already booted with profile {self._profile_name}; "
                        f"shut it down before booting profile {profile_name}."
                    )
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

        return self._return_with_health({
            "booted": True,
            "profile": self._profile_name,
            "tmux": self.tmux_session_spec(),
        })

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

    def _start_profile_services(self, selected_nodes: list[str], *, wait_ready: bool = True) -> dict:
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
            if wait_ready:
                service.wait_ready(service.spec.ready_timeout_sec)
            snapshot = service.snapshot()
            started[service_id] = {
                **start_result,
                "alive": snapshot.alive,
                "ready": snapshot.ready,
                "reason": snapshot.ready_reason,
            }

        return started

    def _wait_for_service_blocks(self, blocked_nodes: dict[str, dict[str, str]], service_results: dict) -> dict:
        service_ids = {
            service_id
            for service_errors in blocked_nodes.values()
            for service_id in service_errors
            if service_id in getattr(self, "_service_runtimes", {})
        }
        for service_id in sorted(service_ids):
            service = self._service_runtimes[service_id]
            service.wait_ready(service.spec.ready_timeout_sec)
            snapshot = service.snapshot()
            if snapshot.alive and not snapshot.ready:
                restart_result = service.restart()
                service.wait_ready(service.spec.ready_timeout_sec)
                snapshot = service.snapshot()
                service_results.setdefault(service_id, {})["readiness_restart"] = restart_result
            existing = service_results.get(service_id, {})
            service_results[service_id] = {
                **existing,
                "alive": snapshot.alive,
                "ready": snapshot.ready,
                "reason": snapshot.ready_reason,
            }
        return service_results

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

    def _wait_for_lifecycle_nodes(self, node_keys: list[str], timeout_sec: float = 15.0) -> tuple[bool, list[str]]:
        assert self._supervisor is not None
        try:
            return self._supervisor.wait_for_managed_nodes(node_keys=node_keys, timeout_sec=timeout_sec)
        except TypeError:
            return self._supervisor.wait_for_managed_nodes()

    def start(self, *, activate: bool, select_nodes: list[str], include_dependencies: bool) -> dict:
        self._ensure_ros_runtime()
        self._require_booted()
        assert self._supervisor is not None

        service_results = self._start_profile_services(select_nodes, wait_ready=bool(select_nodes))
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
        prestarted_managed: list[dict] = []
        prestart_failed = False
        prestart_error: str | None = None
        if not select_nodes and blocked_nodes:
            initial_blocked_nodes = set(blocked_nodes)
            all_nodes = self.managed_node_ids()
            unblocked_nodes = [node_id for node_id in all_nodes if node_id not in initial_blocked_nodes]
            forced_dependency_expansion = True
            if unblocked_nodes:
                ready, missing_nodes = self._wait_for_lifecycle_nodes(unblocked_nodes, timeout_sec=45.0)
                if not ready:
                    prestart_failed = True
                    prestart_error = (
                        "Deferred unblocked-node startup while waiting for service dependencies; "
                        "lifecycle services not yet available from: " + ", ".join(sorted(missing_nodes))
                    )
                else:
                    success, started = self._supervisor.start(
                        activate=activate,
                        select_nodes=unblocked_nodes,
                        ignore_dependencies=False,
                    )
                    prestarted_managed.extend(started)
                    if not success:
                        prestart_failed = True
                        prestart_error = self._format_start_failure(
                            activate=activate,
                            ignored_nodes=initial_blocked_nodes,
                            selected_nodes=set(unblocked_nodes),
                        )

            self._wait_for_service_blocks(blocked_nodes, service_results)
            blocked_nodes = self._nodes_blocked_by_services([])

            if blocked_nodes:
                return {
                    "success": False,
                    "managed_nodes": prestarted_managed,
                    "services": service_results,
                    "blocked_nodes": blocked_nodes,
                    "error": "Managed nodes are blocked by unavailable services after waiting: "
                    + self._format_service_blocks(blocked_nodes),
                }

            delayed_nodes = [node_id for node_id in all_nodes if node_id in initial_blocked_nodes]
            if prestart_failed:
                effective_select_nodes = all_nodes
            elif delayed_nodes:
                effective_select_nodes = delayed_nodes
            else:
                result = {
                    "success": True,
                    "managed_nodes": prestarted_managed,
                    "services": service_results,
                    "blocked_nodes": {},
                }
                self._set_desired_active(all_nodes, activate)
                return self._return_with_health(result)

        ready, missing_nodes = self._wait_for_lifecycle_nodes(effective_select_nodes, timeout_sec=60.0)
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
        if forced_dependency_expansion:
            managed = [*prestarted_managed, *managed]
        result = {
            "success": success,
            "managed_nodes": managed,
            "services": service_results,
            "blocked_nodes": blocked_nodes,
        }
        if not success:
            result["error"] = self._format_start_failure(
                activate=activate,
                ignored_nodes=set(blocked_nodes),
                selected_nodes=set(effective_select_nodes) if effective_select_nodes else None,
            )
            if prestart_error:
                result["prestart_error"] = prestart_error
        elif prestart_error:
            result["warning"] = "Initial unblocked-node startup failed before service dependencies became ready; retried full startup after services were ready."
            result["prestart_error"] = prestart_error
        if success:
            desired_nodes = [
                node["key"]
                for node in managed
                if isinstance(node, dict) and isinstance(node.get("key"), str)
            ]
            if not desired_nodes:
                desired_nodes = select_nodes
            self._set_desired_active(desired_nodes, activate)
        return self._return_with_health(result)

    def stop(self, *, cleanup: bool, select_nodes: list[str], include_dependencies: bool) -> dict:
        self._ensure_ros_runtime()
        self._require_booted()
        assert self._supervisor is not None
        desired_nodes = select_nodes or self.managed_node_ids()
        self._set_desired_active(desired_nodes, False)
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
        return self._return_with_health(result)

    async def restart(self, *, cold: bool, select_nodes: list[str], include_dependencies: bool) -> dict:
        self._require_booted()
        if cold and not select_nodes:
            profile_name = self._profile_name or os.environ.get("III_SYSTEM_PROFILE", "sim")
            shutdown_result = await self.shutdown_runtime(
                select_nodes=[],
                include_dependencies=include_dependencies,
            )
            if not shutdown_result.get("success"):
                shutdown_result.setdefault("error", "System shutdown failed during cold full restart.")
                return shutdown_result

            boot_result = self.boot(profile_name)
            if not boot_result.get("booted"):
                return {
                    "success": False,
                    "shutdown": shutdown_result,
                    "boot": boot_result,
                    "error": "System boot failed during cold full restart.",
                }

            await asyncio.sleep(1.0)
            loop = asyncio.get_running_loop()
            start_timeout_sec = float(os.environ.get("III_SYSTEM_FULL_RESTART_START_TIMEOUT_SEC", "240"))
            start_deadline = time.monotonic() + start_timeout_sec
            start_attempts: list[dict] = []
            start_result: dict = {}
            while True:
                start_result = await loop.run_in_executor(
                    None,
                    lambda: self.start(
                        activate=True,
                        select_nodes=[],
                        include_dependencies=include_dependencies,
                    ),
                )
                start_attempts.append({
                    "success": bool(start_result.get("success")),
                    "error": start_result.get("error"),
                    "blocked_nodes": start_result.get("blocked_nodes", {}),
                })
                if start_result.get("success"):
                    break
                if time.monotonic() >= start_deadline:
                    break
                error_text = str(start_result.get("error", ""))
                if (
                    "Timed out waiting for lifecycle services" not in error_text
                    and "blocked by unavailable services" not in error_text
                    and "blocked by" not in error_text
                ):
                    break
                time.sleep(2.0)

            start_result["shutdown"] = shutdown_result
            start_result["boot"] = boot_result
            start_result["cold_runtime_restart"] = True
            start_result["start_attempts"] = start_attempts
            if not start_result.get("success") and start_attempts:
                start_result.setdefault(
                    "error",
                    f"Cold full restart did not reach active state within {start_timeout_sec}s.",
                )
            return start_result

        stop_result = self.stop(
            cleanup=cold,
            select_nodes=select_nodes,
            include_dependencies=include_dependencies,
        )
        if not stop_result["success"]:
            return stop_result
        process_restart_result = {}
        if cold and select_nodes:
            process_restart_result = await self._restart_launch_processes(select_nodes)
            if not process_restart_result.get("success", True):
                return {
                    **stop_result,
                    "success": False,
                    "process_restart": process_restart_result,
                    "error": process_restart_result.get("error", "Failed to restart selected launch processes."),
                }
        return self.start(
            activate=True,
            select_nodes=select_nodes,
            include_dependencies=include_dependencies,
        ) | ({"process_restart": process_restart_result} if process_restart_result else {})

    async def _restart_launch_processes(self, entity_ids: list[str], timeout_sec: float = 20.0) -> dict:
        """Force launch-owned process replacement for selected cold restarts.

        Lifecycle cleanup does not reload a rebuilt executable. The launch service
        owns the OS process and respawns entities that have respawn enabled, so a
        selected cold restart must also terminate the launch process and wait for a
        new PID before lifecycle activation.
        """
        restarted: dict[str, dict] = {}
        deadline = time.monotonic() + timeout_sec

        for entity_id in entity_ids:
            with self._lock:
                state = self._entity_states.get(entity_id)
                old_pid = state.pid if state is not None else None
                was_alive = bool(state and state.alive and old_pid)

            if not was_alive or old_pid is None:
                restarted[entity_id] = {
                    "success": True,
                    "skipped": True,
                    "reason": "launch process was not alive",
                }
                continue

            try:
                os.kill(old_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

            kill_deadline = min(deadline, time.monotonic() + 5.0)
            while time.monotonic() < kill_deadline:
                with self._lock:
                    current = self._entity_states.get(entity_id)
                    current_pid = current.pid if current is not None else None
                    current_alive = bool(current and current.alive)
                if not current_alive or current_pid != old_pid:
                    break
                # Launch process-exit/start handlers run on this event loop and
                # update ``_entity_states``. Yield while polling so the respawn
                # event can actually be observed.
                await asyncio.sleep(0.1)
            else:
                try:
                    os.kill(old_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

            while time.monotonic() < deadline:
                with self._lock:
                    current = self._entity_states.get(entity_id)
                    current_pid = current.pid if current is not None else None
                    current_alive = bool(current and current.alive)
                if current_alive and current_pid is not None and current_pid != old_pid:
                    restarted[entity_id] = {
                        "success": True,
                        "old_pid": old_pid,
                        "new_pid": current_pid,
                    }
                    break
                await asyncio.sleep(0.1)

            if entity_id not in restarted:
                restarted[entity_id] = {
                    "success": False,
                    "old_pid": old_pid,
                    "new_pid": None,
                    "error": f"Timed out waiting for launch process respawn for {entity_id}",
                }

        success = all(item.get("success", False) for item in restarted.values())
        result = {"success": success, "entities": restarted}
        if not success:
            result["error"] = "Timed out waiting for selected launch process respawn."
        return result

    @staticmethod
    def _pid_is_alive(pid: int) -> bool:
        if pid <= 1:
            return False
        try:
            with open(f"/proc/{pid}/stat", encoding="utf-8") as stat_file:
                stat_fields = stat_file.read().split()
            if len(stat_fields) >= 3 and stat_fields[2] == "Z":
                return False
            os.kill(pid, 0)
        except (FileNotFoundError, ProcessLookupError):
            return False
        except PermissionError:
            return True
        return True

    async def _reap_launch_entity_processes(
        self,
        entity_pids: dict[str, int],
        *,
        terminate_timeout_sec: float = 2.0,
        kill_timeout_sec: float = 1.0,
    ) -> dict:
        """Ensure launch-owned entity processes are gone before a new boot."""

        pending = {
            entity_id: pid
            for entity_id, pid in entity_pids.items()
            if self._pid_is_alive(pid)
        }
        forced_termination = sorted(pending)
        for pid in pending.values():
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

        deadline = time.monotonic() + terminate_timeout_sec
        while pending and time.monotonic() < deadline:
            await asyncio.sleep(0.05)
            pending = {
                entity_id: pid
                for entity_id, pid in pending.items()
                if self._pid_is_alive(pid)
            }

        for pid in pending.values():
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

        kill_deadline = time.monotonic() + kill_timeout_sec
        while pending and time.monotonic() < kill_deadline:
            await asyncio.sleep(0.05)
            pending = {
                entity_id: pid
                for entity_id, pid in pending.items()
                if self._pid_is_alive(pid)
            }

        return {
            "success": not pending,
            "forced_termination": forced_termination,
            "survivors": pending,
        }

    async def shutdown_runtime(self, *, select_nodes: list[str], include_dependencies: bool) -> dict:
        self._ensure_ros_runtime()
        if not self._booted:
            return self._return_with_health({"success": True, "message": "System runtime is not booted."})

        assert self._supervisor is not None
        with self._lock:
            launch_entity_pids = {
                entity_id: state.pid
                for entity_id, state in self._entity_states.items()
                if state.generation == self._launch_generation and state.alive and state.pid is not None
            }
        self._set_desired_active(self.managed_node_ids(), False)
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
        process_cleanup = await self._reap_launch_entity_processes(launch_entity_pids)
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
        result = {
            "success": bool(process_cleanup["success"]),
            "process_cleanup": process_cleanup,
        }
        if not process_cleanup["success"]:
            result["error"] = "Launch entity processes remained alive after forced shutdown."
        return self._return_with_health(result)

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

    def _format_start_failure(
        self,
        *,
        activate: bool,
        ignored_nodes: set[str] | None = None,
        selected_nodes: set[str] | None = None,
    ) -> str:
        assert self._supervisor is not None
        ignored_nodes = ignored_nodes or set()
        states = self._supervisor._get_node_states()
        if activate:
            failed = {
                key: self._state_label(state)
                for key, state in states.items()
                if key not in ignored_nodes
                and (selected_nodes is None or key in selected_nodes)
                and state.id != State.PRIMARY_STATE_ACTIVE
            }
            target = "ACTIVE"
        else:
            failed = {
                key: self._state_label(state)
                for key, state in states.items()
                if key not in ignored_nodes
                and (selected_nodes is None or key in selected_nodes)
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
        self._ensure_ros_runtime()
        managed_nodes: dict[str, str] = {}
        if self._booted and self._supervisor is not None:
            # Lifecycle operations verify and cache every resulting state.
            # Status is an operator polling path and must not synchronously
            # round-trip to every ROS lifecycle service. Process liveness and
            # recovery are reported separately below.
            states = self._supervisor.cached_node_states()
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
                    "desired_active": state.desired_active,
                    "recovery_in_progress": state.recovery_in_progress,
                }
                for key, state in self._entity_states.items()
            },
        }

    def runtime_snapshot(self) -> dict:
        """Return non-blocking readiness state for command gating and health."""
        with self._lock:
            booted = self._booted
            profile = self._profile_name
            process_states = {
                key: {
                    "alive": state.alive,
                    "start_count": state.start_count,
                    "exit_count": state.exit_count,
                    "desired_active": state.desired_active,
                    "recovery_in_progress": state.recovery_in_progress,
                }
                for key, state in self._entity_states.items()
            }

        services = self._service_statuses()
        managed_nodes = {
            key: (
                "active"
                if state["alive"]
                and state["desired_active"]
                and not state["recovery_in_progress"]
                else "inactive"
            )
            for key, state in process_states.items()
        }
        nodes_active = bool(managed_nodes) and all(
            label == "active" for label in managed_nodes.values()
        )
        services_ready = all(service.get("ready") for service in services.values())
        return {
            "booted": booted,
            "profile": profile,
            "active": bool(booted and nodes_active and services_ready),
            "managed_nodes": managed_nodes,
            "services": services,
            "processes": process_states,
        }

    def health_status_message(self):
        if SystemHealthStatus is None or SubsystemHealthStatus is None:
            raise RuntimeError("iii_drone_interfaces health messages are unavailable")

        status = self.runtime_snapshot()
        services = status.get("services", {})
        processes = status.get("processes", {})
        managed_nodes = status.get("managed_nodes", {})
        subsystems = []
        degraded_reasons = []

        for service_id, service_status in sorted(services.items()):
            subsystem = SubsystemHealthStatus()
            subsystem.subsystem_id = service_id
            subsystem.label = service_id
            subsystem.ready = bool(service_status.get("ready"))
            subsystem.degraded = not subsystem.ready
            subsystem.status = (
                SubsystemHealthStatus.STATUS_OK
                if subsystem.ready
                else SubsystemHealthStatus.STATUS_DEGRADED
            )
            subsystem.reason = service_status.get("reason") or ""
            if subsystem.reason and subsystem.degraded:
                subsystem.degraded_reasons = [subsystem.reason]
                degraded_reasons.append(f"{service_id}: {subsystem.reason}")
            subsystem.owner = "supervision"
            subsystems.append(subsystem)

        for entity_id, process_status in sorted(processes.items()):
            subsystem = SubsystemHealthStatus()
            subsystem.subsystem_id = entity_id
            subsystem.label = entity_id
            subsystem.ready = bool(process_status.get("alive"))
            subsystem.degraded = not subsystem.ready
            subsystem.status = (
                SubsystemHealthStatus.STATUS_OK
                if subsystem.ready
                else SubsystemHealthStatus.STATUS_UNAVAILABLE
            )
            subsystem.reason = "" if subsystem.ready else "process is not alive"
            if subsystem.reason:
                subsystem.degraded_reasons = [subsystem.reason]
            subsystem.owner = "supervision"
            subsystems.append(subsystem)

        msg = SystemHealthStatus()
        if self._node is not None:
            msg.stamp = self._node.get_clock().now().to_msg()
        msg.profile = status.get("profile") or os.environ.get("III_SYSTEM_PROFILE", "unknown")
        msg.daemon_ready = True
        msg.runtime_booted = bool(status.get("booted"))
        msg.system_active = bool(managed_nodes) and all(label == "active" for label in managed_nodes.values())
        msg.managed_node_count = len(managed_nodes)
        msg.active_managed_node_count = sum(1 for label in managed_nodes.values() if label == "active")
        msg.service_count = len(services)
        msg.ready_service_count = sum(1 for service in services.values() if service.get("ready"))
        msg.subsystems = subsystems

        if not msg.runtime_booted:
            degraded_reasons.append("system is not booted")
        if services and msg.ready_service_count != msg.service_count:
            degraded_reasons.append("one or more daemon-managed services are not ready")
        if managed_nodes and msg.active_managed_node_count != msg.managed_node_count:
            degraded_reasons.append("one or more managed nodes are not active")

        msg.ready = msg.runtime_booted and not degraded_reasons
        msg.degraded = bool(degraded_reasons)
        msg.degraded_reasons = degraded_reasons
        if not msg.runtime_booted:
            msg.system_state = SystemHealthStatus.SYSTEM_STATE_STOPPED
        elif msg.ready:
            msg.system_state = SystemHealthStatus.SYSTEM_STATE_READY
        elif msg.degraded:
            msg.system_state = SystemHealthStatus.SYSTEM_STATE_DEGRADED
        else:
            msg.system_state = SystemHealthStatus.SYSTEM_STATE_RUNNING
        return msg

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
        return self._return_with_health({
            **result,
            "service": service_id,
            "alive": snapshot.alive,
            "ready": snapshot.ready,
            "reason": snapshot.ready_reason,
        })

    def service_stop(self, service_id: str) -> dict:
        self._require_booted()
        if service_id not in self._service_runtimes:
            raise KeyError(f"Unknown service: {service_id}")
        result = self._service_runtimes[service_id].stop()
        snapshot = self._service_runtimes[service_id].snapshot()
        return self._return_with_health({
            **result,
            "service": service_id,
            "alive": snapshot.alive,
            "ready": snapshot.ready,
            "reason": snapshot.ready_reason,
        })

    def service_restart(self, service_id: str) -> dict:
        self._require_booted()
        if service_id not in self._service_runtimes:
            raise KeyError(f"Unknown service: {service_id}")
        assert self._supervisor is not None
        profile = get_system_profile(self._profile_name or os.environ.get("III_SYSTEM_PROFILE", "sim"))
        dependencies = profile.service_dependencies()
        states = self._supervisor._get_node_states()
        active_dependents = sorted(
            node_id
            for node_id, requirements in dependencies.items()
            if service_id in requirements
            and node_id in states
            and states[node_id].id == State.PRIMARY_STATE_ACTIVE
        )

        dependent_stop: dict = {"success": True, "managed_nodes": []}
        if active_dependents:
            success, managed = self._supervisor.stop(
                cleanup=False,
                select_nodes=active_dependents,
                ignore_dependencies=True,
            )
            dependent_stop = {"success": success, "managed_nodes": managed}
            if not success:
                return self._return_with_health({
                    "success": False,
                    "service": service_id,
                    "dependent_nodes": active_dependents,
                    "dependent_stop": dependent_stop,
                    "error": "Failed to deactivate service-dependent nodes before restart.",
                })

        service = self._service_runtimes[service_id]
        result = service.restart()
        service.wait_ready(service.spec.ready_timeout_sec)
        snapshot = service.snapshot()
        dependent_start: dict = {"success": True, "managed_nodes": []}
        if result.get("success") and snapshot.ready and active_dependents:
            ready, missing_nodes = self._wait_for_lifecycle_nodes(active_dependents, timeout_sec=60.0)
            if ready:
                success, managed = self._supervisor.start(
                    activate=True,
                    select_nodes=active_dependents,
                    ignore_dependencies=False,
                )
                dependent_start = {"success": success, "managed_nodes": managed}
            else:
                dependent_start = {
                    "success": False,
                    "managed_nodes": [],
                    "missing_nodes": missing_nodes,
                }

        success = bool(result.get("success") and snapshot.ready and dependent_start["success"])
        return self._return_with_health({
            **result,
            "success": success,
            "service": service_id,
            "alive": snapshot.alive,
            "ready": snapshot.ready,
            "reason": snapshot.ready_reason,
            "dependent_nodes": active_dependents,
            "dependent_stop": dependent_stop,
            "dependent_start": dependent_start,
            **({"error": "Service or dependent-node recovery did not reach ready/active state."} if not success else {}),
        })

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
            self._executor = None
        if self._executor_thread is not None:
            self._executor_thread.join(timeout=1.0)
            self._executor_thread = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()
