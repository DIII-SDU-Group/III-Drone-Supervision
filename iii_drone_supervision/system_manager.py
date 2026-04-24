"""Daemon-owned III system manager built on ROS 2 launch and lifecycle APIs."""

from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Lock, Thread
import time
import os

from launch import LaunchDescription, LaunchService
from launch.actions import GroupAction, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import RegisterEventHandler
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .supervisor import Supervisor
from .system_spec import entity_log_dir, get_system_profile
from .tmux_spec import get_tmux_session_spec


@dataclass
class EntityRuntimeState:
    entity_id: str
    alive: bool = False
    start_count: int = 0
    exit_count: int = 0


class SystemManager:
    """Owns launch runtime, lifecycle orchestration, and CLI-facing status."""

    def __init__(self):
        self._lock = Lock()
        self._booted = False
        self._profile_name: str | None = None

        self._launch_service: LaunchService | None = None
        self._launch_thread: Thread | None = None
        self._node: Node | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._executor_thread: Thread | None = None
        self._supervisor: Supervisor | None = None

        self._entity_states: dict[str, EntityRuntimeState] = {}
        self._log_dirs: dict[str, str] = {}

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

    def _build_launch_description(self, profile_name: str) -> LaunchDescription:
        profile = get_system_profile(profile_name)
        entities = []
        self._entity_states = {}
        self._log_dirs = {}

        for entity in profile.entities:
            log_dir = entity_log_dir(profile_name, entity.entity_id)
            log_dir.mkdir(parents=True, exist_ok=True)
            self._entity_states[entity.entity_id] = EntityRuntimeState(entity_id=entity.entity_id)
            self._log_dirs[entity.entity_id] = str(log_dir)

            target_action = entity.launch_factory(profile_name)
            action = GroupAction(
                [
                    SetEnvironmentVariable("ROS_LOG_DIR", str(log_dir)),
                    target_action,
                ]
            )
            entities.append(action)
            entities.append(
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=lambda action, expected=target_action: action == expected,
                        on_start=self._make_process_started_callback(entity.entity_id),
                    )
                )
            )
            entities.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=lambda action, expected=target_action: action == expected,
                        on_exit=self._make_process_exited_callback(entity.entity_id),
                    )
                )
            )

        return LaunchDescription(entities)

    def _make_process_started_callback(self, entity_id: str):
        def callback(event, context):
            del event, context
            with self._lock:
                state = self._entity_states[entity_id]
                state.alive = True
                state.start_count += 1
            return None

        return callback

    def _make_process_exited_callback(self, entity_id: str):
        def callback(event, context):
            del event, context
            with self._lock:
                state = self._entity_states[entity_id]
                state.alive = False
                state.exit_count += 1
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
            self._launch_service = LaunchService()
            launch_description = self._build_launch_description(profile.name)
            self._launch_service.include_launch_description(launch_description)
            self._launch_thread = Thread(target=self._launch_service.run, daemon=True)
            self._launch_thread.start()
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

    def start(self, *, activate: bool, select_nodes: list[str], include_dependencies: bool) -> dict:
        self._require_booted()
        assert self._supervisor is not None
        success, managed = self._supervisor.start(
            activate=activate,
            select_nodes=select_nodes,
            ignore_dependencies=not include_dependencies,
        )
        return {"success": success, "managed_nodes": managed}

    def stop(self, *, cleanup: bool, select_nodes: list[str], include_dependencies: bool) -> dict:
        self._require_booted()
        assert self._supervisor is not None
        success, managed = self._supervisor.stop(
            cleanup=cleanup,
            select_nodes=select_nodes,
            ignore_dependencies=not include_dependencies,
        )
        return {"success": success, "managed_nodes": managed}

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

    def shutdown_runtime(self, *, select_nodes: list[str], include_dependencies: bool) -> dict:
        self._require_booted()
        assert self._supervisor is not None
        self._supervisor.shutdown(
            select_nodes=select_nodes,
            ignore_dependencies=not include_dependencies,
        )
        if self._launch_service is not None:
            self._launch_service.shutdown()
        if self._launch_thread is not None and self._launch_thread.is_alive():
            self._launch_thread.join(timeout=10.0)
        if self._supervisor is not None:
            self._supervisor.destroy()
        with self._lock:
            self._booted = False
            self._launch_service = None
            self._launch_thread = None
            self._supervisor = None
        return {"success": True}

    def managed_node_ids(self) -> list[str]:
        profile_name = self._profile_name or os.environ.get("III_SYSTEM_PROFILE", "sim")
        return list(get_system_profile(profile_name).build_supervision_config()["managed_nodes"].keys())

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

    def close(self) -> None:
        if self._booted:
            self.shutdown_runtime(select_nodes=[], include_dependencies=True)
        if self._executor is not None and self._node is not None:
            self._executor.remove_node(self._node)
            self._executor.shutdown(timeout_sec=1.0)
        if self._node is not None:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
