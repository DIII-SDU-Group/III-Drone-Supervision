"""Daemon-owned service process management for non-lifecycle system services."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import os
from pathlib import Path
import signal
import subprocess
from threading import Lock, Thread
import time

import rclpy
from rclpy import qos
from rclpy.node import Node

from .system_spec import SystemServiceSpec, TopicReadinessSpec


@dataclass
class ServiceRuntimeSnapshot:
    """Serializable runtime state for a daemon-owned service."""

    service_id: str
    alive: bool
    ready: bool
    start_count: int
    exit_count: int
    generation: int
    pid: int | None
    command: str | None
    current_log_path: str | None
    last_returncode: int | None
    ready_reason: str


class TopicReadinessMonitor:
    """Tracks recent ROS topic messages used as service readiness heartbeats."""

    def __init__(self, node: Node, service_id: str, topic_specs: tuple[TopicReadinessSpec, ...]):
        self._node = node
        self._service_id = service_id
        self._topic_specs = topic_specs
        self._topic_spec_map = {topic_spec.topic: topic_spec for topic_spec in topic_specs}
        self._last_seen: dict[str, float] = {}
        self._ready_since: dict[str, float] = {}
        self._lock = Lock()
        self._subscriptions = []

        for topic_spec in self._topic_specs:
            self._subscriptions.append(self._create_subscription(topic_spec))

    def _create_subscription(self, topic_spec: TopicReadinessSpec):
        message_type = topic_spec.message_type.split("/")
        message_type_module = ".".join(message_type[:-1])
        message_type_class = message_type[-1]
        module = importlib.import_module(message_type_module)
        message_class = getattr(module, message_type_class)

        return self._node.create_subscription(
            message_class,
            topic_spec.topic,
            lambda message, topic=topic_spec.topic: self._mark_seen(topic),
            qos.QoSProfile(
                reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
                durability=qos.QoSDurabilityPolicy.VOLATILE,
                history=qos.QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

    def _mark_seen(self, topic: str) -> None:
        now = time.monotonic()
        with self._lock:
            topic_spec = self._topic_spec_map[topic]
            last_seen = self._last_seen.get(topic)
            if last_seen is None or now - last_seen > topic_spec.timeout_sec:
                self._ready_since.pop(topic, None)
            self._last_seen[topic] = now

    def readiness(self) -> tuple[bool, str]:
        if not self._topic_specs:
            return True, "no readiness checks configured"

        now = time.monotonic()
        missing = []
        stale = []
        stabilizing = []
        with self._lock:
            for topic_spec in self._topic_specs:
                last_seen = self._last_seen.get(topic_spec.topic)
                if last_seen is None:
                    self._ready_since.pop(topic_spec.topic, None)
                    missing.append(topic_spec.topic)
                elif now - last_seen > topic_spec.timeout_sec:
                    self._ready_since.pop(topic_spec.topic, None)
                    stale.append(f"{topic_spec.topic} stale for {now - last_seen:.1f}s")
                else:
                    ready_since = self._ready_since.setdefault(topic_spec.topic, now)
                    stable_for_sec = max(0.0, topic_spec.stable_for_sec)
                    ready_duration = now - ready_since
                    if ready_duration < stable_for_sec:
                        stabilizing.append(
                            f"{topic_spec.topic} fresh for {ready_duration:.1f}/{stable_for_sec:.1f}s"
                        )

        if missing:
            return False, "waiting for topic(s): " + ", ".join(sorted(missing))
        if stale:
            return False, "; ".join(stale)
        if stabilizing:
            return False, "waiting for stable topic(s): " + ", ".join(sorted(stabilizing))
        return True, "ready"

    def destroy(self) -> None:
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._subscriptions.clear()


class ServiceProcess:
    """Owns one daemon-managed process plus readiness monitoring and logs."""

    def __init__(self, spec: SystemServiceSpec, profile_name: str, node: Node, log_dir: Path):
        self.spec = spec
        self.profile_name = profile_name
        self._node = node
        self._log_dir = log_dir
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._monitor = TopicReadinessMonitor(node, spec.service_id, spec.readiness_topics)

        self._lock = Lock()
        self._process: subprocess.Popen | None = None
        self._generation = 0
        self._start_count = 0
        self._exit_count = 0
        self._last_returncode: int | None = None
        self._command: str | None = None
        self._current_log_path: str | None = str(self._log_dir / "current.log")
        self._stop_requested = False
        self._watcher_thread: Thread | None = None
        self._output_thread: Thread | None = None
        self._write_prepared_log()

    @staticmethod
    def _write_log_file(path: Path, text: str | bytes, *, append: bool = True) -> None:
        if isinstance(text, str):
            payload = text.encode("utf-8", errors="replace")
        else:
            payload = text
        mode = "ab" if append else "wb"
        with open(path, mode) as file:
            file.write(payload)
            if payload and not payload.endswith(b"\n"):
                file.write(b"\n")

    def _append_process_log(self, text: str | bytes) -> None:
        self._write_log_file(self._log_dir / "process.log", text)

    def _write_current_log(self, text: str | bytes, *, append: bool = True) -> None:
        self._write_log_file(self._log_dir / "current.log", text, append=append)

    @staticmethod
    def _run_separator(
        kind: str,
        *,
        service_id: str,
        generation: int,
        pid: int | None = None,
        command: str | None = None,
        returncode: int | None = None,
    ) -> str:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S %z", time.localtime())
        line = "=" * 88
        fields = f"service={service_id} generation={generation}"
        if pid is not None:
            fields += f" pid={pid}"
        if returncode is not None:
            fields += f" returncode={returncode}"
        if command is not None:
            fields += f" command={command!r}"
        return f"\n{line}\n[system_manager] SERVICE {kind}: {fields} time={timestamp}\n{line}\n"

    def _write_prepared_log(self) -> None:
        try:
            command = self.spec.command(self.profile_name)
        except Exception as exc:
            message = (
                f"[system_manager] Service {self.spec.service_id} is loaded, but its command "
                f"could not be resolved: {exc}\n"
            )
            header = self._run_separator(
                "PREPARED",
                service_id=self.spec.service_id,
                generation=self._generation,
            )
        else:
            message = (
                f"[system_manager] Service {self.spec.service_id} is loaded by the daemon and not running yet.\n"
                "[system_manager] It starts during `iii system start` when required by the active profile, "
                f"or via `iii system service start {self.spec.service_id}`.\n"
            )
            header = self._run_separator(
                "PREPARED",
                service_id=self.spec.service_id,
                generation=self._generation,
                command=command,
            )

        text = header + message
        self._append_process_log(text)
        self._write_current_log(text, append=False)

    def start(self) -> dict:
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                return {"success": True, "already_running": True, "pid": self._process.pid}

            self._generation += 1
            generation = self._generation
            self._stop_requested = False
            self._command = self.spec.command(self.profile_name)
            working_directory = os.path.expanduser(self.spec.resolved_working_directory())

            try:
                process = subprocess.Popen(
                    self._command,
                    cwd=working_directory,
                    shell=True,
                    start_new_session=True,
                    executable="/bin/bash",
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    env=os.environ.copy(),
                )
            except Exception as exc:
                self._last_returncode = None
                message = f"[system_manager] failed to start service {self.spec.service_id}: {exc}\n"
                self._append_process_log(message)
                self._write_current_log(message, append=False)
                return {"success": False, "error": str(exc)}

            self._process = process
            self._start_count += 1
            self._current_log_path = str(self._log_dir / "current.log")
            header = self._run_separator(
                "START",
                service_id=self.spec.service_id,
                generation=generation,
                pid=process.pid,
                command=self._command,
            )
            self._append_process_log(header)
            self._write_current_log(header, append=False)

            self._output_thread = Thread(
                target=self._pump_output,
                args=(process, generation),
                daemon=True,
            )
            self._watcher_thread = Thread(
                target=self._watch_process,
                args=(process, generation),
                daemon=True,
            )
            self._output_thread.start()
            self._watcher_thread.start()

            return {"success": True, "already_running": False, "pid": process.pid}

    def _pump_output(self, process: subprocess.Popen, generation: int) -> None:
        if process.stdout is None:
            return
        for line in iter(process.stdout.readline, b""):
            self._append_process_log(line)
            with self._lock:
                write_current = self._generation == generation and self._current_log_path is not None
            if write_current:
                self._write_current_log(line)

    def _watch_process(self, process: subprocess.Popen, generation: int) -> None:
        returncode = process.wait()
        restart = False
        with self._lock:
            current = self._process is process and self._generation == generation
            if current:
                self._exit_count += 1
                self._last_returncode = returncode
                self._process = None
                restart = self.spec.restart_on_exit and not self._stop_requested

        footer = self._run_separator(
            "END",
            service_id=self.spec.service_id,
            generation=generation,
            pid=process.pid,
            returncode=returncode,
        )
        self._append_process_log(footer)
        if current:
            self._write_current_log(footer)

        if restart and rclpy.ok():
            time.sleep(self.spec.restart_delay_sec)
            with self._lock:
                if self._stop_requested:
                    return
            self.start()

    def stop(self) -> dict:
        with self._lock:
            self._stop_requested = True
            process = self._process

        if process is None:
            return {"success": True, "already_stopped": True}

        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

            try:
                process.wait(timeout=self.spec.stop_timeout_sec)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()

        watcher_thread = self._watcher_thread
        if watcher_thread is not None and watcher_thread.is_alive():
            watcher_thread.join(timeout=1.0)

        with self._lock:
            if self._process is process:
                self._process = None

        return {"success": True, "already_stopped": False}

    def restart(self) -> dict:
        stop_result = self.stop()
        if not stop_result["success"]:
            return stop_result
        return self.start()

    def wait_ready(self, timeout_sec: float) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        last_reason = "not checked"
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            last_reason = snapshot.ready_reason
            if snapshot.ready:
                return True, last_reason
            if not snapshot.alive:
                return False, last_reason
            time.sleep(0.2)
        snapshot = self.snapshot()
        return snapshot.ready, snapshot.ready_reason or last_reason

    def snapshot(self) -> ServiceRuntimeSnapshot:
        with self._lock:
            process = self._process
            alive = process is not None and process.poll() is None
            pid = process.pid if alive else None
            start_count = self._start_count
            exit_count = self._exit_count
            generation = self._generation
            command = self._command
            current_log_path = self._current_log_path
            last_returncode = self._last_returncode

        if alive:
            ready, reason = self._monitor.readiness()
        else:
            ready = False
            reason = "service process is not running"

        return ServiceRuntimeSnapshot(
            service_id=self.spec.service_id,
            alive=alive,
            ready=ready,
            start_count=start_count,
            exit_count=exit_count,
            generation=generation,
            pid=pid,
            command=command,
            current_log_path=current_log_path,
            last_returncode=last_returncode,
            ready_reason=reason,
        )

    def destroy(self) -> None:
        self.stop()
        self._monitor.destroy()
