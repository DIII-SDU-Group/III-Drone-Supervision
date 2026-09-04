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
from rclpy.wait_for_message import wait_for_message

from .system_spec import Px4MessageFormatReadinessSpec, SystemServiceSpec, TopicReadinessSpec
from .log_retention import DEFAULT_ENTITY_LOG_MAX_BYTES, configured_max_bytes, write_bounded_log


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
        self._last_message_timestamp: dict[str, int] = {}
        self._last_message_changed: dict[str, float] = {}
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
            lambda message, topic=topic_spec.topic: self._mark_seen(topic, message),
            qos.QoSProfile(
                reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
                durability=qos.QoSDurabilityPolicy.VOLATILE,
                history=qos.QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

    @staticmethod
    def _message_timestamp(message) -> int | None:
        timestamp = getattr(message, "timestamp", None)
        if timestamp is None:
            return None
        try:
            return int(timestamp)
        except (TypeError, ValueError):
            return None

    def _mark_seen(self, topic: str, message) -> None:
        now = time.monotonic()
        with self._lock:
            topic_spec = self._topic_spec_map[topic]
            last_seen = self._last_seen.get(topic)
            if last_seen is None or now - last_seen > topic_spec.timeout_sec:
                self._ready_since.pop(topic, None)
            message_timestamp = self._message_timestamp(message)
            if message_timestamp is not None:
                previous_timestamp = self._last_message_timestamp.get(topic)
                if previous_timestamp != message_timestamp:
                    self._last_message_timestamp[topic] = message_timestamp
                    self._last_message_changed[topic] = now
            self._last_seen[topic] = now

    def reset(self) -> None:
        with self._lock:
            self._last_seen.clear()
            self._last_message_timestamp.clear()
            self._last_message_changed.clear()
            self._ready_since.clear()

    def _probe_topic_once(self, topic_spec: TopicReadinessSpec) -> None:
        """Synchronously verify topic flow if the daemon callback path missed it."""
        if not rclpy.ok():
            return
        message_type = topic_spec.message_type.split("/")
        message_type_module = ".".join(message_type[:-1])
        message_type_class = message_type[-1]
        module = importlib.import_module(message_type_module)
        message_class = getattr(module, message_type_class)
        probe_node = rclpy.create_node(f"{self._service_id}_readiness_probe")
        try:
            received, _ = wait_for_message(
                message_class,
                probe_node,
                topic_spec.topic,
                qos_profile=qos.QoSProfile(
                    reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
                    durability=qos.QoSDurabilityPolicy.VOLATILE,
                    history=qos.QoSHistoryPolicy.KEEP_LAST,
                    depth=1,
                ),
                time_to_wait=min(0.5, max(0.05, topic_spec.timeout_sec)),
            )
            if received:
                self._mark_seen(topic_spec.topic, _)
        finally:
            probe_node.destroy_node()

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
                    missing.append(topic_spec)
                elif now - last_seen > topic_spec.timeout_sec:
                    stale.append((topic_spec, now - last_seen))
                else:
                    ready_since = self._ready_since.setdefault(topic_spec.topic, now)
                    stable_for_sec = max(0.0, topic_spec.stable_for_sec)
                    ready_duration = now - ready_since
                    last_timestamp = self._last_message_timestamp.get(topic_spec.topic)
                    last_changed = self._last_message_changed.get(topic_spec.topic)
                    max_timestamp_age = max(0.5, min(1.0, topic_spec.timeout_sec * 0.5))
                    if last_timestamp is not None and (
                        last_changed is None or now - last_changed > max_timestamp_age
                    ):
                        stale.append((topic_spec, now - (last_changed or last_seen)))
                        continue
                    if ready_duration < stable_for_sec:
                        stabilizing.append(
                            f"{topic_spec.topic} fresh for {ready_duration:.1f}/{stable_for_sec:.1f}s"
                        )

        if missing:
            # The monitor already owns subscriptions on the daemon's spinning
            # node.  Readiness snapshots are used by status and command gates,
            # so they must never create another ROS node or synchronously wait
            # for traffic.  Repeated temporary nodes caused unbounded DDS
            # threads and made otherwise read-only status calls stall.
            return False, "waiting for topic(s): " + ", ".join(
                sorted(topic_spec.topic for topic_spec in missing)
            )
        if stale:
            return False, "; ".join(
                f"{topic_spec.topic} stale for {age:.1f}s"
                for topic_spec, age in stale
            )
        if stabilizing:
            return False, "waiting for stable topic(s): " + ", ".join(sorted(stabilizing))
        return True, "ready"

    def destroy(self) -> None:
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._subscriptions.clear()


class Px4MessageFormatReadinessMonitor:
    """Actively probes PX4's message-format request/response path."""

    def __init__(self, node: Node, specs: tuple[Px4MessageFormatReadinessSpec, ...]):
        self._node = node
        self._specs = specs
        self._last_success: dict[str, float] = {}
        self._last_probe: dict[str, float] = {}
        self._lock = Lock()
        self._subscription = None
        self._publisher = None

        if not specs:
            return

        from px4_msgs.msg import MessageFormatRequest, MessageFormatResponse

        self._request_type = MessageFormatRequest
        qos_profile = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=qos.QoSDurabilityPolicy.VOLATILE,
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._subscription = self._node.create_subscription(
            MessageFormatResponse,
            "/fmu/out/message_format_response",
            self._mark_response,
            qos_profile,
        )
        self._publisher = self._node.create_publisher(
            MessageFormatRequest,
            "/fmu/in/message_format_request",
            qos_profile,
        )

    @staticmethod
    def _decode_topic_name(topic_name) -> str:
        return bytes(topic_name).split(b"\0", 1)[0].decode("ascii", errors="replace")

    @staticmethod
    def _encode_topic_name(topic_name: str) -> list[int]:
        encoded = topic_name.encode("ascii")
        return list(encoded[:50]) + [0] * max(0, 50 - len(encoded))

    def _mark_response(self, message) -> None:
        if not message.success:
            return
        topic_name = self._decode_topic_name(message.topic_name)
        now = time.monotonic()
        with self._lock:
            self._last_success[topic_name] = now

    def reset(self) -> None:
        with self._lock:
            self._last_success.clear()
            self._last_probe.clear()

    def _publish_probe(self, topic_name: str, now: float) -> None:
        if self._publisher is None:
            return

        with self._lock:
            last_probe = self._last_probe.get(topic_name, 0.0)
            if now - last_probe < 0.5:
                return
            self._last_probe[topic_name] = now

        request = self._request_type()
        request.timestamp = int(time.time() * 1e6)
        request.protocol_version = self._request_type.LATEST_PROTOCOL_VERSION
        request.topic_name = self._encode_topic_name(topic_name)
        self._publisher.publish(request)

    def _probe_topic_once(self, topic_name: str) -> None:
        """Synchronously verify PX4 message-format round trip if callbacks missed it."""
        if not self._specs or not rclpy.ok():
            return

        from px4_msgs.msg import MessageFormatRequest, MessageFormatResponse

        qos_profile = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=qos.QoSDurabilityPolicy.VOLATILE,
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        probe_node = rclpy.create_node("px4_message_format_readiness_probe")
        probe_executor = rclpy.executors.SingleThreadedExecutor(context=probe_node.context)
        probe_executor.add_node(probe_node)
        responses = []

        def callback(message):
            if not message.success:
                return
            if self._decode_topic_name(message.topic_name) == topic_name:
                responses.append(message)

        subscription = probe_node.create_subscription(
            MessageFormatResponse,
            "/fmu/out/message_format_response",
            callback,
            qos_profile,
        )
        publisher = probe_node.create_publisher(
            MessageFormatRequest,
            "/fmu/in/message_format_request",
            qos_profile,
        )
        try:
            request = MessageFormatRequest()
            request.timestamp = 0
            request.protocol_version = MessageFormatRequest.LATEST_PROTOCOL_VERSION
            request.topic_name = self._encode_topic_name(topic_name)

            deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < deadline and not responses:
                publisher.publish(request)
                probe_executor.spin_once(timeout_sec=0.1)

            if responses:
                with self._lock:
                    self._last_success[topic_name] = time.monotonic()
        finally:
            probe_executor.remove_node(probe_node)
            probe_executor.shutdown()
            probe_node.destroy_subscription(subscription)
            probe_node.destroy_publisher(publisher)
            probe_node.destroy_node()

    def readiness(self) -> tuple[bool, str]:
        if not self._specs:
            return True, "no PX4 message-format checks configured"

        now = time.monotonic()
        waiting = []
        with self._lock:
            for spec in self._specs:
                last_success = self._last_success.get(spec.topic_name)
                if last_success is None:
                    waiting.append(spec.topic_name)

        for topic_name in waiting:
            self._publish_probe(topic_name, now)

        if waiting:
            # Responses arrive through the persistent subscription on the
            # daemon's executor.  Keep this method non-blocking: callers poll
            # it while waiting and status paths call it directly.
            return False, "waiting for PX4 message-format response for: " + ", ".join(
                sorted(waiting)
            )
        return True, "ready"

    def destroy(self) -> None:
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        if self._publisher is not None:
            self._node.destroy_publisher(self._publisher)
            self._publisher = None


class ServiceReadinessMonitor:
    """Combines passive topic flow and active protocol probes for a service."""

    def __init__(self, node: Node, spec: SystemServiceSpec):
        self._has_checks = bool(spec.readiness_topics or spec.px4_message_format_readiness)
        self._topic_monitor = TopicReadinessMonitor(node, spec.service_id, spec.readiness_topics)
        self._px4_message_format_monitor = Px4MessageFormatReadinessMonitor(
            node,
            spec.px4_message_format_readiness,
        )

    def readiness(self) -> tuple[bool, str]:
        if not self._has_checks:
            return True, "no readiness checks configured"

        topic_ready, topic_reason = self._topic_monitor.readiness()
        px4_ready, px4_reason = self._px4_message_format_monitor.readiness()

        if topic_ready and px4_ready:
            return True, "ready"

        reasons = []
        if not topic_ready:
            reasons.append(topic_reason)
        if not px4_ready:
            reasons.append(px4_reason)
        return False, "; ".join(reasons)

    def reset(self) -> None:
        self._topic_monitor.reset()
        self._px4_message_format_monitor.reset()

    def destroy(self) -> None:
        self._topic_monitor.destroy()
        self._px4_message_format_monitor.destroy()


class ServiceProcess:
    """Owns one daemon-managed process plus readiness monitoring and logs."""

    def __init__(self, spec: SystemServiceSpec, profile_name: str, node: Node, log_dir: Path):
        self.spec = spec
        self.profile_name = profile_name
        self._node = node
        self._log_dir = log_dir
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._monitor = ServiceReadinessMonitor(node, spec)

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
        write_bounded_log(
            path,
            text,
            append=append,
            max_bytes=configured_max_bytes(
                "III_SYSTEM_ENTITY_LOG_MAX_BYTES",
                DEFAULT_ENTITY_LOG_MAX_BYTES,
            ),
        )

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
            self._monitor.reset()
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
