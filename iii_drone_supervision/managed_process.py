#########################################################################
# Imports:
#########################################################################

from datetime import timedelta, datetime
from time import sleep
import subprocess
from subprocess import signal
import os
import threading
import importlib

import rclpy
from rclpy.node import Node
from rclpy import qos

from iii_drone_supervision.process_management_configuration import ProcessManagementConfiguration

#########################################################################
# Class:
#########################################################################

class ManagedProcess:
    def __init__(
        self,
        process_management_configuration: ProcessManagementConfiguration,
        parent_node: Node
    ) -> None:
        self.process_management_configuration = process_management_configuration

        self._is_started = False
        self._process = None
        
        self._parent_node = parent_node
        
        self._init_process_monitoring()
            
    def _init_process_monitoring(self) -> None:
        process_monitor_command_dict = self.process_management_configuration.process_monitor_command

        if process_monitor_command_dict is not None:
            command_type = process_monitor_command_dict["type"]

            if command_type == "command":
                return
            elif command_type == "topic":
                topic: str = process_monitor_command_dict["topic"]
                message_type: str = process_monitor_command_dict["message_type"]
                self._monitor_topic_check_field = process_monitor_command_dict.get("check_field", None)
                self._monitor_topic_check_value = process_monitor_command_dict.get("check_value", None)
                self._monitor_topic_timeout_sec = process_monitor_command_dict.get("timeout_sec")

                message_type = message_type.split("/")

                message_type_module = ".".join(message_type[:-1])
                message_type_class = message_type[-1]

                module = importlib.import_module(message_type_module)
                message_class = getattr(module, message_type_class)

                self._last_monitor_message_ok_time: rclpy.time.Time = None
                self._last_monitor_message_ok_time_lock = threading.Lock()

                self._monitor_cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

                self._monitor_topic_sub = self._parent_node.create_subscription(
                    message_class,
                    topic,
                    self._monitor_topic_callback,
                    qos.QoSProfile(
                        reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
                        durability=qos.QoSDurabilityPolicy.VOLATILE,
                        history=qos.QoSHistoryPolicy.KEEP_LAST,
                        depth=1
                    ),
                    callback_group=self._monitor_cb_group
                )

            else:
                raise RuntimeError(f"Unsupported process monitor command type: {command_type}")
    def _monitor_topic_callback(
        self,
        message
    ) -> None:
        valid = False
        
        if self._monitor_topic_check_field is not None:
            try:
                value = eval(f"message.{self._monitor_topic_check_field}")
            except Exception:
                return
            
            if self._monitor_topic_check_value is None or value == self._monitor_topic_check_value:
                valid = True
        else:
            valid = True
            
        if valid:
            with self._last_monitor_message_ok_time_lock:
                self._last_monitor_message_ok_time = self._parent_node.get_clock().now()

    def configure(
        self
    ) -> bool:
        return True
    
    def cleanup(
        self
    ) -> bool:
        return True

    def start(
        self
    ) -> bool:
        if not self._is_started:
            start_time = datetime.now()

            self._process = subprocess.Popen(
                self.process_management_configuration.command,
                cwd=self.process_management_configuration.working_directory,
                shell=True,
                start_new_session=True,
                executable='/bin/bash',
            )

            self._is_started = True
            
            if self.process_management_configuration.process_monitor_command is not None:
                success = False

                while self.process_management_configuration.process_start_timeout is None or (datetime.now() - start_time) < self.process_management_configuration.process_start_timeout:
                    if self._process.poll() is not None:
                        stop_success = self.stop()
                        if not stop_success:
                            raise RuntimeError("Failed to stop process after failed start.")
                        
                        return False
                    
                    monitor_start_time = datetime.now()
                    
                    if self.is_running():
                        success = True
                        break
                    
                    monitor_end_time = datetime.now()
                    monitor_duration = monitor_end_time - monitor_start_time
                    
                    process_monitor_period_remaining = self.process_management_configuration.process_monitor_period - monitor_duration

                    if process_monitor_period_remaining.total_seconds() > 0:
                        sleep(process_monitor_period_remaining.total_seconds())

                if not success:
                    stop_success = self.stop()
                    if not stop_success:
                        raise RuntimeError("Failed to stop process after failed start.")
                    
                    return False
        
        return True
    
    def stop(
        self
    ) -> bool:
        if self._is_started:
            os.killpg(self._process.pid, signal.SIGKILL)
            self._process.wait()
            self._process = None

            self._is_started = False
            
            if self.process_management_configuration.process_monitor_command is not None:
                success = False
                
                start_time = datetime.now()
                while self.process_management_configuration.process_stop_timeout is None or (datetime.now() - start_time) < self.process_management_configuration.process_stop_timeout:
                    if not self.is_running():
                        success = True
                        break

                    sleep(self.process_management_configuration.process_monitor_period.total_seconds())

                if not success:
                    return False
        
        self._is_started = False
        return True
    
    def is_running(
        self
    ) -> bool:
        if self._process is None:
            return False
        
        if self._process.poll() is not None:
            return False
        
        if self.process_management_configuration.process_monitor_command is not None:
            process_monitor_command_dict = self.process_management_configuration.process_monitor_command
            
            command_type = process_monitor_command_dict["type"]
            
            if command_type == "command":
                command = process_monitor_command_dict["command"]
                
                monitor_process = subprocess.Popen(
                    command,
                    cwd=self.process_management_configuration.working_directory,
                    shell=True,
                    start_new_session=True,
                    executable='/bin/bash'
                )
                monitor_process.wait()

                return monitor_process.returncode == 0
            elif command_type == "topic":
                with self._last_monitor_message_ok_time_lock:
                    if self._last_monitor_message_ok_time is None:
                        return False

                    if self._monitor_topic_timeout_sec is not None and (self._parent_node.get_clock().now() - self._last_monitor_message_ok_time).nanoseconds / 1e9 > self._monitor_topic_timeout_sec:
                        return False

                    return True
            else:
                raise NotImplementedError(f"Unsupported process monitor command type: {command_type}")

        return self._is_started
    
    @property
    def is_started(self) -> bool:
        return self._is_started
    
    