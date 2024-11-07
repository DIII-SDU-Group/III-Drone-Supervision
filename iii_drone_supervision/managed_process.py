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
from functools import partial

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

        self._monitor_topic_configs: list[dict] = []
        self._last_monitor_message_ok_times: list[rclpy.time.Time] = []
        self._last_monitor_message_ok_time_locks: list[threading.Lock] = []
        # self._monitor_cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._monitor_topic_subscribers: list[rclpy.subscription.Subscription] = []
        self._topic_monitor_cnt = 0
        
        self._init_process_monitoring()
            
    def _init_process_monitoring(self) -> None:
        process_monitor_command: list = self.process_management_configuration.process_monitor_command

        if process_monitor_command is None:
            return
        
        if len(process_monitor_command) > 1:
            raise NotImplementedError("Process monitoring not implemented for multiple commands.")
        
        for process_monitor_command_dict in process_monitor_command:
            if process_monitor_command_dict is not None:
                command_type = process_monitor_command_dict["type"]

                if command_type == "command":
                    return
                elif command_type == "topic":
                    self._monitor_topic_configs.append(process_monitor_command_dict)
                    # check_field = process_monitor_command_dict.get("check_field", None)
                    # check_value = process_monitor_command_dict.get("check_value", None)
                    # timeout_sec = process_monitor_command_dict.get("timeout_sec")

                    self._last_monitor_message_ok_times.append(None)
                    self._last_monitor_message_ok_time_locks.append(threading.Lock())

                    self._monitor_topic_subscribers.append(
                        self._create_subscription(
                            self._parent_node,
                            process_monitor_command_dict,
                            self._topic_monitor_cnt
                        )
                    )
                    
                    self._topic_monitor_cnt += 1

                else:
                    raise RuntimeError(f"Unsupported process monitor command type: {command_type}")

    def _create_subscription(
        self,
        node: Node,
        process_monitor_command_dict: dict,
        idx
    ):
        topic: str = process_monitor_command_dict["topic"]
        message_type: str = process_monitor_command_dict["message_type"]

        message_type = message_type.split("/")

        message_type_module = ".".join(message_type[:-1])
        message_type_class = message_type[-1]

        module = importlib.import_module(message_type_module)
        message_class = getattr(module, message_type_class)

        
        return node.create_subscription(
            message_class,
            topic,
            partial(
                self._monitor_topic_callback, 
                topic_monitor_index=idx
            ),
            qos.QoSProfile(
                reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
                durability=qos.QoSDurabilityPolicy.VOLATILE,
                history=qos.QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        

    def _monitor_topic_callback(
        self,
        message,
        topic_monitor_index: int
    ) -> None:
        
        valid = False
        
        monitor_config = self._monitor_topic_configs[topic_monitor_index]
        
        check_field = monitor_config.get("check_field", None)
        
        if check_field is not None:
            try:
                value = eval(f"message.{check_field}")
            except Exception:
                return

            check_value = monitor_config.get("check_value", None)
            
            if check_value is None or value == check_value:
                valid = True
        else:
            valid = True
            
        if valid:
            with self._last_monitor_message_ok_time_locks[topic_monitor_index]:
                self._last_monitor_message_ok_times[topic_monitor_index] = self._parent_node.get_clock().now()


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

                temp_node = rclpy.create_node(self.process_management_configuration.node_name + "_temp_monitor")

                sub = self._create_subscription(
                    temp_node,
                    self.process_management_configuration.process_monitor_command[0],
                    0
                )
                
                # while True:
                while self.process_management_configuration.process_start_timeout is None or (datetime.now() - start_time) < self.process_management_configuration.process_start_timeout:
                    if self._process.poll() is not None:
                        stop_success = self.stop()
                        if not stop_success:
                            raise RuntimeError("Failed to stop process after failed start.")
                        
                        return False

                    rclpy.spin_once(temp_node, timeout_sec=1)
                    
                    # monitor_start_time = datetime.now()
                    
                    if self.is_running():
                        success = True
                        break
                    
                    # monitor_end_time = datetime.now()
                    # monitor_duration = monitor_end_time - monitor_start_time
                    
                    # process_monitor_period_remaining = self.process_management_configuration.process_monitor_period - monitor_duration

                    # if process_monitor_period_remaining.total_seconds() > 0:
                        # sleep(process_monitor_period_remaining.total_seconds())
                    # rate = self._parent_node.create_rate(10)
                        # rate = self._parent_node.create_rate(1. / process_monitor_period_remaining.total_seconds())
                    # rate.sleep()

                sub.destroy()
                temp_node.destroy_node()
                
                del sub
                del temp_node

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
            # self._process.kill()
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

                    # sleep(self.process_management_configuration.process_monitor_period.total_seconds())
                    rate = self._parent_node.create_rate(1 / self.process_management_configuration.process_monitor_period.total_seconds())
                    rate.sleep()

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
            process_monitor_cnt = 0
            
            for process_monitor_command_dict in self.process_management_configuration.process_monitor_command:
                command_type = process_monitor_command_dict["type"]
                
                if command_type == "command":
                    raise NotImplementedError("Command process monitoring not implemented.")
                    # command = process_monitor_command_dict["command"]
                    
                    # monitor_process = subprocess.Popen(
                    #     command,
                    #     cwd=self.process_management_configuration.working_directory,
                    #     shell=True,
                    #     start_new_session=True,
                    #     executable='/bin/bash'
                    # )
                    # monitor_process.wait()

                    # if monitor_process.returncode != 0:
                    #     return False
                
                elif command_type == "topic":
                    with self._last_monitor_message_ok_time_locks[process_monitor_cnt]:
                       last_monitor_message_ok_time = self._last_monitor_message_ok_times[process_monitor_cnt]

                    if last_monitor_message_ok_time is None:
                        return False

                    monitor_config = self._monitor_topic_configs[process_monitor_cnt]
                    timeout_sec = monitor_config["timeout_sec"]

                    if (self._parent_node.get_clock().now() - last_monitor_message_ok_time).nanoseconds / 1e9 > timeout_sec:
                        return False

                    process_monitor_cnt += 1
                    
                    continue
                
                else:
                    raise NotImplementedError(f"Unsupported process monitor command type: {command_type}")

        return self._is_started
    
    @property
    def is_started(self) -> bool:
        return self._is_started
    
    