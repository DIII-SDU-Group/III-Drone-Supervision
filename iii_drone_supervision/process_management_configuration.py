#########################################################################
# Imports:
#########################################################################

from typing import Optional
from datetime import timedelta
import yaml

from rclpy.logging import LoggingSeverity
import os

#########################################################################
# Defines:
#########################################################################

PROCESS_MANAGEMENT_CONFIGURATION_FILE_KEYS = [
    ("node_name", str, True),
    ("node_namespace", str, True),
    ("log_level", str, False),

    ("command", str, True),
    ("working_directory", str, True),

    ("process_monitor_command", list, False),
    ("process_monitor_period_sec", int, True),
    ("process_start_timeout_sec", int, False),
    ("process_stop_timeout_sec", int, False)
]

#########################################################################
# Class:
#########################################################################

class ProcessManagementConfiguration:
    def __init__(
        self,
        process_management_configuration_file: str
    ) -> None:
        with open(process_management_configuration_file, "r") as file:
            self.process_management_configuration = yaml.safe_load(file)
            
        self.verify_process_management_configuration(self.process_management_configuration)
        if "process_monitor_command" in self.process_management_configuration:
            self.verify_process_monitor_command(self.process_management_configuration["process_monitor_command"])
        
        self._node_name = self.process_management_configuration["node_name"]
        self._node_namespace = self.process_management_configuration["node_namespace"]
        self._log_level = self.process_management_configuration.get("log_level", "info")
        self._log_level = self._log_level.lower()

        if self._log_level == "debug":
            self._log_level = LoggingSeverity.DEBUG
        elif self._log_level == "info":
            self._log_level = LoggingSeverity.INFO
        elif self._log_level == "warn":
            self._log_level = LoggingSeverity.WARN
        elif self._log_level == "error":
            self._log_level = LoggingSeverity.ERROR
        elif self._log_level == "fatal":
            self._log_level = LoggingSeverity.FATAL
        else:
            raise ValueError(f"Invalid value for field 'log_level' in process management configuration file, must be one of 'debug', 'info', 'warn', 'error', 'fatal'.")

        self._command = self.process_management_configuration["command"]
        self._working_directory = os.path.expandvars(self.process_management_configuration["working_directory"])

        self._process_monitor_command = self.process_management_configuration.get("process_monitor_command", None)
        self._process_monitor_period = timedelta(seconds=self.process_management_configuration["process_monitor_period_sec"])
        self._process_start_timeout = timedelta(seconds=self.process_management_configuration.get("process_start_timeout_sec")) if "process_start_timeout_sec" in self.process_management_configuration else None
        self._process_stop_timeout = timedelta(seconds=self.process_management_configuration.get("process_stop_timeout_sec")) if "process_stop_timeout_sec" in self.process_management_configuration else None

        if (self._process_start_timeout is not None or self._process_stop_timeout is not None) and self._process_monitor_command is None:
            raise ValueError("Process start and stop timeouts require a process monitor command.")

        if self._process_start_timeout and self._process_start_timeout < self._process_monitor_period * 2:
            raise ValueError("Process start timeout must be at least twice the process monitor period.")
        
        if self._process_stop_timeout and self._process_stop_timeout < self._process_monitor_period * 2:
            raise ValueError("Process stop timeout must be at least twice the process monitor period.")
        
    @property
    def node_name(self) -> str:
        return self._node_name
    
    @property
    def node_namespace(self) -> str:
        return self._node_namespace

    @property
    def log_level(self) -> LoggingSeverity:
        return self._log_level
    
    @property
    def command(self) -> str:
        return self._command
    
    @property
    def working_directory(self) -> str:
        return self._working_directory
    
    @property
    def process_monitor_command(self) -> Optional[list]:
        return self._process_monitor_command

    @property
    def process_monitor_period(self) -> timedelta:
        return self._process_monitor_period

    @property
    def process_start_timeout(self) -> Optional[timedelta]:
        return self._process_start_timeout
    
    @property
    def process_stop_timeout(self) -> Optional[timedelta]:
        return self._process_stop_timeout

    @staticmethod
    def verify_process_management_configuration(
        process_management_configuration: dict
    ) -> None:
        config_keys = list(process_management_configuration.keys())
        
        for key, tp, required in PROCESS_MANAGEMENT_CONFIGURATION_FILE_KEYS:
            if required and key not in process_management_configuration:
                raise ValueError(f"Missing required field '{key}' in process management configuration file.")
            
            if key in process_management_configuration and not isinstance(process_management_configuration[key], tp):
                raise ValueError(f"Invalid value for field '{key}' in process management configuration file, must be of type '{tp}'.")

            # Pop key from config_keys:
            try:
                config_keys.remove(key)
            except ValueError:
                pass
            
        if len(config_keys) > 0:
            raise ValueError(f"Unknown fields in process management configuration file: {config_keys}")

    @staticmethod
    def verify_process_monitor_command(
        process_monitor_command: list
    ) -> None:
        for cmd in process_monitor_command:
            if "type" not in cmd:
                raise ValueError("Missing required field 'type' in process monitor command.")
            
            type_ = cmd["type"]
            
            if type_ == "command":
                if "command" not in cmd:
                    raise ValueError("Missing required field 'command' in process monitor command for type 'bash'.")
                
                if not isinstance(cmd["command"], str):
                    raise ValueError("Invalid value for field 'command' in process monitor command for type 'bash', must be of type 'str'.")
                
            elif type_ == "topic":
                if "topic" not in cmd:
                    raise ValueError("Missing required field 'topic' in process monitor command for type 'topic'.")
                
                if not isinstance(cmd["topic"], str):
                    raise ValueError("Invalid value for field 'topic' in process monitor command for type 'topic', must be of type 'str'.")
                
                if "type" not in cmd:
                    raise ValueError("Missing required field 'type' in process monitor command for type 'topic'.")
                
                if not isinstance(cmd["type"], str):
                    raise ValueError("Invalid value for field 'type' in process monitor command for type 'topic', must be of type 'str'.")
                
                if "timeout_sec" not in cmd:
                    raise ValueError("Missing required field 'timeout_sec' in process monitor command for type 'topic'.")
                
                if not isinstance(cmd["timeout_sec"], int):
                    raise ValueError("Invalid value for field 'timeout_sec' in process monitor command for type 'topic', must be of type 'int'.")
                
            else:
                raise ValueError(f"Invalid value '{type_}' for field 'type' in process monitor command, must be one of 'command', 'topic'.")