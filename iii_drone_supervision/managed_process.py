#########################################################################
# Imports:
#########################################################################

from datetime import timedelta, datetime
from time import sleep
import subprocess
from subprocess import signal
import os
import threading

from iii_drone_supervision.process_management_configuration import ProcessManagementConfiguration

#########################################################################
# Class:
#########################################################################

class ManagedProcess:
    def __init__(
        self,
        process_management_configuration: ProcessManagementConfiguration
    ) -> None:
        self.process_management_configuration = process_management_configuration

        self._is_started = False
        self._process = None
        self.cnt = 0
    #     self._output_thread = None

    # def _read_output(
    #     self
    # ) -> None:
    #     for line in iter(self._process.stdout.readline, b''):
    #         print(line.decode(), end='', flush=True)

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
            # command = 'stdbuf -o0 ' + self.process_management_configuration.command

            start_time = datetime.now()

            self._process = subprocess.Popen(
                # command,
                self.process_management_configuration.command,
                cwd=self.process_management_configuration.working_directory,
                shell=True,
                start_new_session=True,
                executable='/bin/bash',
                # stdout=subprocess.PIPE if self.process_management_configuration.attach else None,
            )

            # if self.process_management_configuration.attach:
            #     self._output_thread = threading.Thread(target=self._read_output)
            #     self._output_thread.start()
            
            self._is_started = True
            
            if self.process_management_configuration.process_start_timeout is not None and self.process_management_configuration.process_monitor_command is not None:
                success = False
                while (datetime.now() - start_time) < self.process_management_configuration.process_start_timeout:
                    if self._process.poll() is not None:
                        stop_success = self.stop()
                        if not stop_success:
                            raise RuntimeError("Failed to stop process after failed start.")
                        
                        return False
                    if self.is_running():
                        success = True
                        break

                    sleep(self.process_management_configuration.process_monitor_period.total_seconds())

                if not success:
                    stop_success = self.stop()
                    if not stop_success:
                        raise RuntimeError("Failed to stop process after failed start.")
                    
                    return False
        
        return True
    
    def stop(
        self
    ) -> bool:
        if self._is_started and self.is_running():
            os.killpg(self._process.pid, signal.SIGKILL)
            self._process.wait()
            self._process = None

            self._is_started = False
            
            if self.process_management_configuration.process_stop_timeout is not None and self.process_management_configuration.process_monitor_command is not None:
                success = False
                
                start_time = datetime.now()
                while (datetime.now() - start_time) < self.process_management_configuration.process_stop_timeout:
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
        self.cnt += 1
        if self.cnt == 10:
            return False

        if self._process is None:
            return False
        
        if self._process.poll() is not None:
            return False
        
        if self.process_management_configuration.process_monitor_command is not None:
            monitor_process = subprocess.Popen(
                self.process_management_configuration.process_monitor_command,
                cwd=self.process_management_configuration.working_directory,
                shell=True,
                start_new_session=True,
                executable='/bin/bash'
            )
            monitor_process.wait()

            return monitor_process.returncode == 0

        return self._is_started
    
    @property
    def is_started(self) -> bool:
        return self._is_started
    
    