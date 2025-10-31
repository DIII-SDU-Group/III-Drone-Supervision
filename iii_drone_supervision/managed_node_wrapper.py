#!/usr/bin/python3

#########################################################################
# Imports:
#########################################################################

from typing import Optional
import argparse
import threading
import time
import os
import sys

import rclpy
from rclpy.timer import Timer
from rclpy.logging import set_logger_level

from rclpy.lifecycle import Node, State, TransitionCallbackReturn

from iii_drone_supervision.process_management_configuration import ProcessManagementConfiguration
from iii_drone_supervision.managed_process import ManagedProcess

#########################################################################
# Debugging:
#########################################################################

SIMULATION = os.environ.get('SIMULATION', 'false').lower() == 'true'

if SIMULATION:
    import debugpy

#########################################################################
# Class:
#########################################################################

class ManagedNodeWrapper(Node):
    """
        Class for wrapping any process as a managed node.
    """
    
    def __init__(
        self,
        process_management_configuration: ProcessManagementConfiguration,
        **kwargs
    ) -> None:
        """
            Constructor.
        """
        
        self.process_management_configuration = process_management_configuration
        
        # Initialize the superclass:
        super().__init__(
            self.process_management_configuration.node_name,
            namespace=self.process_management_configuration.node_namespace,
            **kwargs
        )

        self.get_logger().set_level(
            self.process_management_configuration.log_level
        )
        
        self.managed_process = ManagedProcess(
            self.process_management_configuration,
            self
        )
        
        self.process_monitor_timer: Optional[Timer] = None
        
        self.get_logger().info(f"Managed node wrapper '{self.process_management_configuration.node_name}' initialized.")

        self._error = False

    def cleanup(
        self
    ) -> bool:
        if self.process_monitor_timer:
            self.process_monitor_timer.destroy()
            self.process_monitor_timer = None

        success = True
            
        if self.managed_process:
            success &= self.managed_process.stop()
            success &= self.managed_process.cleanup()

        self.get_logger().debug(f"Finished cleaning up.")
            
        return success

    def on_configure(
        self, 
        state: State
    ) -> TransitionCallbackReturn:
        """
            Callback for the configure transition.
        """

        self.get_logger().debug(f"Configuring...")

        ret = super().on_configure(state)

        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error(f"Base class configure failed.")
            return ret
        
        try:
            success = self.managed_process.configure()
            
        except Exception as e:
            self.get_logger().fatal("Configure failed with unknown exception: " + str(e))
            return TransitionCallbackReturn.ERROR
        
        if success:
            self.get_logger().debug(f"Configuration succeeded.")
            
            return TransitionCallbackReturn.SUCCESS
        
        self.get_logger().error(f"Configuration failed.")
        
        return TransitionCallbackReturn.FAILURE
    
    def on_activate(
        self, 
        state: State
    ) -> TransitionCallbackReturn:
        """
            Callback for the activate transition.
        """

        self.get_logger().debug(f"Activating...")
        
        ret = super().on_activate(state)

        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error(f"Base class activate failed.")
            return ret
        
        try:
            success = self.managed_process.start()
            self.process_monitor_timer = self.create_timer(
                self.process_management_configuration.process_monitor_period.total_seconds(),
                self.process_monitor_callback
            )
            
        except Exception as e:
            self.get_logger().fatal("Activate failed with unknown exception: " + str(e))
            return TransitionCallbackReturn.ERROR
        
        if success:
            self.get_logger().debug(f"Activate succeeded.")
            return TransitionCallbackReturn.SUCCESS
        
        self.get_logger().error(f"Activate failed.")
        return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(
        self, 
        state: State
    ) -> TransitionCallbackReturn:
        """
            Callback for the deactivate transition.
        """
        
        if self._error:
            self._error = False
            raise RuntimeError("Error occurred, skipping cleanup.")
    
        self.get_logger().debug(f"Deactivating...")

        ret = super().on_deactivate(state)
        
        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error(f"Base class deactivate failed.")
            return ret
        
        try:
            self.process_monitor_timer.cancel()
            self.process_monitor_timer.destroy()
            self.process_monitor_timer = None
            success = self.managed_process.stop()
            
        except Exception as e:
            self.get_logger().fatal("Deactivate failed with unknown exception: " + str(e))
            return TransitionCallbackReturn.ERROR
        
        if success:
            self.get_logger().debug(f"Deactivate succeeded.")
            return TransitionCallbackReturn.SUCCESS
        
        self.get_logger().error(f"Deactivate failed.")
        return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(
        self, 
        state: State
    ) -> TransitionCallbackReturn:
        """
            Callback for the cleanup transition.
        """

        self.get_logger().debug(f"Cleaning up...")
        
        ret = super().on_cleanup(state)
        
        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error(f"Base class cleanup failed.")
            return ret
        
        try:
            success = self.cleanup()
            
        except Exception as e:
            self.get_logger().fatal("Cleanup failed with unknown exception: " + str(e))
            return TransitionCallbackReturn.ERROR
        
        if success:
            self.get_logger().debug(f"Cleanup succeeded.")
            return TransitionCallbackReturn.SUCCESS
        
        self.get_logger().error(f"Cleanup failed.")
        return TransitionCallbackReturn.FAILURE
    
    def on_shutdown(
        self, 
        state: State
    ) -> TransitionCallbackReturn:
        """
            Callback for the shutdown transition.
        """

        self.get_logger().debug(f"Shutting down...")
        
        ret = super().on_shutdown(state)
        
        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error(f"Base class shutdown failed.")
            return ret
        
        try:
            success = self.cleanup()
            
        except Exception as e:
            self.get_logger().fatal("Shutdown failed with unknown exception: " + str(e))
            return TransitionCallbackReturn.ERROR
        
        if success:
            self.get_logger().debug(f"Shutdown succeeded.")

            def shutdown_rclpy():
                time.sleep(1)
                rclpy.shutdown()

            thread = threading.Thread(target=shutdown_rclpy)
            thread.start()
            
            return TransitionCallbackReturn.SUCCESS
        
        self.get_logger().error(f"Shutdown failed.")
        return TransitionCallbackReturn.FAILURE
    
    def on_error(
        self, 
        state: State
    ) -> TransitionCallbackReturn:
        """
            Callback for the error transition.
        """
        
        self.get_logger().debug(f"Error...")

        ret = super().on_error(state)
        
        if ret == TransitionCallbackReturn.ERROR or ret == TransitionCallbackReturn.FAILURE:
            self.get_logger().error(f"Base class error failed.")
            return ret
        
        try:
            success = self.cleanup()
            
        except Exception as e:
            self.get_logger().fatal("Error failed with unknown exception: " + str(e))
            return TransitionCallbackReturn.ERROR
        
        if success:
            self.get_logger().debug(f"Error succeeded.")
            return TransitionCallbackReturn.SUCCESS
        
        self.get_logger().error(f"Error failed.")
        return TransitionCallbackReturn.FAILURE

    def process_monitor_callback(
        self
    ) -> None:
        """
            Callback for the process monitor timer.
        """
        
        if not self.managed_process.is_running():
            self.get_logger().error("Process is not running.")
            self._error = True
            self.trigger_deactivate()
        else:
            self.get_logger().debug("Process is running.")

#########################################################################
# Main:
#########################################################################

def main() -> None:
    """
        Main function.
    """
    
    parser = argparse.ArgumentParser(
        description="Managed node wrapper."
    )
    
    parser.add_argument(
        "configuration_file",
        type=str,
        help="The process management configuration file."
    )
    
    # Ignore everything after --ros-args
    args, _ = parser.parse_known_args()
    
    
    # args = parser.parse_args()

    rclpy.init(args=sys.argv)
    
    process_management_configuration = ProcessManagementConfiguration(
        args.configuration_file
    )

    if SIMULATION:
        DEBUG_PORT = int(os.environ.get(f'{process_management_configuration.node_name.upper()}_DEBUG_PORT', 0))
        
        if DEBUG_PORT > 0:
            debugpy.listen(
                (
                    'localhost',
                    DEBUG_PORT
                )
            )
            
            print("Listening for debugger on port " + str(DEBUG_PORT))

    
    managed_node_wrapper = ManagedNodeWrapper(
        process_management_configuration
    )
    
    # multi_threaded_executor = rclpy.executors.MultiThreadedExecutor()
    
    # multi_threaded_executor.add_node(managed_node_wrapper)
    
    try:
        # multi_threaded_executor.spin()
        rclpy.spin(managed_node_wrapper)
        managed_node_wrapper.destroy_node()
        
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, rclpy.exceptions.ROSInterruptException):
        print("Received shutdown request. Shutting down gracefully.")
        managed_node_wrapper.cleanup()
        managed_node_wrapper.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("Finished shutting down.")
    
    return

if __name__ == "__main__":
    main()