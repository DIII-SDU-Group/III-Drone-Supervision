#!/usr/bin/python3

#########################################################################
# Imports:
#########################################################################

import argparse
from typing import Optional
from threading import Thread, Lock
from time import sleep
import os
import sys

import rclpy
from rclpy.timer import Timer
from rclpy.node import Node
from rclpy.service import Service
from rclpy.action import ActionServer

from iii_drone_interfaces.srv import GetManagedNodes
from iii_drone_interfaces.action import SupervisorStart, SupervisorStop, SupervisorRestart, SupervisorShutdown

from iii_drone_supervision.supervisor import Supervisor

#########################################################################
# Debugging:
#########################################################################

SIMULATION = os.environ.get('SIMULATION', 'false').lower() == 'true'

if SIMULATION:
    import debugpy

#########################################################################
# Class:
#########################################################################

class SupervisorNode(Node):
    """
        Node supervising the execution of the III-Drone system.
    """
    
    def __init__(
        self,
        supervision_config_file: str,
        **kwargs
    ):
        """
            Constructor of the Supervisor class.
        """
        
        # Initialize the Node:
        super().__init__(
            node_name='supervisor',
            namespace='supervision',
            **kwargs
        )
        
        # Get environment variable SUPERVISOR_LOG_LEVEL, set log level accordingly:
        log_level = os.environ.get('SUPERVISOR_LOG_LEVEL')
        
        if log_level is not None:
            log_level = log_level.upper()
            
            if log_level == 'DEBUG':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            elif log_level == 'INFO':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
            elif log_level == 'WARN':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
            elif log_level == 'ERROR':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
            elif log_level == 'FATAL':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)

        self.declare_parameter("monitor_node_states", False)
        self.monitor_node_states = self.get_parameter("monitor_node_states").value

        if self.monitor_node_states:
            self.get_logger().info('SupervisorNode.__init__(): Monitoring node states.')
        else:
            self.get_logger().info('SupervisorNode.__init__(): Not monitoring node states.')
        
        self.supervision_config_file = supervision_config_file
        
        self.supervisor = Supervisor(
            self.supervision_config_file,
            self.monitor_node_states,
            self
        )
        
        # Initialize the actions:
        self.start_action_server = ActionServer(
            self,
            SupervisorStart,
            'supervisor/start',
            self.start_callback
        )
        
        self.stop_action_server = ActionServer(
            self,
            SupervisorStop,
            'supervisor/stop',
            self.stop_callback
        )
        
        self.restart_action_server = ActionServer(
            self,
            SupervisorRestart,
            'supervisor/restart',
            self.restart_callback
        )
        
        self.shutdown_action_server = ActionServer(
            self,
            SupervisorShutdown,
            'supervisor/shutdown',
            self.shutdown_callback
        )
        
        # Initialize the services:
        self.get_managed_nodes_service = self.create_service(
            GetManagedNodes,
            'supervisor/get_managed_nodes',
            self.get_managed_nodes_callback
        )

        # Print the Message:
        if SIMULATION:
            self.get_logger().info('SupervisorNode.__init__(): Supervisor has been initialized in simulation mode.')
        else:
            self.get_logger().info('SupervisorNode.__init__(): Supervisor has been initialized.')

    def start_callback(
        self,
        goal_handle: rclpy.action.server.ServerGoalHandle
    ) -> SupervisorStart.Result:
        """
            Callback for starting supervisor.
        """
        
        result = SupervisorStart.Result()
        feedback = SupervisorStart.Feedback()
        
        action = goal_handle.request.action
        activate = action == SupervisorStart.Goal.START_ACTION_ACTIVATE
        select_nodes = goal_handle.request.select_nodes
        ignore_dependencies = goal_handle.request.ignore_dependencies
        
        if action not in [SupervisorStart.Goal.START_ACTION_ACTIVATE, SupervisorStart.Goal.START_ACTION_CONFIGURE]:
            message = f'Received invalid action {action}.'
            self.get_logger().error(f'SupervisorNode.start_callback(): {message}')
            
            result.success = False
            result.message = message
            
            goal_handle.abort()
            
            return result

        message = "Starting..."
        self.get_logger().info(f'SupervisorNode.start_callback(): {message}')

        def publish_feedback(message: str):
            feedback.message = message
            goal_handle.publish_feedback(feedback)
        
        publish_feedback(message)
        
        try:
            success, _ = self.supervisor.start(
                activate=activate,
                message_callback=publish_feedback,
                select_nodes=select_nodes,
                ignore_dependencies=ignore_dependencies
            )
        except Exception as e:
            self.on_error(e)
            
            message = 'Failed to start the system with exception:\n' + str(e.with_traceback())
            self.get_logger().error(f'SupervisorNode.start_callback(): {message}')

            result.success = False
            result.message = message
            
            goal_handle.abort()
            
            return result

        if not rclpy.ok():
            return result
        
        if not success:
            message = 'Failed to start the system.'
            self.get_logger().error(f'SupervisorNode.start_callback(): {message}')

            result.success = False
            result.message = message
            
            goal_handle.abort()

            return result

        message = 'System has been started.'
        self.get_logger().info(f'SupervisorNode.start_callback(): {message}')
        
        result.success = True
        result.message = message
        
        goal_handle.succeed()
        
        return result
    
    def stop_callback(
        self,
        goal_handle: rclpy.action.server.ServerGoalHandle
    ) -> SupervisorStop.Result:
        """
            Callback for stopping supervisor.
        """
        
        result = SupervisorStop.Result()
        feedback = SupervisorStop.Feedback()
        
        action = goal_handle.request.action
        select_nodes = goal_handle.request.select_nodes
        ignore_dependencies = goal_handle.request.ignore_dependencies
        
        if action not in [SupervisorStop.Goal.STOP_ACTION_DEACTIVATE, SupervisorStop.Goal.STOP_ACTION_CLEANUP]:
            message = f'Received invalid action {action}.'
            self.get_logger().error(f'SupervisorNode.stop_callback(): {message}')
            
            result.success = False
            result.message = message
            
            goal_handle.abort()
            
            return result

        def publish_feedback(message: str):
            # with goal_handle_lock:
            feedback.message = message
            goal_handle.publish_feedback(feedback)
        
        message = "Stopping..."
        self.get_logger().info(f'SupervisorNode.stop_callback(): {message}')

        publish_feedback(message)
        
        try:
            success, _ = self.supervisor.stop(
                cleanup=action == SupervisorStop.Goal.STOP_ACTION_CLEANUP,
                message_callback=publish_feedback,
                select_nodes=select_nodes,
                ignore_dependencies=ignore_dependencies
            )
        except Exception as e:
            self.on_error(e)
            
            message = 'Failed to stop the system with exception:\n' + str(e.with_traceback())
            self.get_logger().error(f'SupervisorNode.stop_callback(): {message}')

            result.success = False
            result.message = message
            
            goal_handle.abort()
            
            return result
        
        if not success:
            message = 'Failed to stop the system.'
            self.get_logger().error(f'SupervisorNode.stop_callback(): {message}')

            result.success = False
            result.message = message
            
            goal_handle.abort()

            return result

        message = 'System has been stopped.'
        self.get_logger().info(f'SupervisorNode.stop_callback(): {message}')
        
        result.success = True
        result.message = message
        
        goal_handle.succeed()
        
        return result
    
    def restart_callback(
        self,
        goal_handle: rclpy.action.server.ServerGoalHandle
    ) -> SupervisorRestart.Result:
        """
            Callback for restarting supervisor.
        """
        
        result = SupervisorRestart.Result()
        feedback = SupervisorRestart.Feedback()
        
        restart_type = goal_handle.request.restart_type
        select_nodes = goal_handle.request.select_nodes
        ignore_dependencies = goal_handle.request.ignore_dependencies
        
        if restart_type not in [SupervisorRestart.Goal.RESTART_TYPE_WARM, SupervisorRestart.Goal.RESTART_TYPE_COLD]:
            message = f'Received invalid restart type {restart_type}.'
            self.get_logger().error(f'SupervisorNode.restart_callback(): {message}')
            
            result.success = False
            result.message = message
            
            goal_handle.abort()
            
            return result

        def publish_feedback(message: str):
            feedback.message = message
            goal_handle.publish_feedback(feedback)
        
        message = "Restarting..."
        self.get_logger().info(f'SupervisorNode.restart_callback(): {message}')

        publish_feedback(message)
        
        try:
            success, stopped_nodes = self.supervisor.stop(
                cleanup=restart_type == SupervisorRestart.Goal.RESTART_TYPE_COLD,
                message_callback=publish_feedback,
                select_nodes=select_nodes,
                ignore_dependencies=ignore_dependencies
            )
        except Exception as e:
            self.on_error(e)
            
            message = 'Failed to stop the system with exception:\n' + str(e.with_traceback())
            self.get_logger().error(f'SupervisorNode.restart_callback(): {message}')

            result.success = False
            result.message = message
            
            goal_handle.abort()
            
            return result

        if not success:
            message = 'Failed to restart the system.'
            self.get_logger().error(f'SupervisorNode.restart_callback(): {message}')

            result.success = False
            result.message = message
            
            goal_handle.abort()

            return result
        
        try:
            success, _ = self.supervisor.start(
                activate=True,
                message_callback=publish_feedback,
                select_nodes=select_nodes,
                restart_nodes=stopped_nodes,
                ignore_dependencies=ignore_dependencies
            )
        except Exception as e:
            self.on_error(e)
            
            message = 'Failed to start the system with exception:\n' + str(e.with_traceback())
            self.get_logger().error(f'SupervisorNode.restart_callback(): {message}')

            result.success = False
            result.message = message
            
            goal_handle.abort()
            
            return result
        
        if not success:
            message = 'Failed to restart the system.'
            self.get_logger().error(f'SupervisorNode.restart_callback(): {message}')

            result.success = False
            result.message = message
            
            goal_handle.abort()

            return result

        message = 'System has been restarted.'
        self.get_logger().info(f'SupervisorNode.restart_callback(): {message}')
        
        result.success = True
        result.message = message
        
        goal_handle.succeed()
        
        return result

    def shutdown_callback(
        self,
        goal_handle: rclpy.action.server.ServerGoalHandle
    ) -> SupervisorShutdown.Result:
        """
            Callback for shutdown.
        """
        
        result = SupervisorShutdown.Result()
        feedback = SupervisorShutdown.Feedback()

        def publish_feedback(message: str):
            feedback.message = message
            goal_handle.publish_feedback(feedback)
        
        message = "Shutting down..."
        self.get_logger().info(f'SupervisorNode.shutdown_callback(): {message}')

        publish_feedback(message)
        
        try:
            success, error_nodes = self.supervisor.shutdown(
                message_callback=publish_feedback
            )
        except Exception as e:
            self.on_error(e)
            
            message = 'Failed to shut down the system with exception:\n' + str(e.with_traceback())
            self.get_logger().error(f'SupervisorNode.shutdown_callback(): {message}')

            result.success = False
            result.message = message
            
            goal_handle.abort()
            
            return result
        
        if not success:
            message = f'Failed to shut down the system. Failed nodes: {error_nodes}'
            self.get_logger().error(f'SupervisorNode.shutdown_callback(): {message}')

            result.success = False
            result.message = message
            
            goal_handle.abort()

            return result

        message = 'System has been shut down.'
        self.get_logger().info(f'SupervisorNode.shutdown_callback(): {message}')
        
        result.success = True
        result.message = message
        
        goal_handle.succeed()

        def shutdown():
            sleep(1)
            self.get_logger().info('SupervisorNode.shutdown_callback(): Shutting down supervisor.')
            rclpy.shutdown()
        
        shutdown_thread = Thread(target=shutdown)
        
        shutdown_thread.start()

        return result

    def get_managed_nodes_callback(
        self,
        request: GetManagedNodes.Request,
        response: GetManagedNodes.Response
    ) -> GetManagedNodes.Response:
        """
            Callback for getting managed nodes.
        """
        
        response.managed_nodes = self.supervisor.managed_nodes
        
        return response

    def on_error(
        self,
        error: Exception
    ):
        """
            Callback for the error transition.
        """

        self.get_logger().error(f'SupervisorNode.on_error(): {error.with_traceback()}')

#########################################################################
# Main:
#########################################################################

def main():
    """
        Main function of the SupervisorNode.
    """
    if SIMULATION:
        SUPERVISOR_DEBUG_PORT = int(os.environ.get('SUPERVISOR_DEBUG_PORT', 0))
        
        if SUPERVISOR_DEBUG_PORT > 0:
            debugpy.listen(
                (
                    'localhost',
                    SUPERVISOR_DEBUG_PORT
                )
            )
            
            print("Listening for debugger on port %d..." % SUPERVISOR_DEBUG_PORT)

    if SIMULATION:
        config_file=os.environ.get('SUPERVISOR_CONFIG_FILE_SIM', None)
    else:
        config_file=os.environ.get('SUPERVISOR_CONFIG_FILE', None)
        
    if config_file is None:
        raise RuntimeError('Supervisor config file environment variable not set.')

    rclpy.init(args=sys.argv)
    
    # node = rclpy.create_node('supervisor_node')

    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(node)

    # thread = Thread(target=executor.spin)
    # thread.start()
    
    # supervisor = Supervisor(args.config_file, node)
    
    # supervisor.start()

    # for i in range(2):
    #     sleep(1)
    #     print("running")

    # supervisor.stop()
    # rclpy.shutdown()
    
    supervisor_node = SupervisorNode(config_file)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(supervisor_node)
    
    try:
        executor.spin()
        if rclpy.ok():
            rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, rclpy.exceptions.ROSInterruptException):
        print("Finished")

    if rclpy.ok():
        print("Shutting down...")
        rclpy.shutdown()
    