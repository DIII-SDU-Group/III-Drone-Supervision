#!/usr/bin/python3

#########################################################################
# Imports:
#########################################################################

import argparse
from typing import Optional
from threading import Thread
from time import sleep
import os

import rclpy
from rclpy.timer import Timer
from rclpy.node import Node
from rclpy.service import Service

from iii_drone_interfaces.srv import SupervisorStart, SupervisorStop, SupervisorShutdown

from iii_drone_supervision.supervisor import Supervisor

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
        
        self.supervision_config_file = supervision_config_file
        
        self.supervisor = Supervisor(
            self.supervision_config_file,
            self
        )

        # Initialize the Services:
        self.start_service = self.create_service(
            SupervisorStart,
            'start',
            self.start_callback
        )
        
        self.stop_service = self.create_service(
            SupervisorStop,
            'stop',
            self.stop_callback
        )
        
        self.shutdown_service = self.create_service(
            SupervisorShutdown,
            'shutdown',
            self.shutdown_callback
        )

        # Print the Message:
        self.get_logger().info('SupervisorNode.__init__(): Supervisor has been initialized.')

    def start_callback(
        self,
        request: SupervisorStart.Request,
        response: SupervisorStart.Response
    ) -> SupervisorStart.Response:
        """
            Callback for starting supervisor.
        """
        
        action = request.action
        start_to_node = request.start_to_node
        
        if start_to_node != "":
            raise NotImplementedError('SupervisorNode.start_callback(): Starting to a specific node is not implemented yet.')

        if action not in [SupervisorStart.Request.START_ACTION_ACTIVATE, SupervisorStart.Request.START_ACTION_CONFIGURE]:
            raise ValueError(f'SupervisorNode.start_callback(): Received invalid action {action}.')

        self.get_logger().info('SupervisorNode.start_callback(): Starting...')
        
        try:
            success = self.supervisor.start(activate=action == SupervisorStart.Request.START_ACTION_ACTIVATE)
        except Exception as e:
            self.on_error(e)
            response.success = False
            return response
        
        if not success:
            self.get_logger().error('SupervisorNode.start_callback(): Failed to start the system.')
            response.success = False
            return response

        self.get_logger().info('SupervisorNode.start_callback(): System has been started.')
        
        response.success = True
        return response
    
    def stop_callback(
        self,
        request: SupervisorStop.Request,
        response: SupervisorStop.Response
    ) -> SupervisorStop.Response:
        """
            Callback for stopping supervisor.
        """
        
        self.get_logger().info('SupervisorNode.stop_callback(): Stopping...')

        action = request.action
        stop_from_node = request.stop_from_node
        
        if stop_from_node != "":
            raise NotImplementedError('SupervisorNode.stop_callback(): Stopping from a specific node is not implemented yet.')
        
        if action not in [SupervisorStop.Request.STOP_ACTION_DEACTIVATE, SupervisorStop.Request.STOP_ACTION_CLEANUP]:
            raise ValueError(f'SupervisorNode.stop_callback(): Received invalid action {action}.')
        
        try:
            success = self.supervisor.stop(
                cleanup=action == SupervisorStop.Request.STOP_ACTION_CLEANUP
            )
        except Exception as e:
            self.on_error(e)
            response.success = False
            return response
        
        if not success:
            self.get_logger().error('SupervisorNode.stop_callback(): Failed to stop the system.')
            response.success = False
            return response
        
        self.get_logger().info('SupervisorNode.stop_callback(): System has been stopped.')
        
        response.success = True
        return response

    def shutdown_callback(
        self,
        request: SupervisorShutdown.Request,
        response: SupervisorShutdown.Response
    ) -> SupervisorShutdown.Response:
        """
            Callback for shutdown.
        """
        
        self.get_logger().info('SupervisorNode.shutdown_callback(): Shutting down...')

        try:
            success, error_nodes = self.supervisor.shutdown()
        except Exception as e:
            self.on_error(e)
            response.success = False
            return response
        
        if not success:
            self.on_error(
                Exception(
                    f"SupervisorNode.shutdown_callback(): Failed to shut down some nodes: {error_nodes}"
                )
            )
            response.success = False
            return response
        
        def shutdown():
            sleep(1)
            self.get_logger().info('SupervisorNode.shutdown_callback(): Shutting down supervisor.')
            rclpy.shutdown()
        
        shutdown_thread = Thread(target=shutdown)
        
        self.get_logger().info('SupervisorNode.shutdown_callback(): System has been shut down.')
        
        response.success = True
        
        shutdown_thread.start()

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
    parser = argparse.ArgumentParser("III-Drone supervisor")
    parser.add_argument(
        '--config-file',
        type=str,
        default=os.environ.get('SUPERVISOR_CONFIG_FILE', None),
        help='Path to the supervision configuration file.'
    )
    
    args = parser.parse_args()

    if not args.config_file:
        raise RuntimeError('SupervisorNode: No supervision configuration file provided. Either provide it as an argument or set the SUPERVISOR_CONFIG_FILE environment variable.')
    args = parser.parse_args()

    rclpy.init()
    
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
    
    
    supervisor_node = SupervisorNode(args.config_file)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(supervisor_node)
    
    try:
        executor.spin()
        if rclpy.ok():
            rclpy.shutdown()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, rclpy.exceptions.ROSInterruptException):
        print("Finished")
    