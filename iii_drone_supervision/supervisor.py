"""Core supervision engine for III managed-node orchestration.

This module validates the supervision graph, expands dependency transitions,
tracks managed-node clients, and executes bringup/bringdown flows while
respecting configuration and activation dependencies.
"""

#########################################################################
# Imports:
#########################################################################

import yaml
from copy import deepcopy
from typing import Optional
from threading import Thread, Lock
from time import sleep

import rclpy
import rclpy.callback_groups
from rclpy.lifecycle import Node

from lifecycle_msgs.msg import State

from iii_drone_supervision.managed_node_client import ManagedNodeClient

#########################################################################
# Class:
#########################################################################

class Supervisor:
    """
        Class for supervising the execution of the III-Drone system.
    """
    
    def __init__(
        self,
        supervision_config_file: str,
        monitor_node_states: bool,
        node: Node
    ):
        """
            Constructor of the Supervisor class.
        """
        
        self._supervisor_config_file = supervision_config_file
        
        with open(self._supervisor_config_file, 'r') as f:
            self._supervision_config = yaml.safe_load(f)

        self.node = node
        
        self._monitor_node_states = monitor_node_states
            
        self.validate_supervision_config(self._supervision_config)
        
        self._managed_nodes_dict: dict = self._supervision_config['managed_nodes']
        self._monitor_period_ms: int = self._supervision_config['monitor_period_ms']
        self._request_state_timeout_ms: int = self._supervision_config['request_state_timeout_ms']
        self._max_threads: int = self._supervision_config['max_threads']

        self._managed_node_clients: dict[ManagedNodeClient] = {}
        
        self._managed_node_transitions: list = self._expand_managed_node_transitions(self._managed_nodes_dict)
        self._leaf_keys: list = []
        self._transition_tree: dict = {}
        self._transition_tree, self._leaf_keys, self._root_keys = self._construct_transition_tree(self._managed_node_transitions)

        self.monitor_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        message = "Initializing managed node clients..."
        
        self._log_debug(message)

        self.monitor_timer = None
        
        self._init_managed_node_clients()
        
        self._log_info("Supervisor initialized.")

    def _init_managed_node_clients(self):
        for key, node in self._managed_nodes_dict.items():
            managed_node_client = ManagedNodeClient(
                parent_node=self.node,
                monitor_period_ms=self._monitor_period_ms,
                monitor_state=self._monitor_node_states,
                request_state_timeout_ms=self._request_state_timeout_ms,
                node_name=node["node_name"],
                node_namespace=node["node_namespace"],
                monitor_callback_group=self.monitor_callback_group
            )

            self._managed_node_clients[key] = managed_node_client

        self.monitor_timer = self.node.create_timer(
            self._monitor_period_ms / 1000,
            self._monitor_managed_nodes
        )
        
    def _monitor_managed_nodes(self):
        for key, managed_node_client in self._managed_node_clients.items():
            managed_node_client: ManagedNodeClient
            managed_node_client.monitor_callback()

    def start(
        self,
        activate: bool = True,
        message_callback: Optional[callable] = None,
        select_nodes: list[str] = [],
        restart_nodes: list[dict] = [],
        ignore_dependencies: bool = False
    ) -> tuple[bool, list[dict]]:
        """
            Method for starting the supervision process.
        """

        if len(select_nodes) > 0:
            failure = False
            message = "Starting system with selected nodes" + (" and their dependencies:" if not ignore_dependencies else ":")
            for select_node in select_nodes:
                message += f"\n\t{select_node}"

                if select_node not in self._managed_nodes_dict:
                    message = f"Node {select_node} not found in managed nodes."
                    self._log_error(message, message_callback)
                    failure = True
                    
            if failure:
                return False, []
                
            self._log_info(message, message_callback)
        
        if not rclpy.ok():
            return False, []
        
        success, started_nodes = self._manage_nodes(
            'bringup',
            ("activation" if activate else "configuration"),
            message_callback=message_callback,
            select_nodes=select_nodes,
            remanage_nodes=restart_nodes,
            ignore_dependencies=ignore_dependencies
        )
        
        if not success:
            return False, started_nodes
        
        return True, started_nodes
            
    def stop(
        self,
        cleanup: bool = True,
        message_callback: Optional[callable] = None,
        select_nodes: list[str] = [],
        ignore_dependencies: bool = False
    ) -> tuple[bool, list[dict]]:
        """
            Method for stopping the supervision process.
        """
        
        if select_nodes is not None:
            message = "Stopping system with selected nodes and their dependencies:"
            
            failure = False
            for select_node in select_nodes:
                message += f"\n\t{select_node}"
                
                if select_node not in self._managed_nodes_dict:
                    message = f"Node {select_node} not found in managed nodes."
                    self._log_error(message, message_callback)
                    failure = True
                    
            if failure:
                return False, []
            
            self._log_info(message, message_callback)
        
        message = "Stopping supervision..."
        self._log_info(message, message_callback)

        stopped_nodes = []
    
        if len(self._managed_node_clients) > 0:
            success, stopped_nodes = self._manage_nodes(
                'bringdown',
                ("configuration" if cleanup else "activation"),
                message_callback=message_callback,
                select_nodes=select_nodes,
                ignore_dependencies=ignore_dependencies
            )
                
            if not success:
                return False, stopped_nodes
        
        return True, stopped_nodes

    def shutdown(
        self,
        message_callback: Optional[callable] = None,
        select_nodes: list[str] = [],
        ignore_dependencies: bool = False
    ) -> tuple[bool,list[str]]:
        """
            Method for shutting down the system.
        """
        message = "Shutting down system..."
        self._log_info(message, message_callback)

        if not self.stop(
            message_callback=message_callback,
            select_nodes=select_nodes,
            ignore_dependencies=ignore_dependencies
        ):
            return False

        if len(self._managed_node_clients) != len(self._managed_nodes_dict):
            self._init_managed_node_clients()
        
        error_nodes = []
        
        for key, managed_node_client in self._managed_node_clients.items():
            managed_node_client: ManagedNodeClient
            
            if len(select_nodes) > 0 and key not in select_nodes:
                continue
            
            message = f"Shutting down:\t{key}..."
            self._log_info(message, message_callback)

            if not managed_node_client.request_shutdown():
                error_nodes.append(key)
                
                message = f"Shutdown:\t{key} failed."
                self._log_error(message, message_callback)
                
            else:
                message = f"Shutted down:\t{key}."
                self._log_info(message, message_callback)
                    
        return len(error_nodes) == 0, error_nodes
    
    def _get_node_states(self) -> dict:
        """
            Method for getting the states of the managed nodes.
        """
        node_states = {}
        
        for key, managed_node_client in self._managed_node_clients.items():
            node_states[key] = managed_node_client.state
            
        return node_states

    def _evaluate_dependency_chain(self) -> list[str]:
        """
            Method for evaluating the dependency chain. Returns a list of tuples (key, level) for dangling nodes, which is the nodes for which the dependencies are not satisfied.
        """
        
        dangling_nodes = []
        
        node_states = self._get_node_states()
        
        for key, node in self._managed_nodes_dict.items():
            active_depend = node.get("active_depend")
            config_depend = node.get("config_depend")
            
            if active_depend:
                for depend_key, depend_state in active_depend.items():
                    if depend_state == "active":
                        if node_states[key].id == State.PRIMARY_STATE_ACTIVE and node_states[depend_key].id != State.PRIMARY_STATE_ACTIVE:
                            dangling_nodes.append((key, "active"))
                            break
                        
                    elif depend_state == "config":
                        if node_states[key].id == State.PRIMARY_STATE_ACTIVE \
                                and (node_states[depend_key].id != State.PRIMARY_STATE_ACTIVE 
                                     and node_states[depend_key].id != State.PRIMARY_STATE_INACTIVE):
                            dangling_nodes.append((key, "active"))
                            break
                        
                    else:
                        raise RuntimeError("Invalid dependency state", depend_state)
                        
            if config_depend:
                for depend_key, depend_state in config_depend.items():
                    if depend_state == "active":
                        if (node_states[key].id == State.PRIMARY_STATE_ACTIVE 
                                or node_states[key].id == State.PRIMARY_STATE_INACTIVE) \
                                and node_states[depend_key].id != State.PRIMARY_STATE_ACTIVE:
                            dangling_nodes.append((key, "config"))
                            break
                        
                    elif depend_state == "config":
                        if (node_states[key].id == State.PRIMARY_STATE_ACTIVE 
                                or node_states[key].id == State.PRIMARY_STATE_INACTIVE) \
                                and (node_states[depend_key].id != State.PRIMARY_STATE_ACTIVE 
                                     and node_states[depend_key].id != State.PRIMARY_STATE_INACTIVE):
                            dangling_nodes.append((key, "config"))
                            break
                        
                    else:
                        raise RuntimeError("Invalid dependency state", depend_state)
                    
        return dangling_nodes
        
    def _log_info(
        self,
        message: str,
        message_callback: Optional[callable] = None
    ):
        """Helper function to log info messages and call the callback if defined."""
        self.node.get_logger().info(message)
        if message_callback:
            message_callback(message)
            
    def _log_debug(
        self,
        message: str,
        message_callback: Optional[callable] = None
    ):
        """Helper function to log debug messages and call the callback if defined."""
        self.node.get_logger().debug(message)
        if message_callback:
            message_callback(message)
    
    def _log_fatal(
        self,
        message: str,
        message_callback: Optional[callable] = None
    ):
        """Helper function to log fatal messages and call the callback if defined."""
        self.node.get_logger().fatal(message)
        if message_callback:
            message_callback(message)
    
    def _log_error(
        self,
        message: str,
        message_callback: Optional[callable] = None
    ):
        """Helper function to log error messages and call the callback if defined."""
        self.node.get_logger().error(message)
        if message_callback:
            message_callback(message)
            
    def _log_warn(
        self,
        message: str,
        message_callback: Optional[callable] = None
    ):
        """Helper function to log warning messages and call the callback if defined."""
        self.node.get_logger().warn(message)
        if message_callback:
            message_callback(message)

    def _evalaute_managed_tree_nodes(
        self,
        managed_tree_nodes: list[str]
    ) -> list[dict]:
        """
            Helper function to evaluate managed tree nodes.
        """
        managed_nodes = {}
        
        for key in managed_tree_nodes:
            node_key = self._transition_tree[key]["key"]
            transition = self._transition_tree[key]["transition"]
            
            if node_key not in managed_nodes:
                managed_nodes[node_key] = {
                    "key": node_key,
                    "transition": transition,
                }
            elif transition == "active":
                managed_nodes[node_key]["transition"] = transition
                
        return list(managed_nodes.values())

    def _expand_managed_tree_nodes(
        self,
        managed_tree_nodes: list[dict],
        operation: str,
    ) -> dict:
        """
            Helper function to expand managed tree nodes.
        """
        managed_nodes = {}
        
        def add_node(key: str):
            if key not in managed_nodes:
                managed_nodes[key] = self._transition_tree[key]
        
        for node in managed_tree_nodes:
            node_key = node["key"]
            transition = node["transition"]
            
            transition_key = f"{node_key}_{transition}"
            
            if operation == 'bringup' and transition == 'active':
                config_transition_key = f"{node_key}_config"
                add_node(config_transition_key)
                
            elif operation == 'bringdown' and transition == 'config':
                active_transition_key = f"{node_key}_active"
                add_node(active_transition_key)
                
            add_node(transition_key)
                
        return managed_nodes
        
    def _add_node_to_tree_recursive(
        self,
        key: str,
        transition_tree: dict,
        operation: str,
        ignore_dependencies: bool = False
    ) -> dict:
        """
            Helper function to add a node and its dependencies to the filtered transition tree.
        """
        transition_tree_copy = transition_tree.copy()
        dependencies = []

        tree_node = self._transition_tree[key]
        
        if key not in transition_tree_copy:
            transition_tree_copy[key] = tree_node
        
            dependencies.extend(tree_node["depends_on" if operation == 'bringup' else 'depends_by'])
            
            for dependency in dependencies:
                if ignore_dependencies:
                    dep_check_name = "_".join(dependency.split("_")[:-1])
                    key_check_name = "_".join(key.split("_")[:-1])

                    if dep_check_name != key_check_name:
                        continue
                    
                transition_tree_copy = self._add_node_to_tree_recursive(
                    dependency, 
                    transition_tree_copy,
                    operation,
                    ignore_dependencies=ignore_dependencies
                )
                
        return transition_tree_copy

    def _build_transition_tree(
        self,
        operation: str,
        level: str|dict,
        select_nodes: list[str],
        ignore_dependencies: bool = False
    ) -> dict:
        transition_tree = {}
            
        if isinstance(level, str):
            if level == 'configuration' and operation == 'bringup':
                for key, tree_node in self._transition_tree.items():
                    if (tree_node["transition"] == 'config' \
                            and (len(select_nodes) == 0 or tree_node["key"] in select_nodes)):
                        transition_tree = self._add_node_to_tree_recursive(
                            key,
                            transition_tree,
                            operation,
                            ignore_dependencies=len(select_nodes) > 0 and ignore_dependencies
                        )
                            
            elif level == 'activation' and operation == 'bringdown':
                for key, tree_node in self._transition_tree.items():
                    if (tree_node["transition"] == 'active' \
                            and (len(select_nodes) == 0 or tree_node["key"] in select_nodes)):
                        transition_tree = self._add_node_to_tree_recursive(
                            key,
                            transition_tree,
                            operation,
                            ignore_dependencies=len(select_nodes) > 0 and ignore_dependencies
                        )

            else:
                for key, tree_node in self._transition_tree.items():
                    if len(select_nodes) == 0 or tree_node["key"] in select_nodes:
                        transition_tree = self._add_node_to_tree_recursive(
                            key,
                            transition_tree,
                            operation,
                            ignore_dependencies=len(select_nodes) > 0 and ignore_dependencies
                        )
                        
        elif isinstance(level, dict):
            # transition_map = {
            #     "configuration": "config",
            #     "activation": "active"
            # }
            
            for key, _level in level.items():
                if key in select_nodes:
                    for tree_key, tree_node in self._transition_tree.items():
                        if tree_node["key"] == key and tree_node["transition"] == _level:
                            transition_tree = self._add_node_to_tree_recursive(
                                tree_key,
                                transition_tree,
                                operation,
                                ignore_dependencies=len(select_nodes) > 0 and ignore_dependencies
                            )
                            
                            break
                        
                else:
                    self._log_fatal(
                        f"Node {key} not found in selected nodes. When providing a dictionary for level, the keys must be in the select_nodes list."
                    )
                    
                    raise KeyError(f"Node {key} not found in selected nodes.")
                    
        return transition_tree

    def _get_transitions_from_level(
        self,
        level: str
    ) -> dict:
        """
            Getter for the transitions at a specific level.
        """
        transitions = {}
        
        for key, tree_node in self._transition_tree.items():
            if tree_node["transition"] == level:
                transitions[key] = tree_node
                
        return transitions
    
    def _merge_transitions(
        self,
        transitions_a: dict,
        transitions_b: dict,
        operation: str
    ):
        """
            Method for merging two transition dictionaries.
        """
        merged_transitions = transitions_a.copy()
        
        for key, tree_node in transitions_b.items():
            merged_transitions = self._add_node_to_tree_recursive(
                key,
                merged_transitions,
                operation
            )
            
        return merged_transitions
    
    def _get_ready_nodes(
        self,
        operation: str,
        transition_tree: dict
    ) -> list[str]:
        """
            Method for getting the ready nodes.
        """
        ready_nodes = []
        
        for key, tree_node in transition_tree.items():
            if len(tree_node["depends_on" if operation == 'bringup' else 'depends_by']) == 0:
                ready_nodes.append(key)
                
            else:
                no_keys_in_tree = True
                for depend_key in tree_node["depends_on" if operation == 'bringup' else 'depends_by']:
                    if depend_key in transition_tree:
                        no_keys_in_tree = False
                        break
                    
                if no_keys_in_tree:
                    ready_nodes.append(key)
                
        return ready_nodes

    def _manage_nodes(
        self, 
        operation: str, 
        level: str|dict,
        message_callback: Optional[callable] = None,
        select_nodes: list[str] = [],
        remanage_nodes: list[dict] = [],
        ignore_dependencies: bool = False
    ) -> tuple[bool, list[dict]]:
        """
            General method for managing the state of the nodes.
            operation: 'bringup' or 'bringdown'
            level: 'activation' or 'configuration'
            message_callback: Optional callback for logging messages
            select_nodes: List of nodes to select for the operation
            remanage_nodes: List of nodes to remanage
        """
        # Ensure valid operation and level inputs
        assert operation in ['bringup', 'bringdown'], "Invalid operation. Must be 'bringup' or 'bringdown'."
        assert level in ['activation', 'configuration'] or isinstance(level,dict), "Invalid level. Must be 'activation' or 'configuration'."

        if isinstance(level, dict):
            for key, _level in level.items():
                if key not in select_nodes:
                    self._log_error(
                        f"Node {key} not found in selected nodes. When providing a dictionary for level, the keys must be in the select_nodes list.",
                        message_callback
                    )
                    
                    return False, []

        # Log the start of the operation
        self._log_info(
            f"{operation.capitalize()} managed nodes at {level} level...",
            message_callback
        )
        
        # Check if there are dangling nodes
        if operation == 'bringup':
            dangling_nodes = self._evaluate_dependency_chain() if not ignore_dependencies else []
            
            if len(dangling_nodes) > 0:
                message = "Dangling nodes detected. The following nodes have dependencies that are not satisfied:"
                for node in dangling_nodes:
                    message += f"\n\t{node[0]}: {node[1]}"
                    
                message += "\nBringing down dangling nodes..."
                self._log_warn(message, message_callback)
                
                success, managed_nodes = self._manage_nodes(
                    'bringdown',
                    {key: _level for key, _level in dangling_nodes},
                    message_callback=message_callback,
                    select_nodes=[node[0] for node in dangling_nodes]
                )
                
                if not success:
                    message = "Failed to bring down all dangling nodes. The following nodes were not brought down:"
                    for key, _level in dangling_nodes:
                        if key not in [node["key"] for node in managed_nodes]:
                            message += f"\n\t{key}: {_level}"
                        
                    message += "\nThe following nodes were brought down:"
                    for node in managed_nodes:
                        message += f"\n\t{node['key']}: {node['transition']}"
                        
                    self._log_error(message, message_callback)

                    return False, []

        # Remove nodes that are not relevant to the current operation
        transition_tree = self._build_transition_tree(
            operation,
            level,
            select_nodes,
            ignore_dependencies=ignore_dependencies
        )
        transition_tree = self._merge_transitions(
            transition_tree,
            self._expand_managed_tree_nodes(
                remanage_nodes, 
                operation
            ),
            operation
        )

        # Initialize lists for tracking nodes and threads
        threads = {}
        ready_tree_nodes = self._get_ready_nodes(
            operation, 
            transition_tree
        )
        started_tree_nodes = []
        managed_tree_nodes = []
        waiting_tree_nodes = list(transition_tree.keys())

        # Remove unneeded nodes from ready list
        for key in ready_tree_nodes.copy():
            if key not in transition_tree:
                ready_tree_nodes.remove(key)

        # Remove initial nodes from waiting list
        for key in ready_tree_nodes:
            waiting_tree_nodes.remove(key)

        # List to track errors
        errors = []

        # Locks for thread safety
        managed_tree_nodes_lock = Lock()
        errors_lock = Lock()

        # cnt = 0
        # cnt_lock = Lock()
        
        def manage_node(node_key: str, transition: str):
            """
                Method for managing a single node state.
            """
            # Define verbs for logging purposes
            verbs = {
                'bringup': {'config': 'Configuring', 'active': 'Activating'},
                'bringdown': {'config': 'Cleaning up', 'active': 'Deactivating'}
            }
            verb_past = {
                'bringup': {'config': 'Configured', 'active': 'Activated'},
                'bringdown': {'config': 'Cleaned up', 'active': 'Deactivated'}
            }
            
            # Log the operation for the current node
            self._log_info(
                f"{verbs[operation][transition]}:\t{node_key}...",
                message_callback
            )
            
            # Get the node client and its state
            managed_node_client: ManagedNodeClient = self._managed_node_clients[node_key]
            state = managed_node_client.state

            nonlocal managed_tree_nodes
            nonlocal errors
            
            success = False
            already_managed = False

            # Handle configuration and activation for bringup and bringdown operations
            if transition == 'config':
                if operation == 'bringup':
                    if managed_node_client.is_configured:
                        success = True
                        already_managed = True
                    elif managed_node_client.request_configure():
                        success = True
                    else:
                        with errors_lock:
                            errors.append(f"Failure on configuration of node {node_key}.")
                elif operation == 'bringdown':
                    if not managed_node_client.is_configured:
                        success = True
                        already_managed = True
                    elif managed_node_client.request_cleanup():
                        success = True
                    else:
                        with errors_lock:
                            errors.append(f"Failure on cleanup of node {node_key}.")
            elif transition == 'active':
                if operation == 'bringup':
                    if managed_node_client.is_active:
                        success = True
                        already_managed = True
                    elif managed_node_client.request_activate():
                        success = True
                    else:
                        with errors_lock:
                            errors.append(f"Failure on activation of node {node_key}.")
                elif operation == 'bringdown':
                    if not managed_node_client.is_active:
                        success = True
                        already_managed = True
                    elif managed_node_client.request_deactivate():
                        success = True
                    else:
                        with errors_lock:
                            errors.append(f"Failure on deactivation of node {node_key}.")
            else:
                with errors_lock:
                    errors.append(f"Invalid transition: {transition}")
            
            if success:
                with managed_tree_nodes_lock:
                    managed_tree_nodes.append(f"{node_key}_{transition}")

                if already_managed:
                    self._log_info(
                        f"{verb_past[operation][transition]}:\t{node_key} already {verb_past[operation][transition].lower()}.",
                        message_callback
                    )

                else:
                    self._log_info(
                        f"{verb_past[operation][transition]}:\t{node_key}.",
                        message_callback
                    )
            else:
                self._log_error(
                    f"{verbs[operation][transition]}:\t{node_key} failed.",
                    message_callback
                )
                
        failed = False
        
        while len(ready_tree_nodes) > 0 or len(waiting_tree_nodes) > 0:
            
            if failed:
                break
            
            if len(threads) < self._max_threads:
                ready_tree_nodes_copy = ready_tree_nodes.copy()
                for tree_key in ready_tree_nodes_copy:
                    thread = Thread(
                        target=manage_node,
                        args=(
                            self._transition_tree[tree_key]["key"], 
                            self._transition_tree[tree_key]["transition"]
                        )
                    )
                    
                    thread.start()
                    threads[tree_key] = thread
                    started_tree_nodes.append(tree_key)
                    ready_tree_nodes.remove(tree_key)

            while True:
                one_finished = False
                
                with errors_lock:
                    if len(errors) > 0:
                        self._log_fatal(
                            f"Errors occurred during {operation}:",
                            message_callback
                        )
                        for error in errors:
                            self._log_fatal(
                                error,
                                message_callback
                            )

                        failed = True
                        
                        break
                    
                with managed_tree_nodes_lock:
                    started_tree_nodes_copy = started_tree_nodes.copy()
                    
                    for managed_tree_key in managed_tree_nodes:
                        if managed_tree_key in started_tree_nodes_copy:
                            one_finished = True
                            started_tree_nodes.remove(managed_tree_key)
                            threads[managed_tree_key].join()
                            threads.pop(managed_tree_key)
                            
                if one_finished:
                    waiting_tree_nodes_copy = waiting_tree_nodes.copy()
                    
                    for waiting_tree_key in waiting_tree_nodes_copy:
                        dependencies = self._transition_tree[waiting_tree_key]["depends_on"] if operation == 'bringup' else self._transition_tree[waiting_tree_key]["depends_by"]
                        dependencies_cp = dependencies.copy()
                        
                        for dep_key in dependencies_cp:
                            if dep_key not in transition_tree:
                                dependencies.remove(dep_key)
                        
                        if all(dependency in managed_tree_nodes for dependency in dependencies):
                            ready_tree_nodes.append(waiting_tree_key)
                            waiting_tree_nodes.remove(waiting_tree_key)
                            
                    if len(started_tree_nodes) == 0 and len(ready_tree_nodes) == 0 and len(waiting_tree_nodes) > 0:
                        self._log_fatal(
                            f"Circular dependency detected. Unmanaged nodes: {waiting_tree_nodes}.",
                            message_callback
                        )
                        failed = True
                        break
                    
                    break
                
                sleep(0.1)

        for thread in threads.values():
            thread.join()
            
        managed_nodes = self._evalaute_managed_tree_nodes(managed_tree_nodes)

        if failed:
            message = f"Failed to {operation} system. Failure on the following nodes:"
            for tree_node in waiting_tree_nodes:
                message += f"\n\t{self._transition_tree[tree_node]['key']}_{self._transition_tree[tree_node]['transition']}"
            message += "\nSuccessfully managed nodes:"
            for managed_node in managed_nodes:
                message += f"\n\t{managed_node['key']}: {managed_node['transition']}"
            self._log_fatal(
                message,
                message_callback
            )
            
            return False, managed_nodes
                            
        final_verbs = {
            'bringup': 'brought up',
            'bringdown': 'brought down'
        }
        
        self._log_info(
            f"Nodes successfully {final_verbs[operation]} to level {level}.",
            message_callback
        )

        return True, managed_nodes

    @staticmethod
    def validate_supervision_config(supervision_config: dict):
        """
            Method for validating the supervision configuration.
        """
        
        assert 'monitor_period_ms' in supervision_config, 'Key "monitor_period_ms" not found in supervision configuration.'
        assert isinstance(supervision_config['monitor_period_ms'], int), 'Key "monitor_period_ms" must be an integer.'
        assert 'request_state_timeout_ms' in supervision_config, 'Key "request_state_timeout_ms" not found in supervision configuration.'
        assert isinstance(supervision_config['request_state_timeout_ms'], int), 'Key "request_state_timeout_ms" must be an integer.'
        assert 'max_threads' in supervision_config, 'Key "max_threads" not found in supervision configuration.'
        assert isinstance(supervision_config['max_threads'], int), 'Key "max_threads" must be an integer.'
        assert 'managed_nodes' in supervision_config, 'Key "managed_nodes" not found in supervision configuration.'
        
        managed_nodes: dict = supervision_config['managed_nodes']
        
        assert managed_nodes is not None and len(managed_nodes) > 0, 'Key "managed_nodes" must contain at least one node.'

        managed_node_keys_list = list(managed_nodes.keys())
        
        for key, node in managed_nodes.items():
            assert 'node_name' in node, 'Key "node_name" not found in managed node configuration.'
            assert isinstance(node['node_name'], str), 'Key "node_name" must be a string.'
            assert 'node_namespace' in node, 'Key "namespace" not found in managed node configuration.'
            assert isinstance(node['node_namespace'], str), 'Key "node_namespace" must be a string.'
            
            if "active_depend" in node:
                for active_depend_key, active_depend_state in node["active_depend"].items():
                    assert str(active_depend_key) in managed_node_keys_list, f'Key "{active_depend_key}" not found in managed node configuration.'
                    assert active_depend_key != key, 'Node cannot depend on itself.'
                    assert str(active_depend_state) in ["active", "config"], 'Dependency must be either "active" or "config".'
                    
            if "config_depend" in node:
                for config_depend_key, config_depend_state in node["config_depend"].items():
                    assert str(config_depend_key) in managed_node_keys_list, f'Key "{config_depend_key}" not found in managed node configuration.'
                    assert config_depend_key != key, 'Node cannot depend on itself.'
                    assert str(config_depend_state) in ["active", "config"], 'Dependency must be either "active" or "config".'

    def _expand_managed_node_transitions(
        self, 
        managed_nodes: dict
    ) -> list:
        """
            Method for expanding the managed nodes into a list of transitions.
        """
        
        def create_entry(key, transition, depends_on):
            """
                Helper function for creating an entry in the managed node transitions list.
            """
            entry = {
                "key": key,
                "transition": transition,
                "depends_on": []
            }
            
            if depends_on:
                for depend_key, depend_transition in depends_on.items():
                    entry["depends_on"].append({
                        "key": depend_key,
                        "transition": depend_transition
                    })

            if transition == "active":
                entry["depends_on"].append({
                    "key": key,
                    "transition": "config"
                })
            
            return entry

        config_transitions = []
        active_transitions = []

        for key, node in managed_nodes.items():
            config_entry = create_entry(key, "config", node.get("config_depend"))
            active_entry = create_entry(key, "active", node.get("active_depend"))
            
            config_transitions.append(config_entry)
            active_transitions.append(active_entry)
            
        managed_node_transitions = config_transitions + active_transitions

        return managed_node_transitions
    
    def _construct_transition_tree(
        self,
        managed_node_transitions: list
    ) -> list:
        """
            Method for creating the transition tree.
        """
        
        managed_node_transitions_copy = deepcopy(managed_node_transitions)
        keys = [entry["key"] for entry in managed_node_transitions_copy]
        
        transition_tree = {}
        leaf_keys = []
        
        for entry in managed_node_transitions:
            key = entry["key"]
            transition = entry["transition"]
            depends_on = entry["depends_on"]
            
            tree_key = f"{key}_{transition}"
            
            if tree_key in transition_tree:
                raise Exception(f"Duplicate key found in managed node transitions: {key}")
            
            tree_entry = {
                "key": key,
                "transition": transition,
                "depends_on": [],
                "depends_by": []
            }
            
            for depend in depends_on:
                depend_key = depend["key"]
                depend_transition = depend["transition"]
                
                depend_tree_key = f"{depend_key}_{depend_transition}"
                
                if depend_key not in keys:
                    raise Exception(f"Dependency not found in managed node transitions: {depend_key}")
                
                tree_entry["depends_on"].append(depend_tree_key)
                
                if depend_tree_key in transition_tree:
                    transition_tree[depend_tree_key]["depends_by"].append(tree_key)
                    
            transition_tree[tree_key] = tree_entry

            if len(tree_entry["depends_on"]) == 0:
                leaf_keys.append(tree_key)
                    
            for other_tree_key in transition_tree:
                if tree_key in transition_tree[other_tree_key]["depends_on"]:
                    tree_entry["depends_by"].append(other_tree_key)

        root_keys = []
        
        for tree_key in transition_tree:
            if len(transition_tree[tree_key]["depends_by"]) == 0:
                root_keys.append(tree_key)
        
        return transition_tree, leaf_keys, root_keys
        
    @property
    def managed_nodes(self) -> list[str]:
        """
            Getter for the managed nodes.
        """
        return list(self._managed_nodes_dict.keys())
                
            
            
                
    
        
                

    
            
        
        
