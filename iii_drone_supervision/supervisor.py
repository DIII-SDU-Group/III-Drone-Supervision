#########################################################################
# Imports:
#########################################################################

import yaml
from copy import deepcopy
from typing import Optional
from threading import Thread, Lock
from time import sleep

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
        node: Node
    ):
        """
            Constructor of the Supervisor class.
        """
        
        self._supervisor_config_file = supervision_config_file
        
        with open(self._supervisor_config_file, 'r') as f:
            self._supervision_config = yaml.safe_load(f)

        self.node = node
            
        self.validate_supervision_config(self._supervision_config)
        
        self._managed_nodes_dict: dict = self._supervision_config['managed_nodes']
        self._monitor_period_ms: int = self._supervision_config['monitor_period_ms']

        self._managed_node_clients: dict[ManagedNodeClient] = {}
        
        self._managed_node_transitions: list = self._expand_managed_node_transitions(self._managed_nodes_dict)
        self._leaf_keys: list = []
        self._transition_tree: dict = {}
        self._transition_tree, self._leaf_keys, self._root_keys = self._construct_transition_tree(self._managed_node_transitions)

    def _init_managed_node_clients(self):
        for key, node in self._managed_nodes_dict.items():
            managed_node_client = ManagedNodeClient(
                parent_node=self.node,
                monitor_period_ms=self._monitor_period_ms,
                node_name=node["node_name"],
                node_namespace=node["node_namespace"]
            )

            self._managed_node_clients[key] = managed_node_client

    def start(
        self,
        activate: bool = True
    ) -> bool:
        """
            Method for starting the supervision process.
        """
        self.node.get_logger().info("Initializing managed node clients...")

        if len(self._managed_node_clients) != len(self._managed_nodes_dict):
            self._init_managed_node_clients()
        
        if not self._manage_nodes(
            'bringup',
            ("activation" if activate else "configuration")
        ):
            return False
        
        return True
            
    def stop(
        self,
        cleanup: bool = True
    ) -> bool:
        """
            Method for stopping the supervision process.
        """
        self.node.get_logger().info("Stopping supervision...")

        if len(self._managed_node_clients) > 0:
            if not self._manage_nodes(
                'bringdown',
                ("configuration" if cleanup else "activation")
            ):
                return False
        
        return True

    def shutdown(self) -> tuple[bool,list[str]]:
        """
            Method for shutting down the system.
        """
        self.node.get_logger().info("Shutting down system...")

        if not self.stop():
            return False

        if len(self._managed_node_clients) != len(self._managed_nodes_dict):
            self._init_managed_node_clients()
        
        error_nodes = []
        
        for key, managed_node_client in self._managed_node_clients.items():
            managed_node_client: ManagedNodeClient
            self.node.get_logger().info(f"Shutting down:\t{key}")

            if not managed_node_client.request_shutdown():
                error_nodes.append(key)
                
                self.node.get_logger().error(f"Shutdown:\t{key} failed")
                
            else:
                self.node.get_logger().info(f"Shutted down:\t{key}")

        return len(error_nodes) == 0, error_nodes

    def _manage_nodes(
        self, 
        operation: str, 
        level: str
    ) -> bool:
        """
            General method for managing the state of the nodes.
            operation: 'bringup' or 'bringdown'
            level: 'activation' or 'configuration'
        """
        # Ensure valid operation and level inputs
        assert operation in ['bringup', 'bringdown'], "Invalid operation. Must be 'bringup' or 'bringdown'."
        assert level in ['activation', 'configuration'], "Invalid level. Must be 'activation' or 'configuration'."
        
        # Log the start of the operation
        self.node.get_logger().info(f"{operation.capitalize()} managed nodes at {level} level...")

        # Remove nodes that are not relevant to the current operation
        filtered_transition_tree = {}
        
        if level == 'configuration' and operation == 'bringup':
            for key, tree_node in self._transition_tree.items():
                dependencies = []
                
                if tree_node["transition"] == 'config':
                    if key not in filtered_transition_tree:
                        filtered_transition_tree[key] = tree_node
                    
                        dependencies.extend(tree_node["depends_on"])
                    
                while len(dependencies) > 0:
                    dependencies_copy = dependencies.copy()
                    
                    for dependency in dependencies_copy:
                        dependecy_tree_node = self._transition_tree[dependency]
                        
                        if dependency not in filtered_transition_tree:
                            filtered_transition_tree[dependency] = dependecy_tree_node
                        
                            dependencies.extend(dependecy_tree_node["depends_on"])
                            
                        dependencies.remove(dependency)
                        
        elif level == 'activation' and operation == 'bringdown':
            for key, tree_node in self._transition_tree.items():
                dependencies = []
                
                if tree_node["transition"] == 'active':
                    if key not in filtered_transition_tree:
                        filtered_transition_tree[key] = tree_node
                    
                        dependencies.extend(tree_node["depends_by"])
                    
                while len(dependencies) > 0:
                    dependencies_copy = dependencies.copy()
                    
                    for dependency in dependencies_copy:
                        dependecy_tree_node = self._transition_tree[dependency]
                        
                        if dependency not in filtered_transition_tree:
                            filtered_transition_tree[dependency] = dependecy_tree_node
                        
                            dependencies.extend(dependecy_tree_node["depends_by"])
                            
                        dependencies.remove(dependency)

        else:
            filtered_transition_tree = self._transition_tree.copy()

        # Initialize lists for tracking nodes and threads
        threads = []
        ready_tree_nodes = (self._leaf_keys.copy() if operation == 'bringup' else self._root_keys.copy())
        started_tree_nodes = []
        managed_tree_nodes = []
        waiting_tree_nodes = list(filtered_transition_tree.keys())

        # Remove unneeded nodes from ready list
        for key in ready_tree_nodes.copy():
            if key not in filtered_transition_tree:
                ready_tree_nodes.remove(key)

        # Remove initial nodes from waiting list
        for key in ready_tree_nodes:
            waiting_tree_nodes.remove(key)
        
        # List to track errors
        errors = []

        # Locks for thread safety
        managed_tree_nodes_lock = Lock()
        errors_lock = Lock()
        
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
            self.node.get_logger().info(f"{verbs[operation][transition]}:\t{node_key}...")
            
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
                    self.node.get_logger().info(f"{verb_past[operation][transition]}:\t{node_key} already {verb_past[operation][transition].lower()}.")

                else:
                    self.node.get_logger().info(f"{verb_past[operation][transition]}:\t{node_key}.")
            else:
                self.node.get_logger().info(f"{verbs[operation][transition]}:\t{node_key} failed.")

        failed = False
        
        while len(ready_tree_nodes) > 0:
            if failed:
                break
            
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
                threads.append(thread)
                started_tree_nodes.append(tree_key)
                ready_tree_nodes.remove(tree_key)

            while True:
                one_finished = False
                
                with errors_lock:
                    if len(errors) > 0:
                        self.node.get_logger().fatal(f"Errors occurred during {operation}:")
                        for error in errors:
                            self.node.get_logger().fatal(error)

                        failed = True
                        
                        break
                    
                with managed_tree_nodes_lock:
                    started_tree_nodes_copy = started_tree_nodes.copy()
                    
                    for managed_tree_key in managed_tree_nodes:
                        if managed_tree_key in started_tree_nodes_copy:
                            one_finished = True
                            started_tree_nodes.remove(managed_tree_key)
                            
                if one_finished:
                    waiting_tree_nodes_copy = waiting_tree_nodes.copy()
                    
                    for waiting_tree_key in waiting_tree_nodes_copy:
                        dependencies = self._transition_tree[waiting_tree_key]["depends_on"] if operation == 'bringup' else self._transition_tree[waiting_tree_key]["depends_by"]
                        if all(dependency in managed_tree_nodes for dependency in dependencies):
                            ready_tree_nodes.append(waiting_tree_key)
                            waiting_tree_nodes.remove(waiting_tree_key)
                            
                    if len(started_tree_nodes) == 0 and len(ready_tree_nodes) == 0 and len(waiting_tree_nodes) > 0:
                        self.node.get_logger().fatal(f"Circular dependency detected. Unmanaged nodes: {waiting_tree_nodes}.")
                        failed = True
                        break
                    
                    break
                
                sleep(0.1)

        for thread in threads:
            thread.join()

        if failed:
            self.node.get_logger().fatal(f"Failed to {operation} system.")
            return False
                            
        final_verbs = {
            'bringup': 'brought up',
            'bringdown': 'brought down'
        }
        
        self.node.get_logger().info(f"System {final_verbs[operation]} successfully to level {level}.")

        return True


    @staticmethod
    def validate_supervision_config(supervision_config: dict):
        """
            Method for validating the supervision configuration.
        """
        
        assert 'monitor_period_ms' in supervision_config, 'Key "monitor_period_ms" not found in supervision configuration.'
        assert isinstance(supervision_config['monitor_period_ms'], int), 'Key "monitor_period_ms" must be an integer.'
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
        
                
            
            
                
    
        
                

    
            
        
        