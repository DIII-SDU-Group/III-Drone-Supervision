# III-Drone-Supervision

`iii_drone_supervision` manages lifecycle bringup and bringdown across the III system. It is responsible for validating the supervision graph, instantiating managed-node clients, and coordinating startup, shutdown, restart, and monitoring behavior.

## Package Role

This package owns:

- supervision-graph validation and dependency handling
- lifecycle transitions for managed III nodes
- wrapping external processes as ROS lifecycle-managed nodes
- configuration parsing for managed processes

## Module Map

### Core Supervision

- `supervisor.py`: main supervision engine, transition-tree construction, dependency checks, and node-management orchestration
- `supervisor_node.py`: ROS node exposing the supervision engine via services and actions

### Managed Node and Process Integration

- `managed_node_client.py`: client abstraction for interacting with lifecycle-managed nodes
- `managed_node_wrapper.py`: wrapper node that exposes non-lifecycle processes through lifecycle semantics
- `managed_process.py`: process-launching and monitoring support
- `process_management_configuration.py`: YAML configuration loader/validator for managed processes

## Supervision Model

The supervision layer works in terms of:

- managed nodes identified by logical keys
- two transition levels: `config` and `active`
- dependency edges for configuration and activation separately

This lets the package reason about partial system bringup, selected-node operations, restart flows, and dangling dependency cleanup.

## Configuration

The package expects a supervision YAML file with:

- `monitor_period_ms`
- `request_state_timeout_ms`
- `max_threads`
- `managed_nodes`

Each managed node entry declares the ROS node identity plus optional `config_depend` and `active_depend` graphs. Process-managed nodes additionally use per-node process management configuration files parsed by `process_management_configuration.py`.

## Tests

The current tests cover:

- process-management configuration parsing and validation
- log-level and timeout handling
- invalid configuration detection
- transition-tree expansion and dependency-edge construction
- dangling dependency detection
- ready-node and filtered-tree behavior

Typical package-only commands:

```bash
python3 -m pytest src/III-Drone-Supervision/test -q
```

## Extension Guidelines

- keep graph semantics centralized in `supervisor.py`
- add tests for every new configuration key or dependency rule
- document any new supervision action/service behavior in this README
