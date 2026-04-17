# III-Drone-Supervision

`iii_drone_supervision` is the workspace bringup and runtime-management package for the III system.
It owns the canonical system-management path:

- the authoritative launch-time system graph
- the daemon-managed service graph
- the lifecycle dependency graph used for managed start/stop/restart operations
- the background system-manager daemon used by `iii system ...`
- the derived tmux session/view model used for operator visibility

## Documentation Map

- [Architecture](docs/architecture.md)
- [Operator Flows](docs/operator-flows.md)

## Package Role

This package is responsible for five related concerns:

1. `Launch topology`
   The canonical process/node set for each runtime profile is declared in [`iii_drone_supervision/system_spec.py`](iii_drone_supervision/system_spec.py).

2. `Daemon-managed services`
   Non-lifecycle services such as `micro_ros_agent` are declared in the system specification and owned directly by the daemon.

3. `Lifecycle orchestration`
   Dependency-aware configure/activate/deactivate/cleanup behavior is handled by the supervision logic in [`iii_drone_supervision/supervisor.py`](iii_drone_supervision/supervisor.py).

4. `Runtime management`
   [`iii_drone_supervision/system_manager.py`](iii_drone_supervision/system_manager.py) owns `LaunchService`, process-state tracking, lifecycle operations, and status aggregation.

5. `Daemon + operator interface`
   [`iii_drone_supervision/system_daemon.py`](iii_drone_supervision/system_daemon.py) exposes the system manager over a Unix socket. The III CLI consumes that API and materializes a tmux session from [`iii_drone_supervision/tmux_spec.py`](iii_drone_supervision/tmux_spec.py).

## Runtime Model

Operational flow:

1. Source a workspace profile from `setup/`, typically `setup/setup_dev.bash`.
2. Start the runtime through the CLI:
   - `iii system boot`
3. The CLI ensures the background system daemon is running.
4. The daemon loads the requested profile, instantiates the canonical launch graph through the ROS 2 launch API, creates the supervision model for managed nodes, and prepares daemon-managed services.
5. The CLI derives a tmux session from the tmux view specification and opens panes that show status/log streams rather than per-node startup commands.
6. Use:
   - `iii system start`
   - `iii system service start <service_id>`
   - `iii system service stop <service_id>`
   - `iii system service restart <service_id>`
   - `iii system stop`
   - `iii system restart`
   - `iii system status`
   - `iii system logs <entity_id>`

Direct launch without the daemon is supported through:

```bash
ros2 launch iii_drone_supervision system.launch.py profile:=sim
```

That path launches the canonical process graph, but it does not provide the daemon’s socket API, unified status view, or tmux integration.

## Module Map

### Canonical System Description

- `system_spec.py`
  Defines `SystemEntitySpec`, `SystemServiceSpec`, `ManagedNodeSpec`, and `SystemProfileSpec`. This is the source of truth for:
  - which entities exist
  - which daemon-managed services exist
  - which profiles include them
  - launch factories
  - service commands and readiness checks
  - lifecycle metadata and dependency edges
  - per-entity and per-service log directories

- `launch/system.launch.py`
  Direct ROS 2 launch entrypoint for the canonical system graph.

### Runtime Management

- `system_manager.py`
  Long-lived runtime controller that:
  - owns a `LaunchService`
  - builds the active launch description from the chosen profile
  - owns daemon-managed services such as `micro_ros_agent`
  - tracks process start/exit state
  - instantiates `Supervisor`
  - exposes boot/start/stop/restart/shutdown/status operations

- `service_manager.py`
  Process-control and readiness-monitoring support for daemon-owned services. Services are regular processes with restart, log, status, and readiness support; they are not ROS lifecycle nodes.

- `system_daemon.py`
  Background daemon that exposes the system manager over a Unix socket with a small JSON request/response protocol.

### Lifecycle Supervision

- `supervisor.py`
  Core lifecycle/dependency engine used by the system manager.

- `managed_node_client.py`
  Client abstraction for interacting with lifecycle-managed nodes.

### Managed External Process Support

- `managed_node_wrapper.py`
  Wraps arbitrary processes or nested launch fragments behind lifecycle semantics.

- `managed_process.py`
  Low-level process execution and monitoring support.

- `process_management_configuration.py`
  Loads and validates per-process management configuration.

- `node_management_config/*.yaml`
  Command/monitor definitions for wrapped processes such as TF bringup and simulation sensors.

### Operator View Definition

- `tmux_spec.py`
  Defines the tmux view model. This is intentionally separate from the runtime graph so operator layout can evolve without becoming part of the launch/lifecycle schema.

## Configuration Model

There are two relevant configuration layers in this package.

### 1. Canonical system specification

`system_spec.py` is the main source of runtime truth. For each entity it can define:

- stable `entity_id`
- launch factory
- optional lifecycle-managed node identity
- respawn policy
- profile membership

For each daemon-managed service it can define:

- stable `service_id`
- command factory
- working directory
- restart policy
- readiness topic checks
- profile membership

Profiles currently supported:

- `sim`
- `real`
- `opti_track`

Profile-specific differences are encoded as conditional entities and dependency overrides inside the canonical system specification instead of separate top-level launch descriptions.

### 2. Process-wrapper configuration

The package also carries:

- `node_management_config/*.yaml`

These files define wrapped external processes and nested launch fragments used by the canonical system specification.

## Process, Lifecycle, And Tmux Boundaries

Responsibility split:

- `system_spec.py`
  Owns runtime topology, daemon-managed service metadata, and lifecycle metadata.

- `tmux_spec.py`
  Owns operator presentation only.

- `system_manager.py`
  Owns process runtime, daemon-managed services, lifecycle orchestration, status, and log-directory bookkeeping.

- `tools/III-Drone-CLI`
  Owns the terminal UI surface and tmux session creation.

This separation is intentional:

- tmux should not be responsible for launching individual nodes
- launch should remain the authoritative runtime graph
- daemon-managed services should be controlled by the daemon, not by tmux or simulation helper scripts
- lifecycle transitions should remain under supervision logic
- the CLI should talk to the daemon rather than directly to ROS services for basic system-management flows

## Logs And Visibility

Each launched entity and daemon-managed service gets its own log directory under the resolved runtime/log base path. The CLI uses daemon-reported log directories for:

- `iii system logs <entity_id>`
- tmux panes that follow per-entity logs

Each pane observes daemon-owned runtime logs instead of spawning a node directly. Service logs use the same `current.log` and `process.log` convention as launched entities.

## Development Notes

- Keep new process topology in `system_spec.py`, not in tmux definitions or handwritten shell launch scripts.
- Keep tmux presentation changes in `tmux_spec.py`.
- If you add a new managed entity, update:
  - system profile definitions
  - lifecycle dependencies if applicable
  - tmux presentation if it should appear in the default operator layout
  - tests for profile resolution and daemon routing
- If you add a daemon-managed service, update:
  - service definitions in `system_spec.py`
  - service dependencies for lifecycle nodes that require it
  - tmux presentation if operators should watch it by default
  - daemon/CLI tests for service routing
- Prefer direct tests of the system specification and daemon API over large integration-only assertions.

## Tests

Package-level tests cover runtime-management behavior, including:

- supervision graph logic
- canonical profile resolution
- launch-group generation
- tmux-spec consistency
- daemon command routing

Typical package-only command:

```bash
python3 -m pytest src/III-Drone-Supervision/test -q
```
