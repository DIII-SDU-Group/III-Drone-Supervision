# III-Drone-Supervision

`iii_drone_supervision` is the workspace bringup and runtime-management package for the III system.
It now owns the canonical system-management path:

- the authoritative launch-time system graph
- the lifecycle dependency graph used for managed start/stop/restart operations
- the background system-manager daemon used by `iii system ...`
- the derived tmux session/view model used for operator visibility

The package still contains the legacy supervisor ROS node and managed-process wrappers, but the preferred operational path is now the daemon-managed launch runtime.

## Documentation Map

- [Architecture](docs/architecture.md)
- [Operator Flows](docs/operator-flows.md)

## Package Role

This package is responsible for four related concerns:

1. `Launch topology`
   The canonical process/node set for each runtime profile is declared in [`iii_drone_supervision/system_spec.py`](iii_drone_supervision/system_spec.py).

2. `Lifecycle orchestration`
   Dependency-aware configure/activate/deactivate/cleanup behavior is still handled by the supervision logic in [`iii_drone_supervision/supervisor.py`](iii_drone_supervision/supervisor.py).

3. `Runtime management`
   [`iii_drone_supervision/system_manager.py`](iii_drone_supervision/system_manager.py) owns `LaunchService`, process-state tracking, lifecycle operations, and status aggregation.

4. `Daemon + operator interface`
   [`iii_drone_supervision/system_daemon.py`](iii_drone_supervision/system_daemon.py) exposes the system manager over a Unix socket. The III CLI consumes that API and materializes a tmux session from [`iii_drone_supervision/tmux_spec.py`](iii_drone_supervision/tmux_spec.py).

## Preferred Runtime Model

The preferred operational flow is:

1. Source a workspace profile from `setup/`, typically `setup/setup_dev.bash`.
2. Start the runtime through the CLI:
   - `iii system boot`
3. The CLI ensures the background system daemon is running.
4. The daemon loads the requested profile, instantiates the canonical launch graph through the ROS 2 launch API, and creates the supervision model for managed nodes.
5. The CLI derives a tmux session from the tmux view specification and opens panes that show status/log streams rather than per-node startup commands.
6. Use:
   - `iii system start`
   - `iii system stop`
   - `iii system restart`
   - `iii system status`
   - `iii system logs <entity_id>`

Direct launch without the daemon is still supported through:

```bash
ros2 launch iii_drone_supervision system.launch.py profile:=sim
```

That path launches the canonical process graph, but it does not provide the daemon’s socket API, unified status view, or tmux integration.

## Module Map

### Canonical System Description

- `system_spec.py`
  Defines `SystemEntitySpec`, `ManagedNodeSpec`, and `SystemProfileSpec`. This is the source of truth for:
  - which entities exist
  - which profiles include them
  - launch factories
  - lifecycle metadata and dependency edges
  - per-entity log directories

- `launch/system.launch.py`
  Direct ROS 2 launch entrypoint for the canonical system graph.

### Runtime Management

- `system_manager.py`
  Long-lived runtime controller that:
  - owns a `LaunchService`
  - builds the active launch description from the chosen profile
  - tracks process start/exit state
  - instantiates `Supervisor`
  - exposes boot/start/stop/restart/shutdown/status operations

- `system_daemon.py`
  Background daemon that exposes the system manager over a Unix socket with a small JSON request/response protocol.

### Lifecycle Supervision

- `supervisor.py`
  Core lifecycle/dependency engine used by the system manager.

- `supervisor_node.py`
  Legacy ROS-node wrapper around the supervision engine. It remains available, but it is no longer the preferred integration path for `iii system`.

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

There are now two relevant configuration layers in this package.

### 1. Canonical system specification

`system_spec.py` is the main source of runtime truth. For each entity it can define:

- stable `entity_id`
- launch factory
- optional lifecycle-managed node identity
- respawn policy
- profile membership

Profiles currently supported:

- `sim`
- `real`
- `opti_track`

Profile-specific differences are encoded as conditional entities and dependency overrides inside the canonical system specification instead of separate top-level launch descriptions.

### 2. Legacy/compatibility process configuration

The package still carries:

- `supervision_config/*.yaml`
- `node_management_config/*.yaml`

These remain relevant where the legacy supervisor path or managed wrappers are still used. They are no longer the preferred top-level source of truth for process topology.

## Process, Lifecycle, And Tmux Boundaries

The current intended responsibility split is:

- `system_spec.py`
  Owns runtime topology and lifecycle metadata.

- `tmux_spec.py`
  Owns operator presentation only.

- `system_manager.py`
  Owns process runtime, lifecycle orchestration, status, and log-directory bookkeeping.

- `tools/III-Drone-CLI`
  Owns the terminal UI surface and tmux session creation.

This separation is intentional:

- tmux should not be responsible for launching individual nodes
- launch should remain the authoritative runtime graph
- lifecycle transitions should remain under supervision logic
- the CLI should talk to the daemon rather than directly to ROS services for basic system-management flows

## Logs And Visibility

Each launched entity gets its own ROS log directory under the resolved runtime/log base path. The CLI uses daemon-reported log directories for:

- `iii system logs <entity_id>`
- tmux panes that follow per-entity logs

This replaces the older “one startup command per tmux pane” model with “one canonical launch runtime plus multiple observation panes”.

## Development Notes

- Keep new process topology in `system_spec.py`, not in tmux definitions or handwritten shell launch scripts.
- Keep tmux presentation changes in `tmux_spec.py`.
- If you add a new managed entity, update:
  - system profile definitions
  - lifecycle dependencies if applicable
  - tmux presentation if it should appear in the default operator layout
  - tests for profile resolution and daemon routing
- Prefer direct tests of the system specification and daemon API over large integration-only assertions.

## Tests

Current package-level tests cover both legacy and new runtime-management behavior, including:

- supervision graph logic
- canonical profile resolution
- launch-group generation
- tmux-spec consistency
- daemon command routing

Typical package-only command:

```bash
python3 -m pytest src/III-Drone-Supervision/test -q
```
