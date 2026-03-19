# Supervision Architecture

## Overview

`iii_drone_supervision` now contains both the legacy supervision path and the newer launch-driven system-manager path.

The preferred architecture is:

1. `system_spec.py`
   Declares the canonical III runtime graph and profile-conditioned differences.
2. `system_manager.py`
   Owns ROS 2 launch runtime and lifecycle orchestration.
3. `system_daemon.py`
   Exposes the manager as a background daemon over a Unix socket.
4. `tools/III-Drone-CLI`
   Talks to the daemon and builds the tmux session from the tmux view specification.

This replaces the older pattern where tmux panes were responsible for spawning individual node commands and the supervisor only entered later to transition them through lifecycle states.

## Main Building Blocks

### `system_spec.py`

This is the canonical runtime graph.

It defines:

- `ManagedNodeSpec`
  Lifecycle identity plus `config_depend` and `active_depend` edges.

- `SystemEntitySpec`
  A single launchable runtime entity with a stable `entity_id`, launch factory, profile membership, and respawn policy.

- `SystemProfileSpec`
  A resolved runtime profile containing entities plus supervision settings and helper methods for generating the legacy supervisor config structure.

The important design constraint is that `entity_id` is stable across:

- daemon status reporting
- CLI addressing
- tmux panes
- profile selection

### `system_manager.py`

`SystemManager` is the operational core.

It is responsible for:

- ensuring ROS 2 client runtime is initialized
- creating and owning a `LaunchService`
- generating the active `LaunchDescription` from `system_spec.py`
- tracking per-entity process state through launch process event handlers
- instantiating the existing `Supervisor` with profile-derived lifecycle metadata
- exposing boot/start/stop/restart/shutdown/status/tmux/log-dir operations

Conceptually it merges two planes:

- `process plane`
  Process existence, launch, exit tracking, and log directories.

- `lifecycle plane`
  Configure/activate/deactivate/cleanup ordering and dependency-aware operations.

### `system_daemon.py`

The daemon is a thin transport wrapper around `SystemManager`.

It provides:

- a long-lived background process
- Unix-socket request/response handling
- JSON commands such as `boot`, `start`, `status`, `restart`, and `log_dir`

The daemon is intentionally not exposed as a ROS service/action API. This reduces the CLI’s dependency on ROS transport details and works better when runtime ROS processes live inside a container.

### `tmux_spec.py`

The tmux model is separate from the runtime graph on purpose.

It describes:

- tmux session name
- windows
- pane titles
- pane modes such as `status`, `logs`, or `shell`

It references canonical entity IDs from the system specification but does not redefine launch behavior.

This separation prevents tmux from becoming a second source of truth for the running system.

## Runtime Paths

### Managed path

This is the normal operator/developer flow:

1. `iii system boot`
2. CLI ensures the daemon is running.
3. Daemon boots the chosen profile through ROS 2 launch.
4. CLI requests the tmux session spec.
5. CLI spawns tmux panes that show:
   - `iii system status --watch`
   - `iii system logs <entity_id> --follow`
   - an operator shell
6. `iii system start` activates managed nodes in dependency order.

### Unmanaged path

Useful for debugging and direct ROS workflows:

```bash
ros2 launch iii_drone_supervision system.launch.py profile:=sim
```

This uses the same canonical process graph, but without the daemon-managed control surface.

## Profiles

Profiles are resolved inside the canonical system specification rather than by maintaining separate top-level launch descriptions.

Current profiles:

- `sim`
- `real`
- `opti_track`

Profiles vary by:

- included entities
- wrapped launch/process fragments
- dependency overrides
- parameter-file selection

Most shared runtime structure remains in common definitions, which reduces drift between simulated and hardware deployments.

## Legacy Components

The package still contains:

- `supervisor_node.py`
- `supervision_config/*.yaml`
- `node_management_config/*.yaml`
- `managed_node_wrapper.py`

These remain important for compatibility and for wrapping external processes that are not lifecycle-native. They are no longer the preferred place to define the full system topology.
