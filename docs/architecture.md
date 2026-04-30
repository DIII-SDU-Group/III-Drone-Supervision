# Supervision Architecture

## Overview

`iii_drone_supervision` contains the launch-driven system-manager path used by the workspace.

Architecture:

1. `system_spec.py`
   Declares the canonical III runtime graph, daemon-managed services, and profile-conditioned differences.
2. `system_manager.py`
   Owns ROS 2 launch runtime, daemon-managed services, and lifecycle orchestration.
3. `system_daemon.py`
   Exposes the manager as a background daemon over a Unix socket.
4. `tools/III-Drone-CLI`
   Talks to the daemon and builds the tmux session from the tmux view specification.

The daemon owns the launch runtime and service runtime. The CLI materializes tmux from the tmux session specification.

## Main Building Blocks

### `system_spec.py`

This is the canonical runtime graph.

It defines:

- `ManagedNodeSpec`
  Lifecycle identity plus `config_depend`, `active_depend`, and `service_depend` edges.

- `SystemEntitySpec`
  A single launchable runtime entity with a stable `entity_id`, launch factory, profile membership, and respawn policy.

- `SystemServiceSpec`
  A daemon-owned process service with a stable `service_id`, command factory, restart policy, and readiness checks.

- `SystemProfileSpec`
  A resolved runtime profile containing services, entities, supervision settings, and helper methods for generating the supervisor configuration consumed by the runtime manager.

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
- owning daemon-managed services such as `micro_ros_agent`
- monitoring service readiness through ROS topic heartbeats
- tracking per-entity process state through launch process event handlers
- instantiating the existing `Supervisor` with profile-derived lifecycle metadata
- exposing boot/start/stop/restart/shutdown/status/tmux/log-dir/service operations

Conceptually it merges two planes:

- `process plane`
  Process existence, launch, exit tracking, and log directories.

- `service plane`
  Daemon-owned non-lifecycle processes, restart behavior, logs, and readiness checks.

- `lifecycle plane`
  Configure/activate/deactivate/cleanup ordering and dependency-aware operations.

### `system_daemon.py`

The daemon is a thin transport wrapper around `SystemManager`.

It provides:

- a long-lived background process owned by systemd
- Unix-socket request/response handling
- JSON commands such as `boot`, `start`, `status`, `restart`, `service_start`, `service_stop`, `service_restart`, and `log_dir`

The daemon is intentionally not exposed as a ROS service/action API. This reduces the CLI’s dependency on ROS transport details and keeps the same systemd-owned control surface for native onboard deployment and the devcontainer.

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
7. Daemon-managed services needed by selected lifecycle nodes are started first. Nodes blocked by unavailable external resources remain inactive and are reported in status/start output.

### Unmanaged path

Useful for debugging and direct ROS workflows:

```bash
ros2 launch iii_drone_supervision system.launch.py profile:=sim
```

This uses the same canonical launch graph, but without daemon-managed services, service readiness gating, tmux integration, or the daemon control surface.

## Profiles

Profiles are resolved inside the canonical system specification rather than by maintaining separate top-level launch descriptions.

Profiles:

- `sim`
- `real`
- `opti_track`

Profiles vary by:

- included entities
- daemon-managed services
- wrapped launch/process fragments
- dependency overrides
- parameter-file selection

## Services And External Availability

Daemon-managed services are runtime processes that are part of the III system but are not lifecycle nodes. `micro_ros_agent` is the first service in this scope.

`micro_ros_agent` bridges the external PX4 flight controller into ROS 2. In simulation, PX4 SITL/Gazebo is the external flight-controller availability source. On the real drone, the physical PX4 flight controller is the external availability source. The service can be alive while PX4 is unavailable; readiness becomes true only when the configured FMU heartbeat topics are being received.

The service control commands are:

```bash
iii system service list
iii system service start micro_ros_agent
iii system service stop micro_ros_agent
iii system service restart micro_ros_agent
```

Lifecycle nodes can declare service dependencies in `ManagedNodeSpec.service_depend`. For example, `mission_executor` requires `micro_ros_agent: ready`, so it remains inactive when PX4 is absent and can be started after the bridge becomes ready.

Most shared runtime structure remains in common definitions, which reduces drift between simulated and hardware deployments.

## Wrapped Processes

The package includes:

- `node_management_config/*.yaml`
- `managed_node_wrapper.py`

These files define and run external processes or nested launch fragments that are represented as managed entities inside the system graph.

Daemon-managed services are declared in `system_spec.py` instead of `node_management_config/*.yaml`. Process wrappers remain for nested launch fragments that should present lifecycle semantics to the supervisor.
