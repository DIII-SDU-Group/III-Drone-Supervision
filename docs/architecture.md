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
5. `iii-runtime-api`
   Runs on the runtime host beside the daemon and exposes the network-facing
   GUI v2/remote CLI control plane. It uses the daemon Unix socket for runtime
   control and ROS/MAVLink adapters for operator state and commands.

The daemon owns the launch runtime and service runtime. The CLI materializes tmux from the tmux session specification.
The runtime API does not replace the daemon; it is an authenticated network
facade over daemon, ROS, MAVLink/MAVSDK, logs, configuration, rosbag, and map
state surfaces.

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

GUI v2 and remote runtime-control CLI commands reach this managed path through
`iii-runtime-api` rather than by exposing the daemon socket or forwarding shell
commands over SSH.

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
- `hil`
- `opti_track`

Profiles vary by:

- included entities
- daemon-managed services
- wrapped launch/process fragments
- dependency overrides (lifecycle and service dependencies)
- parameter-file selection

`get_system_profile` rejects a profile whose lifecycle dependencies name a node
the profile does not run, or whose service dependencies name a service it does
not own.

### OptiTrack reduced graph

`opti_track` is the "flight basics" profile for the SDU OptiTrack lab. The lab
has no cable, so the profile runs only the control, mission, and runtime
stack:

- `configuration_server`, `tf` (the `tf_real_launch.yaml` wrapper; world->drone
  comes from PX4 odometry), `trajectory_generator`, `maneuver_controller`,
  `rosbag_recorder`, `mission_executor`, and `custom_operation`
- services `micro_ros_agent` and `opti_track_pose_relay`

Payload (`charger_gripper`), sensor (`cable_camera`, `mmwave`), perception
(`hough_transformer`, `pl_dir_computer`, `pl_mapper`), and corridor overview
(`powerline_overview_provider`, `pylon_overview_provider`) entities never
start, whether or not the payload is mounted. Its dependency overrides drop the
edges to them: `maneuver_controller` activates after `trajectory_generator` and
`tf`; `mission_executor` configures after `maneuver_controller` and
`rosbag_recorder`. `real` keeps its full graph.

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

Lifecycle nodes can declare service dependencies in `ManagedNodeSpec.service_depend`. For example, `mission_executor` requires `micro_ros_agent: ready`, so it remains inactive when PX4 is absent and can be started after the bridge becomes ready. A profile override may replace a node's service dependencies.

### `opti_track_pose_relay`

In `opti_track` only, the daemon runs the motion-capture pose relay:

```bash
ros2 run iii_drone_core opti_track_pose_relay --ros-args --params-file <profile parameter file>
```

A service inherits the daemon's environment rather than an entity's launch
environment, so the command carries the profile's active parameter file,
resolved exactly as for the entities. The relay reads the boot-only
`/opti_track/pose_relay/*` parameters from it, subscribes to the lab gateway's
`/body_splitter/body_<id>/pose` on the lab ROS domain, and publishes
`/fmu/in/vehicle_visual_odometry` on the stack's domain.

The service is ready while `/fmu/in/vehicle_visual_odometry`
(`px4_msgs/msg/VehicleOdometry`) flows: seen within 5 s, stable for 2 s, and
with an advancing `timestamp` field (a repeated one reads as stale). The ready
timeout is 120 s. In `opti_track`, `mission_executor` requires
`opti_track_pose_relay: ready` beside `micro_ros_agent: ready`, so the executor
starts only once PX4 is fed motion-capture poses. `custom_operation` activates
through `mission_executor` and carries the same requirement; otherwise starting
it would pull the executor in ungated, and a relay restart would leave it
active on a stopped executor.

The relay and the agent need no start order: the relay publishes without the
agent, and the agent picks the topic up whenever it (re)starts. Services
therefore declare no dependencies on each other.

Most shared runtime structure remains in common definitions, which reduces drift between simulated and hardware deployments.

## Wrapped Processes

The package includes:

- `node_management_config/*.yaml`
- `managed_node_wrapper.py`

These files define and run external processes or nested launch fragments that are represented as managed entities inside the system graph.

Daemon-managed services are declared in `system_spec.py` instead of `node_management_config/*.yaml`. Process wrappers remain for nested launch fragments that should present lifecycle semantics to the supervisor.
