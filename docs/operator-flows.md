# Supervision Operator Flows

## Normal Development Flow

Typical development flow inside the devcontainer:

```bash
source setup/setup_dev.bash
iii system boot
iii system attach
iii system start
```

What happens internally:

1. `iii system boot`
   Starts `iii-system-daemon.service` through systemd if needed and asks it to boot the selected profile.

2. The daemon
   Loads the canonical system profile, starts the launch runtime, prepares daemon-managed services, and prepares the supervision model.

3. The CLI
   Requests the tmux session description and creates a tmux session whose panes are mostly log and status views.

4. `iii system start`
   Starts required daemon-managed services, then tells the daemon-owned system manager to configure and optionally activate managed nodes in dependency order.

PX4 is treated as an external availability source. In the devcontainer, Gazebo/PX4 SITL provides that source. On the real drone, the physical flight controller provides that source. `iii system start` starts the `micro_ros_agent` service and leaves PX4-dependent lifecycle nodes inactive until the FMU topics are available.

## Status And Logs

The most useful runtime inspection commands are:

```bash
iii system status
iii system status --watch
iii system list-nodes
iii system service list
iii system logs <entity_id>
iii system logs <entity_id> --follow
```

`status` reports both:

- process state
- service state and readiness
- managed lifecycle state

The CLI reports both process state and lifecycle state through one command surface.

## Aircraft Services And Clock Gate

Aircraft provisioning (`iii host provision`) installs `iii-system-daemon.service`
and `iii-runtime-api.service`, grouped by `iii.target`. Both run from the
editable workspace: they source `/opt/ros/jazzy/setup.bash` and
`/home/iii/ws/install/setup.bash`, read `/etc/iii/runtime.env` (profile, ROS
domain, daemon socket `/run/iii/system_manager.sock`), and are reinstalled and
restarted by `iii deploy dev --restart`. Starting them does not boot the ROS
graph: boot and start it with `iii system boot` and `iii system start` on the
Pi, or with **Start aircraft system** in the GUI.

On aircraft profiles the runtime API refuses runtime lifecycle commands unless
live PX4 state shows the aircraft disarmed and landed, and it refuses arming and
mission activation until chrony reports a settled clock (offset at most 0.1 s).

Service logs use the same command:

```bash
iii system logs micro_ros_agent
iii system logs micro_ros_agent --follow
```

## Service Control

Daemon-managed services are regular processes owned by the system daemon. They are not ROS lifecycle nodes.

```bash
iii system service start micro_ros_agent
iii system service stop micro_ros_agent
iii system service restart micro_ros_agent
```

`micro_ros_agent` may be alive but not ready. That means the agent process is running, but the PX4 FMU topics used as readiness checks are absent or stale. This is valid while PX4 SITL or the physical flight controller is unavailable.

In `opti_track`, `opti_track_pose_relay` is ready only while it feeds PX4 fresh motion-capture poses on `/fmu/in/vehicle_visual_odometry`, which it signals with its 2 Hz `/opti_track/pose_relay/fresh` heartbeat. While Motive does not stream the configured rigid body, `mission_executor` and `custom_operation` stay inactive. With `/opti_track/pose_relay/rigid_body_id` unset (`-1`) the relay stays alive but never becomes ready (its health reports ERROR); set the id in the `opti_track` parameter set and restart the service.

## Restart Semantics

The user-facing entrypoint is:

```bash
iii system restart --select-nodes <entity_id>
```

Operationally this is a lifecycle-oriented restart handled by the daemon through the supervision engine. If a process has died entirely, the daemon-managed launch runtime is the source of truth for process existence and the status view will show that mismatch.

The important distinction is:

- `process state`
  Is the launched entity alive?

- `lifecycle state`
  Is the managed node configured or active?

- `service state`
  Is the daemon-owned service process alive and ready?

The daemon aggregates both so the CLI can expose a single operator-facing control surface.

## Shutdown

To stop the managed runtime:

```bash
iii system shutdown
```

Runtime stop/shutdown is a daemon command. It leaves the independently supervised
runtime API process online for status and recovery. Receiver-owned activation or
rollback may stop the complete `iii.target`, including both application units,
because selector mutation has a stronger all-units-stopped safety boundary.

To also close the tmux session:

```bash
iii system shutdown --dry-run --operation-id shutdown-aircraft
iii system shutdown --operation-id shutdown-aircraft --resume --confirm
iii system kill-session --dry-run --operation-id remove-local-session
iii system kill-session --operation-id remove-local-session --resume --confirm
```

## Direct Launch For Debugging

When you want the canonical graph without daemon/tmux management:

```bash
ros2 launch iii_drone_supervision system.launch.py profile:=sim
```

This is useful for targeted debugging, integration testing, and validating that the system specification remains directly launchable outside the CLI flow.
