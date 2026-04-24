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
   Ensures the background system daemon is running and asks it to boot the selected profile.

2. The daemon
   Loads the canonical system profile, starts the launch runtime, and prepares the supervision model.

3. The CLI
   Requests the tmux session description and creates a tmux session whose panes are mostly log and status views.

4. `iii system start`
   Tells the daemon-owned system manager to configure and optionally activate managed nodes in dependency order.

## Status And Logs

The most useful runtime inspection commands are:

```bash
iii system status
iii system status --watch
iii system list-nodes
iii system logs <entity_id>
iii system logs <entity_id> --follow
```

`status` reports both:

- process state
- managed lifecycle state

This is the key improvement over the older model where the operator had to reason separately about tmux panes and supervisor state.

## Restart Semantics

The preferred user-facing entrypoint is:

```bash
iii system restart --select-nodes <entity_id>
```

Operationally this is a lifecycle-oriented restart handled by the daemon through the supervision engine. If a process has died entirely, the daemon-managed launch runtime is the source of truth for process existence and the status view will show that mismatch.

The important distinction is:

- `process state`
  Is the launched entity alive?

- `lifecycle state`
  Is the managed node configured or active?

The daemon aggregates both so the CLI can expose a single operator-facing control surface.

## Shutdown

To stop the managed runtime:

```bash
iii system shutdown
```

To also close the tmux session:

```bash
iii system shutdown --kill-session
```

## Direct Launch For Debugging

When you want the canonical graph without daemon/tmux management:

```bash
ros2 launch iii_drone_supervision system.launch.py profile:=sim
```

This is useful for targeted debugging, integration testing, and validating that the system specification remains directly launchable outside the CLI flow.
