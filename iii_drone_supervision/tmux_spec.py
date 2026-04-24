"""Derived tmux session descriptions for III system profiles."""

from __future__ import annotations

from dataclasses import dataclass

from .system_spec import get_system_profile


@dataclass(frozen=True)
class TmuxPaneSpec:
    title: str
    mode: str
    target: str | None = None
    command: str | None = None


@dataclass(frozen=True)
class TmuxWindowSpec:
    name: str
    layout: str
    panes: tuple[TmuxPaneSpec, ...]


@dataclass(frozen=True)
class TmuxSessionSpec:
    session_name: str
    windows: tuple[TmuxWindowSpec, ...]
    startup_window: str = "system"


def get_tmux_session_spec(profile_name: str) -> TmuxSessionSpec:
    profile = get_system_profile(profile_name)
    entity_ids = set(profile.entity_map())

    windows = [
        TmuxWindowSpec(
            name="system",
            layout="even-horizontal",
            panes=(
                TmuxPaneSpec(title="status", mode="status"),
                TmuxPaneSpec(title="shell", mode="shell", command="bash"),
            ),
        ),
    ]

    def logs_window(name: str, layout: str, *targets: str):
        panes = tuple(
            TmuxPaneSpec(title=target, mode="logs", target=target)
            for target in targets
            if target in entity_ids
        )
        if panes:
            windows.append(TmuxWindowSpec(name=name, layout=layout, panes=panes))

    logs_window("background", "even-horizontal", "micro_ros_agent", "tf")
    logs_window("configuration", "even-horizontal", "configuration_server")
    logs_window("payload", "even-horizontal", "charger_gripper")
    logs_window("sensors", "even-horizontal", "sensors", "cable_camera", "mmwave")
    logs_window("perception", "tiled", "hough_transformer", "pl_dir_computer", "pl_mapper")
    logs_window("control", "even-horizontal", "trajectory_generator", "maneuver_controller")
    logs_window("mission", "even-horizontal", "mission_executor", "powerline_overview_provider")

    return TmuxSessionSpec(
        session_name=f"iii_{profile.name}",
        windows=tuple(windows),
    )
