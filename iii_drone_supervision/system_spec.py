"""Canonical III system specification and launch-description helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node


LaunchFactory = Callable[[str], Node]


@dataclass(frozen=True)
class ManagedNodeSpec:
    """Lifecycle metadata for a managed entity."""

    node_name: str
    node_namespace: str
    config_depend: dict[str, str] = field(default_factory=dict)
    active_depend: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class SystemEntitySpec:
    """Single launchable entity in the system graph."""

    entity_id: str
    launch_factory: LaunchFactory
    managed_node: ManagedNodeSpec | None = None
    respawn: bool = True
    profiles: tuple[str, ...] = ("sim", "real", "opti_track")


@dataclass(frozen=True)
class SystemProfileSpec:
    """Resolved system profile."""

    name: str
    entities: tuple[SystemEntitySpec, ...]
    monitor_period_ms: int = 1000
    request_state_timeout_ms: int = 10000
    max_threads: int = 10

    def managed_entities(self) -> tuple[SystemEntitySpec, ...]:
        return tuple(entity for entity in self.entities if entity.managed_node is not None)

    def entity_map(self) -> dict[str, SystemEntitySpec]:
        return {entity.entity_id: entity for entity in self.entities}

    def build_supervision_config(self) -> dict:
        managed_nodes = {}
        for entity in self.managed_entities():
            managed = entity.managed_node
            assert managed is not None
            node_entry = {
                "node_name": managed.node_name,
                "node_namespace": managed.node_namespace,
            }
            if managed.config_depend:
                node_entry["config_depend"] = dict(managed.config_depend)
            if managed.active_depend:
                node_entry["active_depend"] = dict(managed.active_depend)
            managed_nodes[entity.entity_id] = node_entry
        return {
            "monitor_period_ms": self.monitor_period_ms,
            "request_state_timeout_ms": self.request_state_timeout_ms,
            "max_threads": self.max_threads,
            "managed_nodes": managed_nodes,
        }


def _workspace_root_from_env() -> Path | None:
    workspace_dir = os.environ.get("WORKSPACE_DIR")
    if workspace_dir:
        return Path(workspace_dir)
    return None


def resolve_runtime_dir() -> Path:
    runtime_dir = os.environ.get("III_SYSTEM_RUNTIME_DIR")
    if runtime_dir:
        return Path(runtime_dir).expanduser()
    workspace_root = _workspace_root_from_env()
    if workspace_root is not None:
        return workspace_root / "runtime"
    return Path.home() / ".cache" / "iii_drone" / "runtime"


def resolve_log_base_dir(profile_name: str) -> Path:
    base = os.environ.get("ROS_LOG_DIR_BASE")
    if base:
        return Path(base).expanduser() / profile_name
    return resolve_runtime_dir() / "logs" / profile_name


def entity_log_dir(profile_name: str, entity_id: str) -> Path:
    return resolve_log_base_dir(profile_name) / entity_id


def resolve_ros_params_file(profile_name: str) -> str:
    use_sim = profile_name == "sim"
    filename = "ros_params_sim.yaml" if use_sim else "ros_params_real.yaml"

    config_base_dir = os.environ.get("CONFIG_BASE_DIR")
    if config_base_dir:
        configured = Path(config_base_dir).expanduser() / "iii_drone" / filename
        if configured.exists():
            return str(configured)

    workspace_root = _workspace_root_from_env()
    if workspace_root is not None:
        source_copy = workspace_root / "src" / "III-Drone-Configuration" / "config" / filename
        if source_copy.exists():
            return str(source_copy)

    package_share = Path(get_package_share_directory("iii_drone_configuration"))
    return str(package_share / "config" / filename)


def resolve_node_management_config(filename: str) -> str:
    config_dir = os.environ.get("NODE_MANAGEMENT_CONFIG_DIR")
    if config_dir:
        configured = Path(config_dir).expanduser() / filename
        if configured.exists():
            return str(configured)

    workspace_root = _workspace_root_from_env()
    if workspace_root is not None:
        source_copy = workspace_root / "src" / "III-Drone-Supervision" / "node_management_config" / filename
        if source_copy.exists():
            return str(source_copy)

    package_share = Path(get_package_share_directory("iii_drone_supervision"))
    return str(package_share / "node_management_config" / filename)


def _node_entity(
    entity_id: str,
    *,
    package: str,
    executable: str,
    namespace: str | None = None,
    name: str | None = None,
    arguments: Iterable[str] = (),
    ros_arguments: Iterable[str] = (),
    managed_node: ManagedNodeSpec | None = None,
    profiles: tuple[str, ...] = ("sim", "real", "opti_track"),
    respawn: bool = True,
) -> SystemEntitySpec:
    def factory(profile_name: str) -> Node:
        return Node(
            package=package,
            executable=executable,
            namespace=namespace,
            name=name,
            arguments=list(arguments),
            ros_arguments=list(ros_arguments),
            parameters=[resolve_ros_params_file(profile_name)],
            output="log",
            respawn=respawn,
            respawn_delay=2.0,
        )

    return SystemEntitySpec(
        entity_id=entity_id,
        launch_factory=factory,
        managed_node=managed_node,
        respawn=respawn,
        profiles=profiles,
    )


def _managed_wrapper_entity(
    entity_id: str,
    *,
    config_file: str,
    managed_node: ManagedNodeSpec,
    profiles: tuple[str, ...],
    respawn: bool = True,
) -> SystemEntitySpec:
    return _node_entity(
        entity_id,
        package="iii_drone_supervision",
        executable="managed_node_wrapper",
        arguments=(resolve_node_management_config(config_file),),
        managed_node=managed_node,
        profiles=profiles,
        respawn=respawn,
    )


_COMMON_ENTITIES: tuple[SystemEntitySpec, ...] = (
    _node_entity(
        "configuration_server",
        package="iii_drone_configuration",
        executable="configuration_server",
        namespace="/configuration/configuration_server",
        name="configuration_server",
        managed_node=ManagedNodeSpec(
            node_name="configuration_server",
            node_namespace="/configuration/configuration_server",
        ),
    ),
    _node_entity(
        "charger_gripper",
        package="iii_drone_core",
        executable="charger_gripper_node.py",
        namespace="/payload/charger_gripper",
        name="charger_gripper",
        managed_node=ManagedNodeSpec(
            node_name="charger_gripper",
            node_namespace="/payload/charger_gripper",
        ),
    ),
    _node_entity(
        "hough_transformer",
        package="iii_drone_core",
        executable="hough_transformer",
        namespace="/perception/hough_transformer",
        name="hough_transformer",
        managed_node=ManagedNodeSpec(
            node_name="hough_transformer",
            node_namespace="/perception/hough_transformer",
            active_depend={"tf": "active"},
        ),
    ),
    _node_entity(
        "pl_dir_computer",
        package="iii_drone_core",
        executable="pl_dir_computer",
        namespace="/perception/pl_dir_computer",
        name="pl_dir_computer",
        managed_node=ManagedNodeSpec(
            node_name="pl_dir_computer",
            node_namespace="/perception/pl_dir_computer",
            active_depend={"hough_transformer": "active", "tf": "active"},
        ),
    ),
    _node_entity(
        "pl_mapper",
        package="iii_drone_core",
        executable="pl_mapper",
        namespace="/perception/pl_mapper",
        name="pl_mapper",
        managed_node=ManagedNodeSpec(
            node_name="pl_mapper",
            node_namespace="/perception/pl_mapper",
            config_depend={"pl_dir_computer": "config"},
            active_depend={"pl_dir_computer": "active", "tf": "active"},
        ),
    ),
    _node_entity(
        "trajectory_generator",
        package="iii_drone_core",
        executable="trajectory_generator",
        namespace="/control/trajectory_generator",
        name="trajectory_generator",
        managed_node=ManagedNodeSpec(
            node_name="trajectory_generator",
            node_namespace="/control/trajectory_generator",
        ),
    ),
    _node_entity(
        "maneuver_controller",
        package="iii_drone_core",
        executable="maneuver_controller",
        namespace="/control/maneuver_controller",
        name="maneuver_controller",
        managed_node=ManagedNodeSpec(
            node_name="maneuver_controller",
            node_namespace="/control/maneuver_controller",
            active_depend={
                "trajectory_generator": "active",
                "pl_mapper": "active",
                "tf": "active",
            },
        ),
    ),
    _node_entity(
        "powerline_overview_provider",
        package="iii_drone_mission",
        executable="powerline_overview_provider",
        namespace="/mission/powerline_overview_provider",
        name="powerline_overview_provider",
        managed_node=ManagedNodeSpec(
            node_name="powerline_overview_provider",
            node_namespace="/mission/powerline_overview_provider",
            active_depend={"pl_mapper": "active", "tf": "active"},
        ),
    ),
    _node_entity(
        "mission_executor",
        package="iii_drone_mission",
        executable="mission_executor",
        namespace="/mission/mission_executor",
        name="mission_executor",
        managed_node=ManagedNodeSpec(
            node_name="mission_executor",
            node_namespace="/mission/mission_executor",
            config_depend={
                "maneuver_controller": "active",
                "pl_mapper": "active",
                "charger_gripper": "active",
                "powerline_overview_provider": "active",
            },
        ),
    ),
)


_PROFILE_ENTITIES: dict[str, tuple[SystemEntitySpec, ...]] = {
    "sim": (
        _node_entity(
            "micro_ros_agent",
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            arguments=("udp4", "--port", "8888"),
            profiles=("sim",),
            managed_node=None,
        ),
        _managed_wrapper_entity(
            "tf",
            config_file="tf_sim_launch.yaml",
            managed_node=ManagedNodeSpec(
                node_name="tf_sim_launch_manager",
                node_namespace="/managed_nodes",
            ),
            profiles=("sim",),
        ),
        _managed_wrapper_entity(
            "sensors",
            config_file="sensors_sim_launch.yaml",
            managed_node=ManagedNodeSpec(
                node_name="sensors_sim_launch_manager",
                node_namespace="/managed_nodes",
            ),
            profiles=("sim",),
        ),
    ),
    "real": (
        _managed_wrapper_entity(
            "tf",
            config_file="tf_real_launch.yaml",
            managed_node=ManagedNodeSpec(
                node_name="tf_real_launch_manager",
                node_namespace="/managed_nodes",
            ),
            profiles=("real", "opti_track"),
        ),
        _managed_wrapper_entity(
            "cable_camera",
            config_file="cable_camera.yaml",
            managed_node=ManagedNodeSpec(
                node_name="cable_camera_manager",
                node_namespace="/managed_nodes",
            ),
            profiles=("real", "opti_track"),
        ),
        _node_entity(
            "mmwave",
            package="iwr6843aop_pub",
            executable="pcl_pub",
            namespace="/sensor/mmwave",
            name="mmwave",
            managed_node=ManagedNodeSpec(
                node_name="mmwave",
                node_namespace="/sensor/mmwave",
            ),
            profiles=("real", "opti_track"),
        ),
    ),
}


def get_system_profile(profile_name: str) -> SystemProfileSpec:
    normalized = profile_name.strip().lower()
    if normalized not in {"sim", "real", "opti_track"}:
        raise ValueError(f"Unknown system profile: {profile_name}")

    entities = list(_COMMON_ENTITIES)
    if normalized == "sim":
        entities.extend(_PROFILE_ENTITIES["sim"])
        entity_overrides = {
            "hough_transformer": {"active_depend": {"sensors": "active", "tf": "active"}},
            "pl_dir_computer": {"active_depend": {"hough_transformer": "active", "tf": "active", "sensors": "active"}},
            "pl_mapper": {"active_depend": {"pl_dir_computer": "active", "tf": "active", "sensors": "active"}},
        }
    else:
        entities.extend(_PROFILE_ENTITIES["real"])
        entity_overrides = {
            "hough_transformer": {"active_depend": {"cable_camera": "active", "tf": "active"}},
            "pl_dir_computer": {"active_depend": {"hough_transformer": "active", "tf": "active"}},
            "pl_mapper": {"active_depend": {"pl_dir_computer": "active", "tf": "active", "mmwave": "active"}},
        }

    adjusted_entities = []
    for entity in entities:
        if entity.entity_id in entity_overrides and entity.managed_node is not None:
            override = entity_overrides[entity.entity_id]
            managed = entity.managed_node
            adjusted_entities.append(
                SystemEntitySpec(
                    entity_id=entity.entity_id,
                    launch_factory=entity.launch_factory,
                    respawn=entity.respawn,
                    profiles=entity.profiles,
                    managed_node=ManagedNodeSpec(
                        node_name=managed.node_name,
                        node_namespace=managed.node_namespace,
                        config_depend=dict(managed.config_depend),
                        active_depend=override["active_depend"],
                    ),
                )
            )
        else:
            adjusted_entities.append(entity)

    return SystemProfileSpec(name=normalized, entities=tuple(adjusted_entities))


def build_system_launch_description(profile_name: str) -> LaunchDescription:
    profile = get_system_profile(profile_name)
    launch_entities = []
    for entity in profile.entities:
        launch_entities.append(build_entity_launch_group(profile.name, entity))
    return LaunchDescription(launch_entities)


def build_entity_launch_group(profile_name: str, entity: SystemEntitySpec) -> GroupAction:
    log_dir = entity_log_dir(profile_name, entity.entity_id)
    log_dir.mkdir(parents=True, exist_ok=True)
    action = entity.launch_factory(profile_name)
    return GroupAction(
        [
            SetEnvironmentVariable("ROS_LOG_DIR", str(log_dir)),
            action,
        ]
    )
