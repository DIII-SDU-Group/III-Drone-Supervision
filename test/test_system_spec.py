from pathlib import Path

from launch import LaunchContext
from launch.actions import GroupAction, SetEnvironmentVariable
from launch.utilities import perform_substitutions
import pytest

import iii_drone_supervision.system_spec as system_spec_module
from iii_drone_supervision.supervisor import Supervisor
from iii_drone_supervision.system_spec import (
    ManagedNodeSpec,
    SystemEntitySpec,
    SystemProfileSpec,
    build_entity_launch_group,
    build_system_launch_description,
    entity_log_dir,
    get_system_profile,
)
from iii_drone_supervision.tmux_spec import get_tmux_session_spec


_PROFILES = ("sim", "real", "opti_track", "hil")

# The cable profiles' graphs as they were before opti_track got its own reduced
# graph; they must not change with it.
_CABLE_PROFILE_GRAPHS = {
    "sim": {
        "entities": [
            "configuration_server",
            "hough_transformer",
            "pl_dir_computer",
            "pl_mapper",
            "trajectory_generator",
            "maneuver_controller",
            "powerline_overview_provider",
            "pylon_overview_provider",
            "rosbag_recorder",
            "mission_executor",
            "tf",
            "sim_assets",
            "charger_gripper",
            "custom_operation",
        ],
        "lifecycle_edges": {
            "custom_operation": {"active_depend": {"mission_executor": "active"}},
            "hough_transformer": {"active_depend": {"sim_assets": "active", "tf": "active"}},
            "maneuver_controller": {
                "config_depend": {"trajectory_generator": "active"},
                "active_depend": {"pl_mapper": "active", "tf": "active", "trajectory_generator": "active"},
            },
            "mission_executor": {
                "config_depend": {
                    "charger_gripper": "active",
                    "maneuver_controller": "active",
                    "pl_mapper": "active",
                    "powerline_overview_provider": "active",
                    "pylon_overview_provider": "active",
                    "rosbag_recorder": "active",
                }
            },
            "pl_dir_computer": {
                "active_depend": {"hough_transformer": "active", "sim_assets": "active", "tf": "active"}
            },
            "pl_mapper": {
                "config_depend": {"pl_dir_computer": "config"},
                "active_depend": {"pl_dir_computer": "active", "sim_assets": "active", "tf": "active"},
            },
            "powerline_overview_provider": {"active_depend": {"pl_mapper": "active", "tf": "active"}},
        },
    },
    "real": {
        "entities": [
            "configuration_server",
            "charger_gripper",
            "hough_transformer",
            "pl_dir_computer",
            "pl_mapper",
            "trajectory_generator",
            "maneuver_controller",
            "powerline_overview_provider",
            "pylon_overview_provider",
            "rosbag_recorder",
            "mission_executor",
            "tf",
            "cable_camera",
            "mmwave",
            "custom_operation",
        ],
        "lifecycle_edges": {
            "custom_operation": {"active_depend": {"mission_executor": "active"}},
            "hough_transformer": {"active_depend": {"cable_camera": "active", "tf": "active"}},
            "maneuver_controller": {
                "config_depend": {"trajectory_generator": "active"},
                "active_depend": {"pl_mapper": "active", "tf": "active", "trajectory_generator": "active"},
            },
            "mission_executor": {
                "config_depend": {
                    "charger_gripper": "active",
                    "maneuver_controller": "active",
                    "pl_mapper": "active",
                    "powerline_overview_provider": "active",
                    "pylon_overview_provider": "active",
                    "rosbag_recorder": "active",
                }
            },
            "pl_dir_computer": {"active_depend": {"hough_transformer": "active", "tf": "active"}},
            "pl_mapper": {
                "config_depend": {"pl_dir_computer": "config"},
                "active_depend": {"mmwave": "active", "pl_dir_computer": "active", "tf": "active"},
            },
            "powerline_overview_provider": {"active_depend": {"pl_mapper": "active", "tf": "active"}},
        },
    },
    "hil": {
        "entities": [
            "configuration_server",
            "hough_transformer",
            "pl_dir_computer",
            "pl_mapper",
            "trajectory_generator",
            "maneuver_controller",
            "powerline_overview_provider",
            "pylon_overview_provider",
            "rosbag_recorder",
            "mission_executor",
            "tf",
            "custom_operation",
        ],
        "lifecycle_edges": {
            "custom_operation": {"active_depend": {"mission_executor": "active"}},
            "hough_transformer": {"active_depend": {"tf": "active"}},
            "maneuver_controller": {
                "config_depend": {"trajectory_generator": "active"},
                "active_depend": {"pl_mapper": "active", "tf": "active", "trajectory_generator": "active"},
            },
            "mission_executor": {
                "config_depend": {
                    "maneuver_controller": "active",
                    "pl_mapper": "active",
                    "powerline_overview_provider": "active",
                    "pylon_overview_provider": "active",
                    "rosbag_recorder": "active",
                }
            },
            "pl_dir_computer": {"active_depend": {"hough_transformer": "active", "tf": "active"}},
            "pl_mapper": {
                "config_depend": {"pl_dir_computer": "config"},
                "active_depend": {"pl_dir_computer": "active", "tf": "active"},
            },
            "powerline_overview_provider": {"active_depend": {"pl_mapper": "active", "tf": "active"}},
        },
    },
}


def _lifecycle_edges(profile) -> dict:
    return {
        node_id: {
            key: entry[key]
            for key in ("config_depend", "active_depend")
            if key in entry
        }
        for node_id, entry in profile.build_supervision_config()["managed_nodes"].items()
        if "config_depend" in entry or "active_depend" in entry
    }


def test_sim_profile_contains_expected_entities_and_dependencies():
    profile = get_system_profile("sim")
    entity_ids = set(profile.entity_map())

    assert {
        "configuration_server",
        "sim_assets",
        "tf",
        "pl_mapper",
        "mission_executor",
        "custom_operation",
        "pylon_overview_provider",
    } <= entity_ids
    assert all("sim" in entity.profiles for entity in profile.entities)
    assert [entity.entity_id for entity in profile.entities].count("charger_gripper") == 1
    assert "micro_ros_agent" not in entity_ids
    assert "micro_ros_agent" in profile.service_map()

    supervision_config = profile.build_supervision_config()

    assert "configuration_server" in supervision_config["managed_nodes"]
    assert (
        supervision_config["managed_nodes"]["hough_transformer"]["active_depend"]
        == {"sim_assets": "active", "tf": "active"}
    )
    assert "configuration_server" not in supervision_config["managed_nodes"]["trajectory_generator"].get(
        "config_depend", {}
    )
    assert (
        supervision_config["managed_nodes"]["maneuver_controller"]["config_depend"]
        == {"trajectory_generator": "active"}
    )
    assert profile.service_dependencies()["mission_executor"] == {"micro_ros_agent": "ready"}
    assert profile.service_dependencies()["custom_operation"] == {"micro_ros_agent": "ready"}
    assert (
        supervision_config["managed_nodes"]["mission_executor"]["config_depend"]["pylon_overview_provider"]
        == "active"
    )
    assert set(profile.service_dependencies()["mission_executor"]) <= set(profile.service_map())
    assert set(profile.service_dependencies()["custom_operation"]) <= set(profile.service_map())
    assert (
        supervision_config["managed_nodes"]["custom_operation"]["node_name"]
        == "custom_operation_manager"
    )
    assert (
        supervision_config["managed_nodes"]["custom_operation"]["node_namespace"]
        == "/managed_nodes"
    )
    micro_ros_agent = profile.service_map()["micro_ros_agent"]
    assert micro_ros_agent.command("real") == "MicroXRCEAgent udp4 -p 8888"
    readiness_topics = {topic.topic: topic for topic in micro_ros_agent.readiness_topics}
    assert readiness_topics["/fmu/out/vehicle_status_v1"].stable_for_sec > 0.0
    # No high-rate readiness heartbeat: the daemon deserializes it in Python.
    assert "/fmu/out/vehicle_odometry" not in readiness_topics
    # Mode registration is the authoritative message-contract gate.  The
    # service readiness layer must not compete for the same XRCE request topic.
    assert micro_ros_agent.px4_message_format_readiness == ()


def test_real_profile_contains_hardware_entities():
    profile = get_system_profile("real")
    entity_ids = set(profile.entity_map())

    assert {"cable_camera", "mmwave", "tf"} <= entity_ids
    assert all("real" in entity.profiles for entity in profile.entities)
    assert [entity.entity_id for entity in profile.entities].count("charger_gripper") == 1
    assert "micro_ros_agent" in profile.service_map()

    supervision_config = profile.build_supervision_config()
    assert (
        supervision_config["managed_nodes"]["pl_mapper"]["active_depend"]
        == {"pl_dir_computer": "active", "tf": "active", "mmwave": "active"}
    )
    assert "custom_operation" in entity_ids
    assert profile.service_dependencies()["custom_operation"] == {"micro_ros_agent": "ready"}


def test_opti_track_profile_runs_the_reduced_flight_basics_graph(monkeypatch):
    monkeypatch.delenv("III_MICRO_ROS_AGENT_COMMAND", raising=False)
    monkeypatch.delenv("III_MICRO_ROS_AGENT_UDP_PORT", raising=False)
    profile = get_system_profile("opti_track")

    assert [entity.entity_id for entity in profile.entities] == [
        "configuration_server",
        "trajectory_generator",
        "maneuver_controller",
        "rosbag_recorder",
        "mission_executor",
        "tf",
        "custom_operation",
    ]
    assert all("opti_track" in entity.profiles for entity in profile.entities)
    # The lab has no cable: payload drivers, perception, sensors and the
    # corridor overviews never start, mounted payload or not.
    assert {
        "charger_gripper",
        "cable_camera",
        "mmwave",
        "hough_transformer",
        "pl_dir_computer",
        "pl_mapper",
        "powerline_overview_provider",
        "pylon_overview_provider",
        "sim_assets",
    }.isdisjoint(profile.entity_map())
    assert [service.service_id for service in profile.services] == [
        "micro_ros_agent",
        "opti_track_pose_relay",
    ]
    assert profile.entity_map()["tf"].managed_node.node_name == "tf_real_launch_manager"
    # The physical flight controller's transport, not HIL's SITL port.
    assert profile.service_map()["micro_ros_agent"].command("opti_track").endswith("udp4 -p 8888")

    assert _lifecycle_edges(profile) == {
        "maneuver_controller": {
            "config_depend": {"trajectory_generator": "active"},
            "active_depend": {"trajectory_generator": "active", "tf": "active"},
        },
        "mission_executor": {
            "config_depend": {"maneuver_controller": "active", "rosbag_recorder": "active"},
        },
        "custom_operation": {"active_depend": {"mission_executor": "active"}},
    }
    # custom_operation activates through mission_executor and shares its gate.
    assert profile.service_dependencies() == {
        "mission_executor": {"micro_ros_agent": "ready", "opti_track_pose_relay": "ready"},
        "custom_operation": {"micro_ros_agent": "ready", "opti_track_pose_relay": "ready"},
    }


def test_opti_track_pose_relay_service_reads_the_profile_parameter_file(monkeypatch):
    resolved = []

    def resolve(profile_name):
        resolved.append(profile_name)
        return f"/config/iii_drone/parameter_sets/{profile_name}/tracked/default.yaml"

    monkeypatch.setattr(system_spec_module, "resolve_ros_params_file", resolve)
    relay = get_system_profile("opti_track").service_map()["opti_track_pose_relay"]

    assert relay.command("opti_track") == (
        "ros2 run iii_drone_core opti_track_pose_relay --ros-args "
        "--params-file /config/iii_drone/parameter_sets/opti_track/tracked/default.yaml"
    )
    assert resolved == ["opti_track"]
    assert relay.profiles == ("opti_track",)
    assert relay.autostart is True and relay.restart_on_exit is True
    assert relay.ready_timeout_sec == 120.0
    assert relay.px4_message_format_readiness == ()
    assert [
        (topic.topic, topic.message_type, topic.timeout_sec, topic.stable_for_sec)
        for topic in relay.readiness_topics
    ] == [("/opti_track/pose_relay/fresh", "std_msgs/msg/Header", 2.0, 2.0)]

    monkeypatch.setattr(system_spec_module, "resolve_ros_params_file", lambda _name: "/odd dir/p.yaml")
    assert relay.command("opti_track").endswith("--params-file '/odd dir/p.yaml'")


def test_opti_track_pose_relay_uses_the_same_parameter_file_as_the_entities(monkeypatch):
    monkeypatch.setattr(system_spec_module, "Node", lambda **kwargs: kwargs)
    monkeypatch.setattr(
        system_spec_module, "resolve_ros_params_file", lambda profile_name: f"/active/{profile_name}.yaml"
    )
    profile = get_system_profile("opti_track")
    mission_executor = profile.entity_map()["mission_executor"].launch_factory("opti_track")

    assert mission_executor["parameters"] == ["/active/opti_track.yaml", {"use_sim_time": False}]
    assert profile.service_map()["opti_track_pose_relay"].command("opti_track").endswith(
        "--params-file /active/opti_track.yaml"
    )


@pytest.mark.parametrize("profile_name", ["sim", "real", "hil"])
def test_cable_profile_graphs_are_unchanged_by_the_opti_track_graph(profile_name):
    profile = get_system_profile(profile_name)
    expected = _CABLE_PROFILE_GRAPHS[profile_name]

    assert [entity.entity_id for entity in profile.entities] == expected["entities"]
    assert _lifecycle_edges(profile) == expected["lifecycle_edges"]
    assert profile.service_dependencies() == {
        "mission_executor": {"micro_ros_agent": "ready"},
        "custom_operation": {"micro_ros_agent": "ready"},
    }
    assert [service.service_id for service in profile.services] == ["micro_ros_agent"]


@pytest.mark.parametrize("profile_name", _PROFILES)
def test_every_profile_validates_as_a_supervision_graph(profile_name):
    profile = get_system_profile(profile_name)

    Supervisor.validate_supervision_config(profile.build_supervision_config())
    assert set(profile.build_supervision_config()["managed_nodes"]) == set(profile.entity_map())


def test_profile_validation_rejects_a_dependency_on_a_node_the_profile_does_not_run():
    profile = SystemProfileSpec(
        name="opti_track",
        entities=(
            SystemEntitySpec(
                entity_id="maneuver_controller",
                launch_factory=lambda _profile_name: None,
                managed_node=ManagedNodeSpec(
                    node_name="maneuver_controller",
                    node_namespace="/control/maneuver_controller",
                    active_depend={"pl_mapper": "active"},
                ),
            ),
        ),
    )

    with pytest.raises(ValueError, match="'pl_mapper', which profile 'opti_track' does not run"):
        system_spec_module._validate_profile(profile)


def test_hil_profile_runs_pi_px4_tf_without_pi_sensors_or_gazebo(monkeypatch):
    monkeypatch.delenv("III_MICRO_ROS_AGENT_UDP_PORT", raising=False)
    profile = get_system_profile("hil")
    entity_ids = set(profile.entity_map())

    assert {"configuration_server", "tf", "pl_mapper", "mission_executor", "custom_operation"} <= entity_ids
    # The Pi publishes dynamic world->drone from PX4 odometry. Workstation
    # Gazebo must not publish a competing dynamic transform.
    assert profile.entity_map()["tf"].managed_node.node_name == "tf_real_launch_manager"
    assert {"cable_camera", "mmwave", "sim_assets", "charger_gripper"}.isdisjoint(entity_ids)
    assert all("hil" in entity.profiles for entity in profile.entities)
    assert profile.service_map()["micro_ros_agent"].command("hil") == "MicroXRCEAgent udp4 -p 8890"
    # HIL runs only the SITL agent. The physical PX4 transport is intentionally
    # excluded so it cannot consume Pi capacity or perturb the virtual mission.
    assert "micro_ros_agent_physical" not in profile.service_map()

    supervision_config = profile.build_supervision_config()
    assert supervision_config["managed_nodes"]["hough_transformer"]["active_depend"] == {
        "tf": "active",
    }
    assert supervision_config["managed_nodes"]["pl_mapper"]["active_depend"] == {
        "pl_dir_computer": "active",
        "tf": "active",
    }
    assert supervision_config["managed_nodes"]["pl_dir_computer"]["active_depend"] == {
        "hough_transformer": "active",
        "tf": "active",
    }
    assert supervision_config["managed_nodes"]["maneuver_controller"]["active_depend"]["tf"] == "active"
    assert "tf" in supervision_config["managed_nodes"]
    assert "charger_gripper" not in supervision_config["managed_nodes"]["mission_executor"]["config_depend"]


def test_onboard_nodes_use_sim_time_only_in_sim(monkeypatch):
    # HIL runs the onboard stack on wall time like the real drone; the 250 Hz
    # /clock cost each Pi node 4-5 % of a core. The configuration server
    # needs no sim time in any profile.
    monkeypatch.setattr(system_spec_module, "Node", lambda **kwargs: kwargs)
    monkeypatch.setattr(system_spec_module, "resolve_ros_params_file", lambda profile_name: f"{profile_name}.yaml")
    for profile_name, expected in (("sim", True), ("hil", False)):
        entities = get_system_profile(profile_name).entity_map()
        use_sim_time = {
            entity_id: entities[entity_id].launch_factory(profile_name)["parameters"][1]["use_sim_time"]
            for entity_id in ("configuration_server", "maneuver_controller", "mission_executor")
        }
        assert use_sim_time == {
            "configuration_server": False,
            "maneuver_controller": expected,
            "mission_executor": expected,
        }


def test_custom_operation_uses_sim_time_only_in_sim():
    # custom_operation shares reference-stream deadlines with the maneuver
    # controller and mission executor, so its time base follows the same
    # profile rule (SIMULATION stays true in HIL for the simulated radar).
    import re
    import subprocess
    from pathlib import Path

    import yaml

    config = Path(__file__).resolve().parents[1] / "node_management_config" / "custom_operation.yaml"
    command = yaml.safe_load(config.read_text())["command"]
    argument = re.search(r"use_sim_time:=(\S+ .*\))", command).group(1)
    for profile_name, expected in (("sim", "true"), ("hil", "false"), ("real", "false")):
        value = subprocess.run(
            ["bash", "-c", f"echo {argument}"],
            env={"III_SYSTEM_PROFILE": profile_name, "SIMULATION": "true", "PATH": "/usr/bin:/bin"},
            capture_output=True, text=True, check=True,
        ).stdout.strip()
        assert value == expected, (profile_name, value)


def test_hil_micro_ros_port_can_be_overridden(monkeypatch):
    monkeypatch.setenv("III_MICRO_ROS_AGENT_UDP_PORT", "9999")

    assert get_system_profile("hil").service_map()["micro_ros_agent"].command("hil") == "MicroXRCEAgent udp4 -p 9999"


def test_micro_ros_agent_binary_can_be_bound_to_host_tool(monkeypatch):
    monkeypatch.setenv(
        "III_MICRO_ROS_AGENT_BINARY",
        "/opt/iii/tools/micro-xrce-agent/bin/MicroXRCEAgent",
    )

    assert get_system_profile("hil").service_map()["micro_ros_agent"].command("hil") == (
        "/opt/iii/tools/micro-xrce-agent/bin/MicroXRCEAgent udp4 -p 8890"
    )


def test_launch_description_wraps_each_entity_in_log_directory_group(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR_BASE", str(tmp_path))
    monkeypatch.setenv("CONFIG_BASE_DIR", str(tmp_path / "config"))
    monkeypatch.setenv("III_OPERATIONS_ROOT", str(tmp_path / "operations"))

    profile = get_system_profile("sim")
    group = build_entity_launch_group(profile.name, profile.entities[0])

    assert isinstance(group, GroupAction)
    sub_entities = group.get_sub_entities()
    assert any(isinstance(entity, SetEnvironmentVariable) for entity in sub_entities)
    context = LaunchContext()
    environment = {
        perform_substitutions(context, entity.name): perform_substitutions(context, entity.value)
        for entity in sub_entities
        if isinstance(entity, SetEnvironmentVariable)
    }
    assert environment["III_SYSTEM_PROFILE"] == "sim"
    assert entity_log_dir("sim", profile.entities[0].entity_id) == Path(tmp_path) / "sim" / profile.entities[0].entity_id

    description = build_system_launch_description("sim")
    assert len(description.entities) == len(profile.entities)


def test_tmux_spec_only_references_entities_from_the_profile():
    profile = get_system_profile("sim")
    tmux_spec = get_tmux_session_spec("sim")
    known_entities = set(profile.entity_map()) | set(profile.service_map())

    assert tmux_spec.session_name == "iii_sim"
    window_names = [window.name for window in tmux_spec.windows]
    assert window_names.index("services") < window_names.index("background")
    assert any(
        window.name == "services"
        and any(pane.target == "micro_ros_agent" for pane in window.panes)
        for window in tmux_spec.windows
    )

    for window in tmux_spec.windows:
        for pane in window.panes:
            if pane.target is not None:
                assert pane.target in known_entities


@pytest.mark.parametrize(
    ("profile_name", "services"),
    [
        ("opti_track", ["micro_ros_agent", "opti_track_pose_relay"]),
        ("real", ["micro_ros_agent"]),
    ],
)
def test_tmux_services_window_follows_the_profile_services(profile_name, services):
    window = next(
        window for window in get_tmux_session_spec(profile_name).windows if window.name == "services"
    )

    assert [pane.target for pane in window.panes] == services


def test_tmux_spec_has_no_log_targets_unknown_to_every_profile():
    # logs_window() silently drops unknown targets, so a stale name (such as a
    # retired service) would never fail at runtime. Guard the source instead.
    import ast
    import inspect

    from iii_drone_supervision import tmux_spec

    known = set()
    for profile_name in ("sim", "hil", "real", "opti_track"):
        profile = get_system_profile(profile_name)
        known |= set(profile.entity_map()) | set(profile.service_map())
    targets = [
        argument.value
        for node in ast.walk(ast.parse(inspect.getsource(tmux_spec)))
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", None) == "logs_window"
        for argument in node.args[2:]
        if isinstance(argument, ast.Constant)
    ]
    assert targets
    assert sorted(set(targets) - known) == []


def test_tmux_spec_accepts_an_isolated_session_name(monkeypatch):
    monkeypatch.setenv("III_SYSTEM_TMUX_SESSION", "iii_sim_dataset28")

    assert get_tmux_session_spec("sim").session_name == "iii_sim_dataset28"
