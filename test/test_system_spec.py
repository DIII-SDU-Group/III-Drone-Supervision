from pathlib import Path

from launch import LaunchContext
from launch.actions import GroupAction, SetEnvironmentVariable
from launch.utilities import perform_substitutions

from iii_drone_supervision.system_spec import (
    build_entity_launch_group,
    build_system_launch_description,
    entity_log_dir,
    get_system_profile,
)
from iii_drone_supervision.tmux_spec import get_tmux_session_spec


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


def test_opti_track_profile_contains_custom_operation():
    profile = get_system_profile("opti_track")

    assert all("opti_track" in entity.profiles for entity in profile.entities)
    assert "custom_operation" in set(profile.entity_map())
    assert profile.service_dependencies()["custom_operation"] == {"micro_ros_agent": "ready"}


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
