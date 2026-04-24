from pathlib import Path

from launch.actions import GroupAction, SetEnvironmentVariable

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

    assert {"configuration_server", "sensors", "tf", "pl_mapper", "mission_executor"} <= entity_ids

    supervision_config = profile.build_supervision_config()

    assert "configuration_server" in supervision_config["managed_nodes"]
    assert (
        supervision_config["managed_nodes"]["hough_transformer"]["active_depend"]
        == {"sensors": "active", "tf": "active"}
    )
    assert "configuration_server" not in supervision_config["managed_nodes"]["trajectory_generator"].get(
        "config_depend", {}
    )


def test_real_profile_contains_hardware_entities():
    profile = get_system_profile("real")
    entity_ids = set(profile.entity_map())

    assert {"cable_camera", "mmwave", "tf"} <= entity_ids

    supervision_config = profile.build_supervision_config()
    assert (
        supervision_config["managed_nodes"]["pl_mapper"]["active_depend"]
        == {"pl_dir_computer": "active", "tf": "active", "mmwave": "active"}
    )


def test_launch_description_wraps_each_entity_in_log_directory_group(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR_BASE", str(tmp_path))

    profile = get_system_profile("sim")
    group = build_entity_launch_group(profile.name, profile.entities[0])

    assert isinstance(group, GroupAction)
    sub_entities = group.get_sub_entities()
    assert any(isinstance(entity, SetEnvironmentVariable) for entity in sub_entities)
    assert entity_log_dir("sim", profile.entities[0].entity_id) == Path(tmp_path) / "sim" / profile.entities[0].entity_id

    description = build_system_launch_description("sim")
    assert len(description.entities) == len(profile.entities)


def test_tmux_spec_only_references_entities_from_the_profile():
    profile = get_system_profile("sim")
    tmux_spec = get_tmux_session_spec("sim")
    known_entities = set(profile.entity_map())

    assert tmux_spec.session_name == "iii_sim"

    for window in tmux_spec.windows:
        for pane in window.panes:
            if pane.target is not None:
                assert pane.target in known_entities
