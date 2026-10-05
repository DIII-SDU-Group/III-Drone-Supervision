"""The canonical /perception/processing_stack selection between the legacy and powerline_slam perception stacks."""

import json
from pathlib import Path
import sys

import pytest
import yaml

import iii_drone_supervision.system_spec as spec
from iii_drone_supervision.system_manager import SystemManager
from iii_drone_supervision.system_spec import (
    LEGACY_PERCEPTION_ENTITIES,
    LEGACY_STACK,
    ManagedNodeSpec,
    POWERLINE_SLAM_SENSOR_LAYOUT,
    POWERLINE_SLAM_STACK,
    SystemEntitySpec,
    SystemProfileSpec,
    _validate_profile,
    get_system_profile,
    resolve_system_profile,
)
from iii_drone_supervision.tmux_spec import get_tmux_session_spec

sys.path.insert(0, str(Path(__file__).resolve().parent))
from graph_snapshot import canonical_graph, canonical_profile, patch_stand_ins  # noqa: E402

SNAPSHOT = json.loads((Path(__file__).resolve().parent / "resources" / "legacy_graph_186ac916.json").read_text())
CONSUMERS = ("maneuver_controller", "powerline_overview_provider", "mission_executor")


def _powerline_slam_profile(profile_name="sim"):
    return get_system_profile(profile_name, POWERLINE_SLAM_STACK, sensor_layout=POWERLINE_SLAM_SENSOR_LAYOUT)


@pytest.mark.parametrize("profile_name", ["sim", "real", "opti_track", "hil"])
def test_legacy_graph_equals_the_entering_canonical_graph(monkeypatch, profile_name):
    patch_stand_ins(monkeypatch, spec)
    assert canonical_graph(spec, profile_name) == SNAPSHOT["profiles"][profile_name]


def test_legacy_is_the_default_stack():
    for profile_name in ("sim", "real", "opti_track", "hil"):
        profile = get_system_profile(profile_name)
        assert profile.processing_stack == LEGACY_STACK
        assert set(LEGACY_PERCEPTION_ENTITIES) <= set(profile.entity_map())
        assert "powerline_slam" not in profile.entity_map()


def test_powerline_slam_graph_replaces_only_the_legacy_perception_stack(monkeypatch):
    patch_stand_ins(monkeypatch, spec)
    legacy = canonical_profile(get_system_profile("sim"))
    selected = canonical_profile(_powerline_slam_profile())
    legacy_entities = {e["entity_id"]: e for e in legacy["entities"]}
    selected_entities = {e["entity_id"]: e for e in selected["entities"]}
    assert set(selected_entities) == (set(legacy_entities) - set(LEGACY_PERCEPTION_ENTITIES)) | {"powerline_slam"}

    node = selected_entities["powerline_slam"]
    assert node["launch"] == {"arguments": [], "executable": "powerline_slam_node", "name": "powerline_slam",
                              "namespace": "/perception/powerline_slam", "output": "log",
                              "package": "iii_drone_powerline_slam",
                              "parameters": ["<active parameter file:sim>", {"use_sim_time": True}],
                              "respawn": True, "respawn_delay": 2.0, "ros_arguments": []}
    assert node["managed_node"] == {"node_name": "powerline_slam", "node_namespace": "/perception/powerline_slam",
                                    "config_depend": {}, "active_depend": {"sim_assets": "active", "tf": "active"},
                                    "service_depend": {}}
    assert node["profiles"] == ["sim"]

    for entity_id, entity in selected_entities.items():
        if entity_id == "powerline_slam":
            continue
        expected = json.loads(json.dumps(legacy_entities[entity_id]))
        if entity_id in CONSUMERS:
            for kind in ("config_depend", "active_depend"):
                expected["managed_node"][kind].pop("pl_mapper", None)
        assert entity == expected, entity_id
    assert selected["services"] == legacy["services"]

    for node_id, node in selected["supervision_config"]["managed_nodes"].items():
        for kind in ("config_depend", "active_depend"):
            dependencies = set(node.get(kind, {}))
            assert "powerline_slam" not in dependencies, f"{node_id} must not depend on powerline_slam"
            assert not dependencies & set(LEGACY_PERCEPTION_ENTITIES), node_id


@pytest.mark.parametrize("profile_name", ["real", "opti_track", "hil"])
def test_powerline_slam_is_refused_outside_the_sim_profile(profile_name):
    with pytest.raises(ValueError, match="only valid for the sim runtime profile"):
        _powerline_slam_profile(profile_name)


@pytest.mark.parametrize("layout", [None, "d4s_dc_drone"])
def test_powerline_slam_requires_the_powerline_eval_sensor_layout(layout):
    with pytest.raises(ValueError, match="requires /tf/sim/sensor_layout = d4s_dc_drone_powerline_eval"):
        get_system_profile("sim", POWERLINE_SLAM_STACK, sensor_layout=layout)


def test_an_unknown_stack_is_refused():
    with pytest.raises(ValueError, match="is not one of legacy, powerline_slam"):
        get_system_profile("sim", "hough_only")


def _active_file(tmp_path, monkeypatch, parameters):
    path = tmp_path / "active.yaml"
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
    monkeypatch.setattr(spec, "resolve_ros_params_file", lambda _profile_name: str(path))
    return path


def test_resolution_reads_the_constant_from_the_active_parameter_file(tmp_path, monkeypatch):
    _active_file(tmp_path, monkeypatch, {"/perception/processing_stack": "powerline_slam",
                                         "/tf/sim/sensor_layout": POWERLINE_SLAM_SENSOR_LAYOUT})
    profile = resolve_system_profile("sim")
    assert profile.processing_stack == POWERLINE_SLAM_STACK
    assert profile.sensor_layout == POWERLINE_SLAM_SENSOR_LAYOUT
    assert "powerline_slam" in profile.entity_map() and "pl_mapper" not in profile.entity_map()
    with pytest.raises(ValueError, match="only valid for the sim runtime profile"):
        resolve_system_profile("hil")        # HIL shares the sim parameter family but is not the sim runtime profile


def test_resolution_defaults_to_legacy(tmp_path, monkeypatch):
    _active_file(tmp_path, monkeypatch, {"/tf/sim/sensor_layout": "d4s_dc_drone"})
    assert resolve_system_profile("sim").processing_stack == LEGACY_STACK


def test_boot_fails_before_launching_anything(tmp_path, monkeypatch):
    import iii_drone_supervision.system_manager as system_manager_module
    _active_file(tmp_path, monkeypatch, {"/perception/processing_stack": "powerline_slam", "/tf/sim/sensor_layout": "d4s_dc_drone"})
    monkeypatch.setattr(system_manager_module, "resolve_system_profile", spec.resolve_system_profile)
    manager = SystemManager.__new__(SystemManager)
    manager._booted = False
    manager._profile_name = None
    manager._launch_service = None
    manager._launch_generation = 0
    manager._lock = __import__("threading").Lock()
    manager._ensure_ros_runtime = lambda: None
    with pytest.raises(ValueError, match="requires /tf/sim/sensor_layout"):
        manager.boot("sim")
    assert manager._launch_service is None and manager._launch_generation == 0 and not manager._booted


def test_the_manager_uses_the_stack_latched_at_boot():
    manager = SystemManager.__new__(SystemManager)
    manager._profile_name = "sim"
    manager._processing_stack = POWERLINE_SLAM_STACK
    manager._sensor_layout = POWERLINE_SLAM_SENSOR_LAYOUT
    nodes = manager.managed_node_ids()
    assert "powerline_slam" in nodes and not set(LEGACY_PERCEPTION_ENTITIES) & set(nodes)
    manager._processing_stack = None
    assert set(LEGACY_PERCEPTION_ENTITIES) <= set(manager.managed_node_ids())


def test_the_tmux_perception_window_follows_the_stack():
    def perception_targets(session):
        window = next(w for w in session.windows if w.name == "perception")
        return [pane.target for pane in window.panes]
    assert perception_targets(get_tmux_session_spec("sim")) == list(LEGACY_PERCEPTION_ENTITIES)
    assert perception_targets(get_tmux_session_spec("sim", POWERLINE_SLAM_STACK,
                                                    sensor_layout=POWERLINE_SLAM_SENSOR_LAYOUT)) == ["powerline_slam"]


def test_an_unknown_lifecycle_dependency_fails_profile_validation():
    entity = SystemEntitySpec(entity_id="orphan", launch_factory=lambda _profile_name: None,
                              managed_node=ManagedNodeSpec("orphan", "/orphan", active_depend={"missing": "active"}))
    with pytest.raises(ValueError, match="not part of the sim/legacy graph"):
        _validate_profile(SystemProfileSpec(name="sim", entities=(entity,)))
