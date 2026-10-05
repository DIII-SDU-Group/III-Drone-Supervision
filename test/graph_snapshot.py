"""Canonical, environment-independent description of a system-spec graph (legacy regression lock)."""

from __future__ import annotations

import json


def canonical_graph(spec_module, profile_name: str) -> dict:
    """Entities (launch kwargs, lifecycle metadata, respawn, profiles), services and supervision config of a profile.

    ``spec_module`` must have ``Node``, ``resolve_ros_params_file`` and ``resolve_node_management_config`` patched to
    deterministic stand-ins by the caller.
    """
    return canonical_profile(spec_module.get_system_profile(profile_name))


def canonical_profile(profile) -> dict:
    entities = []
    for entity in profile.entities:
        managed = entity.managed_node
        entities.append({
            "entity_id": entity.entity_id,
            "launch": json.loads(json.dumps(entity.launch_factory(profile.name), sort_keys=True, default=str)),
            "managed_node": None if managed is None else {
                "node_name": managed.node_name, "node_namespace": managed.node_namespace,
                "config_depend": dict(sorted(managed.config_depend.items())),
                "active_depend": dict(sorted(managed.active_depend.items())),
                "service_depend": dict(sorted(managed.service_depend.items())),
            },
            "service_depend": dict(sorted(entity.service_depend.items())),
            "respawn": entity.respawn,
            "profiles": list(entity.profiles),
        })
    services = [{"service_id": service.service_id, "readiness_topics": [t.__dict__ for t in service.readiness_topics],
                 "autostart": service.autostart, "restart_on_exit": service.restart_on_exit,
                 "restart_delay_sec": service.restart_delay_sec, "stop_timeout_sec": service.stop_timeout_sec,
                 "ready_timeout_sec": service.ready_timeout_sec, "profiles": list(service.profiles)}
                for service in profile.services]
    return {"name": profile.name, "entities": entities, "services": services,
            "supervision_config": profile.build_supervision_config()}


def patch_stand_ins(monkeypatch, spec_module) -> None:
    monkeypatch.setattr(spec_module, "Node", lambda **kwargs: kwargs)
    monkeypatch.setattr(spec_module, "resolve_ros_params_file", lambda profile_name: f"<active parameter file:{profile_name}>")
    monkeypatch.setattr(spec_module, "resolve_node_management_config", lambda filename: f"<node management config:{filename}>")
