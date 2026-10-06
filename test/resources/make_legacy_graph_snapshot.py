#!/usr/bin/env python3
"""Write the legacy-graph regression snapshot of a Supervision commit's system_spec (default: the upstream baseline
last merged into powerline-slam, 51cc9a6).

    python3 test/resources/make_legacy_graph_snapshot.py [COMMIT] > test/resources/legacy_graph_<commit>.json
"""
from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from graph_snapshot import canonical_graph  # noqa: E402

commit = sys.argv[1] if len(sys.argv) > 1 else "51cc9a67911a023da917ce94b5ea14ebde8b18ed"
source = subprocess.check_output(["git", "show", f"{commit}:iii_drone_supervision/system_spec.py"], cwd=HERE.parents[1])
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / "entering_system_spec.py"
    path.write_bytes(source)
    spec = importlib.util.spec_from_file_location("entering_system_spec", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["entering_system_spec"] = module      # dataclasses resolve annotations through sys.modules
    spec.loader.exec_module(module)
module.Node = lambda **kwargs: kwargs
module.resolve_ros_params_file = lambda profile_name: f"<active parameter file:{profile_name}>"
module.resolve_node_management_config = lambda filename: f"<node management config:{filename}>"
snapshot = {"supervision_commit": subprocess.check_output(["git", "rev-parse", commit], cwd=HERE.parents[1], text=True).strip(),
            "profiles": {name: canonical_graph(module, name) for name in ("sim", "real", "opti_track", "hil")}}
print(json.dumps(snapshot, indent=1, sort_keys=True))
