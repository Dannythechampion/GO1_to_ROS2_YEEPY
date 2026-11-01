"""Read and write the mission file the operator records spots into.

The packaged `config/tape_mission.yaml` is only a template: an installed share
directory is rebuilt on every deploy, so recorded poses written there would be
lost. The live file sits outside the workspace instead -- `GO1_MISSION_FILE`,
or `~/.ros/go1_tape_mission.yaml` -- and is seeded from the template the first
time a spot is recorded.
"""

from __future__ import annotations

import os
from pathlib import Path

import yaml

from omx_navigation.mission import Mission, mission_from_dict, mission_to_dict

ENVIRONMENT_VARIABLE = "GO1_MISSION_FILE"
DEFAULT_PATH = "~/.ros/go1_tape_mission.yaml"

HEADER = """\
# Written by `ros2 run omx_navigation tape_waypoint record <name>`.
#
# Each waypoint is a taped spot in the `map` frame: the midpoint between its
# two wall strips, and the robot's heading squared to the corridor. A waypoint
# with `marker_separation` was recorded at its tape; one without is a
# placeholder, and tape_mission refuses to dock there.
#
# Re-record a spot whenever its tape moves or the map is rebuilt.
"""


def default_mission_path() -> Path:
    return Path(os.path.expanduser(os.environ.get(ENVIRONMENT_VARIABLE) or DEFAULT_PATH))


def load_mission(path) -> Mission:
    with open(path, encoding="utf-8") as handle:
        return mission_from_dict(yaml.safe_load(handle))


def save_mission(path, mission: Mission) -> None:
    """Write atomically, so a crash mid-write never leaves half a mission."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    body = yaml.safe_dump(mission_to_dict(mission), sort_keys=False, default_flow_style=False)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(HEADER + "\n" + body, encoding="utf-8")
    os.replace(temporary, path)
