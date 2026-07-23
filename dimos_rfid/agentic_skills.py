# Copyright 2026. RFID DimOS integration — agent skills without the web UI.

"""Agent skill containers used by RFID agentic blueprints.

DimOS's upstream ``_common_agentic`` also includes ``WebInput``, which imports
``dimos.web.dimos_interface``. That package is present in the DimOS git tree but
missing from the PyPI ``dimos`` wheel (0.0.12.post2), so importing it fails.

Text commands do **not** need the web UI: ``dimos agent-send "…"`` goes through
``McpServer.agent_send`` → LCM ``/human_input`` → ``McpClient``.
"""

from __future__ import annotations

import os

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.core.coordination.blueprints import autoconnect
from dimos.perception.spatial_perception import SpatialMemory
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer

# DimOS `unitree_go2_agentic` sits on `unitree_go2_spatial` (SpatialMemory +
# skills). Our RFID semantic stack uses plain `unitree_go2`, so we add
# SpatialMemory here. Same skill set as `_common_agentic`, minus WebInput.
#
# SpeakSkill is optional: its start() always constructs OpenAI TTS and crashes
# if OPENAI_API_KEY is unset. Include it only when a key is present.
_parts = [
    SpatialMemory.blueprint(),
    NavigationSkillContainer.blueprint(),
    PersonFollowSkillContainer.blueprint(camera_info=GO2Connection.camera_info_static),
    UnitreeSkillContainer.blueprint(),
]

if os.environ.get("OPENAI_API_KEY", "").strip():
    from dimos.agents.skills.speak_skill import SpeakSkill

    _parts.append(SpeakSkill.blueprint())

rfid_agentic_skills = autoconnect(*_parts)

__all__ = ["rfid_agentic_skills"]
