"""Integration test for enhanced joint features (Safety & Calibration).

Verifies that these properties survive the conversion from Blender to Core.
"""

from __future__ import annotations

import bpy
import pytest
from linkforge.blender.adapters.blender_to_core import blender_joint_to_core


def test_enhanced_joint_conversion_roundtrip(clean_scene) -> None:
    """Verify that calibration and safety controller survive Blender to Core conversion."""
    # 1. Setup Links
    p = bpy.data.objects.new("Parent", None)
    c = bpy.data.objects.new("Child", None)
    bpy.context.collection.objects.link(p)
    bpy.context.collection.objects.link(c)
    p.linkforge.is_robot_link = True
    c.linkforge.is_robot_link = True

    # 2. Setup Joint with Enhanced Properties
    j = bpy.data.objects.new("Joint", None)
    bpy.context.collection.objects.link(j)
    j.linkforge_joint.is_robot_joint = True
    j.linkforge_joint.parent_link = p
    j.linkforge_joint.child_link = c
    j.linkforge_joint.joint_type = "REVOLUTE"

    # Safety Controller
    j.linkforge_joint.use_safety_controller = True
    j.linkforge_joint.safety_soft_lower_limit = -1.5
    j.linkforge_joint.safety_soft_upper_limit = 1.5
    j.linkforge_joint.safety_k_position = 200.0
    j.linkforge_joint.safety_k_velocity = 20.0

    # Calibration
    j.linkforge_joint.use_calibration = True
    j.linkforge_joint.use_calibration_rising = True
    j.linkforge_joint.calibration_rising = 0.75
    j.linkforge_joint.use_calibration_falling = True
    j.linkforge_joint.calibration_falling = -0.75

    # 3. Convert to Core
    core_joint = blender_joint_to_core(j)

    # 4. Verify
    assert core_joint.safety_controller is not None
    assert pytest.approx(core_joint.safety_controller.soft_lower_limit) == -1.5
    assert pytest.approx(core_joint.safety_controller.k_position) == 200.0

    assert core_joint.calibration is not None
    assert pytest.approx(core_joint.calibration.rising) == 0.75
    assert pytest.approx(core_joint.calibration.falling) == -0.75


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
