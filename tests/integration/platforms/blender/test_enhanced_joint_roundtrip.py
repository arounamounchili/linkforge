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

    # Use bpy.data.scenes[0] if context.scene is None (common in background tests)
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None
    collection.objects.link(p)
    collection.objects.link(c)

    # Use getattr to satisfy the linter for dynamic Blender properties
    from typing import Any

    p_lf: Any = getattr(p, "linkforge")
    c_lf: Any = getattr(c, "linkforge")
    p_lf.is_robot_link = True
    c_lf.is_robot_link = True

    # 2. Setup Joint with Enhanced Properties
    j = bpy.data.objects.new("Joint", None)
    collection.objects.link(j)

    j_lf: Any = getattr(j, "linkforge_joint")
    j_lf.is_robot_joint = True
    j_lf.parent_link = p
    j_lf.child_link = c
    j_lf.joint_type = "REVOLUTE"

    # Safety Controller
    j_lf.use_safety_controller = True
    j_lf.safety_soft_lower_limit = -1.5
    j_lf.safety_soft_upper_limit = 1.5
    j_lf.safety_k_position = 200.0
    j_lf.safety_k_velocity = 20.0

    # Calibration
    j_lf.use_calibration = True
    j_lf.use_calibration_rising = True
    j_lf.calibration_rising = 0.75
    j_lf.use_calibration_falling = True
    j_lf.calibration_falling = -0.75

    # 3. Convert to Core
    core_joint = blender_joint_to_core(j)
    assert core_joint is not None, "Failed to convert Blender joint to Core"

    # 4. Verify
    assert core_joint.safety_controller is not None
    assert pytest.approx(core_joint.safety_controller.soft_lower_limit) == -1.5
    assert pytest.approx(core_joint.safety_controller.k_position) == 200.0

    assert core_joint.calibration is not None
    assert pytest.approx(core_joint.calibration.rising) == 0.75
    assert pytest.approx(core_joint.calibration.falling) == -0.75


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
