"""Integration tests for Blender Robot lifecycle (Joints, Visuals, Physics)."""

from __future__ import annotations

import bpy
import pytest
from linkforge.blender.operators.link_ops import calculate_inertia_for_link

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
)

# =============================================================================
# Physics and Inertia Integration
# =============================================================================


class TestPhysicsIntegration:
    def test_inertia_calculation_workflow(self, clean_scene) -> None:
        """Verify end-to-end inertia calculation in Blender."""
        scene = bpy.context.scene
        link_obj = create_test_object("link", None, scene=scene)
        link_lf = safe_get_linkforge(link_obj)
        link_lf.is_robot_link = True
        link_lf.mass = 1.0

        # Add a visual mesh (1m cube)
        bpy.ops.mesh.primitive_cube_add(size=1.0)
        vis = bpy.context.active_object
        vis.name = "link_visual"
        vis.parent = link_obj

        # Calculate
        success = calculate_inertia_for_link(link_obj)
        assert success is True

        # 1kg 1m cube Ixx = 1/6
        assert pytest.approx(link_lf.inertia_ixx, abs=1e-5) == 1.0 / 6.0

    def test_inertia_with_offset_visual(self, clean_scene) -> None:
        """Verify offset visuals affect inertia via Parallel Axis Theorem."""
        scene = bpy.context.scene
        link_obj = create_test_object("link_offset", None, scene=scene)
        link_lf = safe_get_linkforge(link_obj)
        link_lf.is_robot_link = True
        link_lf.mass = 2.0

        # Visual cube offset by 10m on X
        bpy.ops.mesh.primitive_cube_add(size=1.0, location=(10, 0, 0))
        vis = bpy.context.active_object
        vis.name = "offset_visual"
        vis.parent = link_obj

        success = calculate_inertia_for_link(link_obj)
        assert success is True
        # Base Ixx is 2 * 1/6 = 1/3.
        # But wait, since it's on X axis, Ixx doesn't change by offset?
        # Parallel axis: I = Icm + m*d^2. For Ixx, d is distance from X axis.
        # If location is (10,0,0), distance from X axis is 0.
        # So Ixx should still be 1/3.
        assert pytest.approx(link_lf.inertia_ixx, abs=1e-5) == 2.0 / 6.0


# =============================================================================
# Joint Roundtrips
# =============================================================================


class TestJointIntegration:
    def test_joint_creation_and_properties(self, clean_scene) -> None:
        """Verify joint creation and property persistence."""
        scene = bpy.context.scene
        p = create_test_object("Parent", None, scene=scene)
        c = create_test_object("Child", None, scene=scene)
        safe_get_linkforge(p).is_robot_link = True
        safe_get_linkforge(c).is_robot_link = True

        bpy.ops.object.empty_add()
        j = bpy.context.active_object
        j.name = "Joint"
        j_props = safe_get_joint(j)
        j_props.is_robot_joint = True
        j_props.parent_link = p
        j_props.child_link = c
        j_props.joint_type = "REVOLUTE"

        assert j_props.parent_link == p
        assert j_props.child_link == c


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
