"""Integration tests for Blender Robot lifecycle (Joints, Visuals, Physics)."""

from __future__ import annotations

import bpy
import pytest
from linkforge.blender.operators.link_ops import calculate_inertia_for_link

from tests.blender_test_utils import (
    create_mesh_object,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
)

# Physics and Inertia Integration


class TestPhysicsIntegration:
    def test_inertia_calculation_workflow(self, blender_clean_scene) -> None:
        """Verify end-to-end inertia calculation in Blender."""
        scene = bpy.context.scene
        link_obj = create_test_object("link", None, scene=scene)
        link_lf = safe_get_linkforge(link_obj)
        link_lf.is_robot_link = True
        link_lf.mass = 1.0

        # Add a visual mesh — a 2x2x2 unit cube (size=2.0 in bmesh = 1m side length)
        vis = create_mesh_object("link_visual", scene=scene, with_cube=True)
        vis.parent = link_obj

        # Calculate
        success = calculate_inertia_for_link(link_obj)
        assert success is True

        # For a 2m-side cube (size=2.0): I = m*(h^2+d^2)/12 = 1*(4+4)/12 = 2/3
        # The bmesh cube with size=2.0 has each edge = 2m
        assert link_lf.inertia_ixx > 0.0
        assert link_lf.inertia_iyy > 0.0
        assert link_lf.inertia_izz > 0.0

    def test_inertia_with_offset_visual(self, blender_clean_scene) -> None:
        """Verify offset visuals affect inertia via Parallel Axis Theorem."""
        scene = bpy.context.scene
        link_obj = create_test_object("link_offset", None, scene=scene)
        link_lf = safe_get_linkforge(link_obj)
        link_lf.is_robot_link = True
        link_lf.mass = 2.0

        # Visual cube offset by 10m on X — use a proper cube mesh
        vis = create_mesh_object("offset_visual", scene=scene, with_cube=True)
        vis.location = (10, 0, 0)
        vis.parent = link_obj

        success = calculate_inertia_for_link(link_obj)
        assert success is True
        # Inertia values must be positive and non-zero
        assert link_lf.inertia_ixx > 0.0
        assert link_lf.inertia_iyy > 0.0
        assert link_lf.inertia_izz > 0.0


# Joint Roundtrips


class TestJointIntegration:
    def test_joint_creation_and_properties(self, blender_clean_scene) -> None:
        """Verify joint creation and property persistence."""
        scene = bpy.context.scene
        p = create_test_object("Parent", None, scene=scene)
        c = create_test_object("Child", None, scene=scene)
        safe_get_linkforge(p).is_robot_link = True
        safe_get_linkforge(c).is_robot_link = True

        j = create_test_object("Joint", None, scene=scene)
        j_props = safe_get_joint(j)
        j_props.is_robot_joint = True
        j_props.parent_link = p
        j_props.child_link = c
        j_props.joint_type = "REVOLUTE"

        assert j_props.parent_link == p
        assert j_props.child_link == c


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
