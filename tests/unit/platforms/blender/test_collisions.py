"""Unit tests for Blender Collision alignment, quality, and scaling."""

from __future__ import annotations

from typing import cast

import bpy
import pytest
from bpy.types import DecimateModifier
from linkforge.blender.operators.link_ops import (
    create_collision_for_link,
)

from tests.blender_test_utils import (
    create_mesh_object,
    create_test_object,
    safe_get_linkforge,
    safe_update,
)


class TestCollisionAlignment:
    def test_collision_alignment_on_rotated_link(self, scene, blender_context) -> None:
        """Verify that generating collision for a rotated link avoids offsets."""
        link_obj = create_test_object("link_obj", None, scene=scene)
        safe_get_linkforge(link_obj).is_robot_link = True
        link_obj.rotation_euler = (1.5708, 0, 0)  # 90 deg X

        # Add a Visual mesh
        visual_obj = create_mesh_object("part_visual", scene=scene)
        visual_obj.parent = link_obj
        visual_obj.matrix_parent_inverse.identity()

        # Generate Collision
        collision_obj = create_collision_for_link(link_obj, "MESH", bpy.context)

        assert collision_obj is not None
        assert collision_obj.parent == link_obj
        # Local transform should be near identity
        assert collision_obj.location.length < 1e-5
        assert collision_obj.rotation_euler.x < 1e-5


class TestCollisionQuality:
    def test_collision_modifier_persistence(self, scene, blender_context) -> None:
        """Verify that generating mesh collision preserves Decimate modifier."""
        link_obj = create_mesh_object("link_obj", scene=scene)
        safe_get_linkforge(link_obj).is_robot_link = True

        safe_get_linkforge(link_obj).collision_quality = 50.0
        create_collision_for_link(link_obj, "MESH", bpy.context)

        collision_obj = next(c for c in link_obj.children if "_collision" in c.name)
        decimate_mod = cast(
            DecimateModifier, next(m for m in collision_obj.modifiers if m.type == "DECIMATE")
        )
        assert decimate_mod.ratio == 0.5


class TestCollisionScaling:
    def test_box_collision_scaling(self, scene, blender_context) -> None:
        """Verify that a scaled cube results in a matching collision primitive."""
        link_obj = create_mesh_object("scaled_link", scene=scene, with_cube=True)
        link_obj.scale = (2.0, 1.5, 0.5)
        safe_update(scene)

        safe_get_linkforge(link_obj).is_robot_link = True

        collision_obj = create_collision_for_link(link_obj, "BOX", bpy.context)
        assert collision_obj is not None
        # Dimensions should be 4x3x1
        assert abs(collision_obj.dimensions.x - 4.0) < 1e-5
        assert abs(collision_obj.dimensions.y - 3.0) < 1e-5
        assert abs(collision_obj.dimensions.z - 1.0) < 1e-5


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
