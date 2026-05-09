"""Unit tests for Blender Link operations, properties, and robustness."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.operators.link_ops import (
    create_collision_for_link,
    execute_collision_preview_update,
    regenerate_collision_mesh,
)

from tests.blender_test_utils import create_robot_link, safe_get_linkforge


@pytest.fixture(name="scene")
def scene_fixture(scene):
    """Ensure a clean scene for each test."""
    # Nuclear wipe of all data blocks
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    for block in list(bpy.data.meshes):
        bpy.data.meshes.remove(block, do_unlink=True)
    for block in list(bpy.data.actions):
        bpy.data.actions.remove(block, do_unlink=True)
    for coll in list(bpy.data.collections):
        if coll.name != "Scene Collection":
            bpy.data.collections.remove(coll)
    return scene


# =============================================================================
# Link Operations
# =============================================================================


class TestLinkOperations:
    def test_create_link_object(self, scene) -> None:
        """Test creating a link object (empty) in Blender."""
        link_obj = create_robot_link("test_link", scene)
        assert link_obj.name.startswith("test_link")
        assert link_obj.type == "EMPTY"
        assert safe_get_linkforge(link_obj).is_robot_link

    def test_create_collision_for_link(self, scene) -> None:
        """Test generating a primitive collision for a link."""
        link_obj = create_robot_link("link_with_collision", scene)

        # Add visual context for size detection
        bpy.ops.mesh.primitive_cube_add(size=1.0)
        vis = bpy.context.active_object
        vis.name = "link_visual"
        vis.parent = link_obj

        col_obj = create_collision_for_link(link_obj, "BOX", bpy.context)

        assert col_obj is not None
        assert col_obj.parent == link_obj
        assert "collision" in col_obj.name.lower()


# =============================================================================
# Link Properties
# =============================================================================


class TestLinkProperties:
    def test_link_name_sanitization(self, scene) -> None:
        """Test that link_name property sanitizes input."""
        bpy.ops.object.empty_add()
        obj = bpy.context.active_object
        assert obj is not None
        obj.name = "Original Name"
        safe_get_linkforge(obj).is_robot_link = True

        # Getter should return sanitized name
        assert safe_get_linkforge(obj).link_name == "Original_Name"

        # Setter should update object name
        safe_get_linkforge(obj).link_name = "New-Link-Name!"
        assert obj.name == "New-Link-Name_"

    def test_automatic_child_renaming(self, scene) -> None:
        """Test that renaming a link object also renames its children."""

        link_obj = create_robot_link("base_link", scene)

        # Create visual child
        bpy.ops.mesh.primitive_cube_add()
        vis_obj = bpy.context.active_object
        vis_obj.name = "base_link_visual"
        vis_obj.parent = link_obj

        # Rename the link
        safe_get_linkforge(link_obj).link_name = "chassis"

        assert link_obj.name == "chassis"
        assert vis_obj.name.startswith("chassis_visual")


# =============================================================================
# Robustness and Edge Cases
# =============================================================================


class TestLinkRobustness:
    def test_execute_collision_preview_update_branches(self, scene) -> None:
        """Test edge cases in collision preview update."""
        link_obj = create_robot_link("Link", scene)

        # Simulate missing view_layer context
        with patch("linkforge.blender.operators.link_ops.bpy") as mock_bpy:
            mock_bpy.data = bpy.data
            mock_bpy.context = MagicMock()
            mock_bpy.context.view_layer = None

            import linkforge.blender.operators.link_ops as link_ops

            link_ops._preview_pending_object = link_obj
            assert execute_collision_preview_update() is None

    def test_regenerate_collision_mesh_validation(self, scene) -> None:
        """Test validation in regenerate_collision_mesh."""
        # Passing non-link object should not crash
        bpy.ops.mesh.primitive_cube_add()
        bpy.context.active_object.name = "NotALink"
        obj = bpy.context.active_object
        regenerate_collision_mesh(obj, "AUTO", bpy.context)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
