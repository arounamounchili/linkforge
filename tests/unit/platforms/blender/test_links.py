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

# Link Operations


class TestLinkOperations:
    def test_create_link_object(self, scene, blender_context) -> None:
        """Test creating a link object (empty) in Blender."""
        link_obj = create_robot_link("test_link", scene)
        assert link_obj.name.startswith("test_link")
        assert link_obj.type == "EMPTY"
        assert safe_get_linkforge(link_obj).is_robot_link

    def test_create_collision_no_geometry(self, scene, blender_context) -> None:
        """Test robustness when creating collision for link with no geometry."""
        link_obj = create_robot_link("empty_link", scene)

        # No children, no geometry
        col_obj = create_collision_for_link(link_obj, "BOX", bpy.context)
        assert link_obj.type == "EMPTY"
        assert safe_get_linkforge(link_obj).is_robot_link

    def test_create_collision_for_link(self, scene, blender_context) -> None:
        """Test generating a primitive collision for a link."""

        link_obj = create_robot_link("link_with_collision", scene)

        # Add visual context for size detection
        from tests.blender_test_utils import create_mesh_object

        vis = create_mesh_object("link_visual", scene)
        vis.parent = link_obj

        col_obj = create_collision_for_link(link_obj, "BOX", bpy.context)

        assert col_obj is not None
        assert col_obj.parent == link_obj
        assert "collision" in col_obj.name.lower()


# Link Properties


class TestLinkProperties:
    def test_link_property_persistence(self, scene, blender_context) -> None:
        """Test setting and getting link forge properties."""
        from tests.blender_test_utils import create_test_object

        obj = create_test_object("test_props", None, scene)
        props = safe_get_linkforge(obj)
        assert props is not None
        obj.name = "Original Name"
        safe_get_linkforge(obj).is_robot_link = True

        # Getter should return sanitized name
        assert safe_get_linkforge(obj).link_name == "Original_Name"

        # Setter should update object name
        safe_get_linkforge(obj).link_name = "New-Link-Name!"
        assert obj.name == "New-Link-Name_"

    def test_automatic_child_renaming(self, scene, blender_context) -> None:
        """Test that renaming a link object also renames its children."""
        from tests.blender_test_utils import create_test_object

        link_obj = create_robot_link("base_link", scene)

        # Create visual child
        vis_obj = create_test_object("base_link_visual", None, scene)
        vis_obj.parent = link_obj

        # Rename the link
        safe_get_linkforge(link_obj).link_name = "chassis"

        assert link_obj.name == "chassis"
        assert vis_obj.name.startswith("chassis_visual")


# Link Utilities


class TestLinkRobustness:
    def test_execute_collision_preview_update_branches(self, scene, blender_context) -> None:
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

    def test_regenerate_collision_mesh_validation(self, scene, blender_context) -> None:
        """Test validation in regenerate_collision_mesh."""
        # Passing non-link object should not crash
        from tests.blender_test_utils import create_test_object

        obj = create_test_object("NotALink", None, scene)
        regenerate_collision_mesh(obj, "AUTO", bpy.context)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
