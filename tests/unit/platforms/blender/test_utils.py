"""Unit tests for Blender Name synchronization, Filters, and Sanitization."""

from __future__ import annotations

from unittest.mock import MagicMock

import bpy
import pytest
from linkforge.blender.adapters.mesh_io import export_link_mesh
from linkforge.blender.utils.decorators import safe_execute
from linkforge_core.exceptions import RobotModelError

from tests.blender_test_utils import (
    safe_get_joint,
    safe_get_linkforge,
)

# =============================================================================
# Name Synchronization and Persistence
# =============================================================================


class TestNameSynchronization:
    def test_link_name_persistence(self, scene) -> None:
        """Test that link_name remains synced even if Blender renames the object."""
        bpy.ops.object.empty_add()
        bpy.context.active_object.name = "base_link"
        obj = bpy.context.active_object
        safe_get_linkforge(obj).is_robot_link = True
        safe_get_linkforge(obj).link_name = "base_link"

        # Simulate Blender renaming (e.g., adding a suffix)
        obj.name = "base_link.001"
        bpy.context.view_layer.update()

        # Getter should return the synced name (likely sanitized)
        assert safe_get_linkforge(obj).link_name == "base_link_001"

    def test_joint_name_persistence(self, scene) -> None:
        """Test that joint_name remains synced even if Blender renames the object."""
        bpy.ops.object.empty_add()
        bpy.context.active_object.name = "joint"
        obj = bpy.context.active_object
        safe_get_joint(obj).is_robot_joint = True
        safe_get_joint(obj).joint_name = "joint"

        obj.name = "joint.002"
        bpy.context.view_layer.update()
        assert safe_get_joint(obj).joint_name == "joint_002"


# =============================================================================
# Sanitization and Fidelity
# =============================================================================


class TestSanitization:
    def test_filename_sanitization(self, tmp_path, scene) -> None:
        """Verify that filename sanitization ensures compatibility."""
        bpy.ops.mesh.primitive_cube_add()
        obj = bpy.context.active_object

        p, _ = export_link_mesh(
            obj=obj,
            link_name="my link.001",
            geometry_type="visual",
            mesh_format="STL",
            meshes_dir=tmp_path,
        )
        assert "my_link_001" in p.name
        assert " " not in p.name


# =============================================================================
# Decorators
# =============================================================================


class TestDecorators:
    def test_safe_execute_success(self, scene) -> None:
        """Test successful execution of a decorated function."""
        mock_self = MagicMock()
        mock_self.reports = []
        mock_self.report = lambda t, m: mock_self.reports.append((t, m))

        @safe_execute
        def my_op(s, c):
            return {"FINISHED"}

        assert my_op(mock_self, None) == {"FINISHED"}
        assert len(mock_self.reports) == 0

    def test_safe_execute_failure(self, scene) -> None:
        """Test error handling in a decorated function."""
        mock_self = MagicMock()
        mock_self.reports = []
        mock_self.report = lambda t, m: mock_self.reports.append((t, m))

        @safe_execute
        def failing_op(s, c):
            raise RobotModelError("Fail")

        assert failing_op(mock_self, None) == {"CANCELLED"}
        assert "Fail" in mock_self.reports[0][1]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
