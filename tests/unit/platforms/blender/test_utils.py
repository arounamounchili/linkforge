"""Unit tests for Blender Name synchronization, Filters, and Sanitization."""

from __future__ import annotations

from unittest.mock import MagicMock

import bpy
import pytest
from linkforge.blender.adapters.mesh_io import export_link_mesh
from linkforge.blender.utils.decorators import safe_execute
from linkforge_core.exceptions import RobotModelError

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
)

# Name Synchronization and Persistence


class TestNameSynchronization:
    def test_link_name_persistence(self, scene, blender_context) -> None:
        """Test that link_name remains synced even if Blender renames the object."""
        obj = create_test_object("base_link", None, scene)
        safe_get_linkforge(obj).is_robot_link = True
        safe_get_linkforge(obj).link_name = "base_link"

        # Simulate Blender renaming (e.g., adding a suffix)
        obj.name = "base_link.001"
        bpy.context.view_layer.update()

        # Getter should return the synced name (likely sanitized)
        assert safe_get_linkforge(obj).link_name == "base_link_001"

    def test_joint_name_persistence(self, scene, blender_context) -> None:
        """Test that joint_name remains synced even if Blender renames the object."""
        obj = create_test_object("joint", None, scene)
        safe_get_joint(obj).is_robot_joint = True
        safe_get_joint(obj).joint_name = "joint"

        obj.name = "joint.002"
        bpy.context.view_layer.update()
        assert safe_get_joint(obj).joint_name == "joint_002"


# Sanitization and Fidelity


class TestSanitization:
    def test_filename_sanitization(self, tmp_path, scene, blender_context) -> None:
        """Verify that filename sanitization ensures compatibility."""
        obj = create_test_object("cube_sanitization", None, scene)

        p, _ = export_link_mesh(
            obj=obj,
            link_name="my link.001",
            geometry_type="visual",
            mesh_format="STL",
            meshes_dir=tmp_path,
        )
        assert "my_link_001" in p.name
        assert " " not in p.name


# Decorators


class TestDecorators:
    def test_safe_execute_success(self, scene, blender_context) -> None:
        """Test successful execution of a decorated function."""
        mock_self = MagicMock()
        mock_self.reports = []
        mock_self.report = lambda t, m: mock_self.reports.append((t, m))

        @safe_execute
        def my_op(s, c):
            return {"FINISHED"}

        assert my_op(mock_self, None) == {"FINISHED"}
        assert len(mock_self.reports) == 0

    def test_safe_execute_failure(self, scene, blender_context) -> None:
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
