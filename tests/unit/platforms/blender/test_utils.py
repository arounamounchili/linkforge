"""Unit tests for Blender Name synchronization, Filters, and Sanitization."""

from __future__ import annotations

import typing
from unittest.mock import MagicMock

import pytest
from linkforge.blender.adapters.mesh_io import export_link_mesh
from linkforge.blender.utils.decorators import OperatorReturn, safe_execute
from linkforge_core.exceptions import RobotModelError

from tests.blender_test_utils import (
    create_mesh_object,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_update,
)

# Name Synchronization and Persistence


class TestNameSynchronization:
    def test_link_name_tracks_object_rename(self, scene, blender_context) -> None:
        """Test that link_name auto-syncs when Blender renames the object.

        The name_sync_handler deliberately propagates obj.name → link_name
        so that robot identities stay consistent after Outliner renames.
        """
        obj = create_test_object("sync_link", None, scene)
        lf = safe_get_linkforge(obj)
        lf.is_robot_link = True
        lf.link_name = "sync_link"

        # Initial state: names should match
        assert lf.link_name == "sync_link"

        # Simulate Blender renaming — the handler should propagate the new name
        obj.name = "sync_link_renamed"
        safe_update(scene)

        # The handler should have updated link_name to match the new obj.name
        assert safe_get_linkforge(obj).link_name == "sync_link_renamed"

    def test_joint_name_tracks_object_rename(self, scene, blender_context) -> None:
        """Test that joint_name auto-syncs when Blender renames the object.

        The name_sync_handler deliberately propagates obj.name → joint_name
        so that joint identities stay consistent after Outliner renames.
        """
        obj = create_test_object("sync_joint", None, scene)
        jf = safe_get_joint(obj)
        jf.is_robot_joint = True
        jf.joint_name = "sync_joint"

        # Initial state: names should match
        assert jf.joint_name == "sync_joint"

        # Simulate Blender renaming — the handler should propagate the new name
        obj.name = "sync_joint_renamed"
        safe_update(scene)

        # The handler should have updated joint_name to match the new obj.name
        assert safe_get_joint(obj).joint_name == "sync_joint_renamed"


# Sanitization and Fidelity


class TestSanitization:
    def test_filename_sanitization(self, tmp_path, scene, blender_context) -> None:
        """Verify that filename sanitization ensures compatibility."""
        obj = create_mesh_object("cube_sanitization", scene)

        p, _ = export_link_mesh(
            obj=obj,
            link_name="my link.001",
            geometry_type="visual",
            mesh_format="STL",
            meshes_dir=tmp_path,
            dry_run=True,
        )
        assert p is not None
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
        def my_op(s: typing.Any, c: typing.Any) -> OperatorReturn:
            return {"FINISHED"}

        assert my_op(mock_self, None) == {"FINISHED"}
        assert len(mock_self.reports) == 0

    def test_safe_execute_failure(self, scene, blender_context) -> None:
        """Test error handling in a decorated function."""
        mock_self = MagicMock()
        mock_self.reports = []
        mock_self.report = lambda t, m: mock_self.reports.append((t, m))

        @safe_execute
        def failing_op(s: typing.Any, c: typing.Any) -> OperatorReturn:
            raise RobotModelError("Fail")

        assert failing_op(mock_self, None) == {"CANCELLED"}
        assert "Fail" in mock_self.reports[0][1]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
