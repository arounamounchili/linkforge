"""Unit tests for Blender Import/Export operations and Robot validation."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.operators.export_ops import (
    LINKFORGE_OT_export_robot_model,
    LINKFORGE_OT_validate_robot,
)

from tests.blender_test_utils import (
    safe_get_linkforge_scene,
)

# Robot Validation Operator


class TestRobotValidation:
    def test_validate_robot_success(self, mocker, scene, blender_context) -> None:
        """Test the validate_robot operator in a success scenario."""
        mock_self = MagicMock()

        # Mock validation result
        val_res = MagicMock()
        val_res.is_valid = True
        val_res.has_warnings = False
        mocker.patch("linkforge_core.validation.RobotValidator.validate", return_value=val_res)
        mocker.patch(
            "linkforge.blender.adapters.blender_to_core.scene_to_robot",
            return_value=(MagicMock(), {}),
        )

        result = LINKFORGE_OT_validate_robot.execute(mock_self, bpy.context)
        assert result == {"FINISHED"}

    def test_validate_robot_failure(self, mocker, scene, blender_context) -> None:
        """Test the validate_robot operator in a failure scenario."""
        mock_self = MagicMock()

        # Mock validation result with errors
        val_res = MagicMock()
        val_res.is_valid = False
        val_res.error_count = 1
        val_res.errors = [MagicMock(message="Test Error")]
        mocker.patch("linkforge_core.validation.RobotValidator.validate", return_value=val_res)
        mocker.patch(
            "linkforge.blender.adapters.blender_to_core.scene_to_robot",
            return_value=(MagicMock(), {}),
        )

        result = LINKFORGE_OT_validate_robot.execute(mock_self, bpy.context)
        # Should return CANCELLED if there are validation errors
        assert result == {"CANCELLED"}


# Robot Export Operator


class TestRobotExport:
    def test_export_urdf_extension_correction(self, mocker, scene, blender_context) -> None:
        """Verify automatic correction of file extensions based on export format."""
        props = safe_get_linkforge_scene(scene)
        props.export_format = "URDF"
        props.validate_before_export = False

        mock_self = MagicMock()
        mock_self.filepath = "/tmp/robot.xacro"  # Wrong extension
        mock_self.report = MagicMock()

        mocker.patch(
            "linkforge.blender.adapters.blender_to_core.scene_to_robot",
            return_value=(MagicMock(), {}),
        )
        mocker.patch(
            "linkforge_core.generators.urdf_generator.URDFGenerator.generate", return_value="<xml/>"
        )

        LINKFORGE_OT_export_robot_model.execute(mock_self, bpy.context)
        # Should be corrected to .urdf
        assert mock_self.filepath.endswith(".urdf")

    def test_export_invoke_branches(self, scene, blender_context) -> None:
        """Test invoke branches (format to extension mapping)."""
        mock_op = MagicMock(spec=LINKFORGE_OT_export_robot_model)

        safe_get_linkforge_scene(scene).export_format = "XACRO"
        with patch("bpy_extras.io_utils.ExportHelper.invoke", return_value={"FINISHED"}):
            LINKFORGE_OT_export_robot_model.invoke(mock_op, bpy.context, MagicMock())
            assert mock_op.filename_ext == ".xacro"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
